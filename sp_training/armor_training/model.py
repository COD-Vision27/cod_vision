"""Recover the *existing* fused MobileNetV3 YOLOv5 graph as a trainable PyTorch FX graph.

No guessed replacement architecture: weights, branches, anchors, grid and output order
come directly from the repository's OpenVINO IR. Fused BN stays fused during fine-tuning.
Only the operators actually present in this model are accepted; unknown IRs fail closed.
"""
import hashlib
import operator
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import torch
from torch import nn, fx
from torch.nn import functional as F
from .contract import ANCHORS, STRIDES


def ints(value):
    return tuple(int(x.strip()) for x in value.split(",") if x.strip())


def reshape_batch(x, shape):
    # All data Reshapes in the bundled graph preserve the leading batch axis.
    shape = tuple(x.shape[i] if v == 0 else v for i,v in enumerate(shape))
    if shape[0] == 1:
        shape = (x.shape[0], *shape[1:])
    return x.reshape(shape)


def grouped_conv(x, w, stride, padding, dilation):
    groups, out_per_group, in_per_group, kh, kw = w.shape
    return F.conv2d(x, w.reshape(groups*out_per_group, in_per_group, kh, kw), stride=stride,
                    padding=padding, dilation=dilation, groups=groups)


def build_model(xml_path):
    xml_path=Path(xml_path); blob=xml_path.with_suffix(".bin").read_bytes()
    tree=ET.parse(xml_path).getroot()
    layers={int(l.attrib["id"]):l for l in tree.find("layers")}
    incoming={i:{} for i in layers}
    for edge in tree.find("edges"):
        a=edge.attrib; incoming[int(a["to-layer"])][int(a["to-port"])]=int(a["from-layer"])
    inputs={i:[v for _,v in sorted(e.items())] for i,e in incoming.items()}
    const={}
    for i,l in layers.items():
        if l.attrib["type"]=="Const":
            d=l.find("data").attrib
            dt={"f16":"<f2","f32":"<f4","i64":"<i8","i32":"<i4"}[d["element_type"]]
            v=np.frombuffer(blob,dtype=dt,offset=int(d["offset"]),count=int(d["size"])//np.dtype(dt).itemsize).copy().reshape(ints(d["shape"]))
            const[i]=v.astype(np.float32) if v.dtype.kind=="f" else v
    # The loss assumes these exact anchors/strides and raw logits. Reject an incompatible base head.
    for i, (scale_id, grid_id) in enumerate(((526,529),(588,591),(650,653))):
        expected=np.ones((1,3,1,1,22),dtype=np.float32)
        expected[...,:8]=np.tile(np.asarray(ANCHORS[i],np.float32).reshape(1,3,1,1,2),(1,1,1,1,4))
        size=640//STRIDES[i]; grid=np.zeros((1,1,size,size,22),np.float32)
        yy,xx=np.meshgrid(np.arange(size),np.arange(size),indexing="ij")
        grid[0,0,...,:8]=np.tile(np.stack([xx,yy],-1),(1,1,4))*STRIDES[i]
        if scale_id not in const or grid_id not in const or not np.array_equal(const[scale_id],expected) or not np.array_equal(const[grid_id],grid):
            raise ValueError("base model anchor/grid contract does not match this training head")
    def constant_id(i):
        while layers[i].attrib["type"]=="Convert": i=inputs[i][0]
        return i if i in const else None
    def constant_value(i):
        c=constant_id(i)
        if c is None: raise ValueError("expected constant IR operand")
        return const[c]
    trainable=set()
    for i,l in layers.items():
        op=l.attrib["type"]
        if op in ("Convolution","GroupConvolution","MatMul"):
            trainable.add(constant_id(inputs[i][1]))
        if op=="Add" and any(layers[j].attrib["type"] in ("Convolution","GroupConvolution","MatMul") for j in inputs[i]):
            trainable.update(c for j in inputs[i] if (c:=constant_id(j)) is not None)
    if None in trainable: raise ValueError("dynamic weights unsupported")
    module=nn.Module(); g=fx.Graph(); nodes={}; results=[]
    for i,l in layers.items():
        op=l.attrib["type"]; d=l.find("data"); d=d.attrib if d is not None else {}
        args=[nodes[j] for j in inputs[i]]
        if op=="Parameter":
            if d.get("shape")!="1,3,640,640": raise ValueError("expected existing 640x640 SP model")
            n=g.placeholder("images")
        elif op=="Const":
            name=f"ir_{i}"
            if i in trainable: module.register_parameter(name,nn.Parameter(torch.from_numpy(const[i])))
            else: module.register_buffer(name,torch.from_numpy(const[i]))
            n=g.get_attr(name)
        elif op=="Convert":
            if d["destination_type"]!="f32": raise ValueError("only fp32 decompression supported")
            n=args[0]  # FP16 constants were promoted once when loaded.
        elif op in ("Convolution","GroupConvolution"):
            padding=ints(d["pads_begin"])
            if padding!=ints(d["pads_end"]): raise ValueError("asymmetric conv padding unsupported")
            fn=F.conv2d if op=="Convolution" else grouped_conv
            if op=="Convolution":
                n=g.call_function(fn,tuple(args),{"stride":ints(d["strides"]),"padding":padding,"dilation":ints(d["dilations"])})
            else: n=g.call_function(fn,(*args,ints(d["strides"]),padding,ints(d["dilations"])))
        elif op in ("Add","Multiply"):
            n=g.call_function(operator.add if op=="Add" else operator.mul,tuple(args))
        elif op in ("ReLU","HSwish","HSigmoid","Swish"):
            if op=="Swish" and len(args)>1 and float(constant_value(inputs[i][1]))!=1: raise ValueError("non-unit Swish beta")
            n=g.call_function({"ReLU":F.relu,"HSwish":F.hardswish,"HSigmoid":F.hardsigmoid,"Swish":F.silu}[op],(args[0],))
        elif op=="ReduceMean":
            n=g.call_function(torch.mean,(args[0],),{"dim":tuple(int(v) for v in constant_value(inputs[i][1]).flat),"keepdim":d["keep_dims"]=="true"})
        elif op=="MatMul":
            for j,flag in enumerate(("transpose_a","transpose_b")):
                if d.get(flag)=="true": args[j]=g.call_method("transpose",(args[j],-1,-2))
            n=g.call_function(torch.matmul,tuple(args))
        elif op=="Reshape":
            n=g.call_function(reshape_batch,(args[0],tuple(int(v) for v in constant_value(inputs[i][1]).flat)))
        elif op=="Transpose":
            n=g.call_method("permute",(args[0],*tuple(int(v) for v in constant_value(inputs[i][1]).flat)))
        elif op=="Concat": n=g.call_function(torch.cat,(args,),{"dim":int(d["axis"])})
        elif op=="Interpolate":
            if d.get("mode")!="nearest" or d.get("coordinate_transformation_mode")!="asymmetric" or d.get("nearest_mode")!="floor":
                raise ValueError("unsupported resize semantics")
            scale=tuple(float(v) for v in constant_value(inputs[i][-1]).flat)
            if len(scale)==4: scale=scale[-2:]
            n=g.call_function(F.interpolate,(args[0],),{"scale_factor":scale,"mode":"nearest"})
        elif op=="Result": results.append(args[0]); n=args[0]
        else: raise ValueError(f"unsupported OpenVINO operator: {op}")
        nodes[i]=n
    if len(results)!=1: raise ValueError("one output required")
    g.output(results[0]); g.eliminate_dead_code(); g.lint()
    model=fx.GraphModule(module,g)
    model.ir_sha256=hashlib.sha256(xml_path.read_bytes()+blob).hexdigest()
    return model
