"""Export and numerically verify the exact SP deployment tensor, without NMS."""
import hashlib
import importlib.metadata
import json
from pathlib import Path
import numpy as np
import torch
from .model import build_model


def _compile(xml):
    import openvino as ov
    return ov.Core().compile_model(str(xml),"CPU",{"INFERENCE_PRECISION_HINT":"f32","INFERENCE_NUM_THREADS":2})


def verify_base(base_model, output=None):
    torch.set_num_threads(2); model=build_model(base_model).eval(); compiled=_compile(base_model)
    reports=[]
    for seed in (0,2027):
        x=np.random.default_rng(seed).random((1,3,640,640),dtype=np.float32)
        with torch.no_grad(): ref=model(torch.from_numpy(x)).numpy()
        actual=compiled([x])[compiled.output(0)]
        error=np.abs(ref-actual)
        if actual.shape!=(1,25200,22) or not np.allclose(ref,actual,atol=.005,rtol=.0001):
            raise AssertionError(f"IR/PyTorch parity failed: max_error={error.max()}")
        reports.append({"seed":seed,"max_abs_error":float(error.max()),"mean_abs_error":float(error.mean())})
    result={"ir_sha256":model.ir_sha256,"shape":[1,25200,22],"parameters":sum(p.numel() for p in model.parameters()),"fp32_parity":reports}
    if output:
        Path(output).parent.mkdir(parents=True,exist_ok=True); Path(output).write_text(json.dumps(result,indent=2)+"\n")
    return result


def export_model(checkpoint, destination, base_model, allow_placeholder=False):
    import onnx
    import openvino as ov
    torch.set_num_threads(2)
    c=torch.load(checkpoint,map_location="cpu",weights_only=True)
    if c["schema"]!="sp-armor-checkpoint-v1": raise ValueError("unknown checkpoint schema")
    if (c["allow_placeholder"] or c["smoke_max_steps"] is not None) and not allow_placeholder:
        raise ValueError("smoke/placeholder checkpoint; use --allow-placeholder only for pipeline verification")
    out=Path(destination).resolve()
    if out.exists() and any(out.iterdir()): raise FileExistsError("export directory is nonempty")
    model=build_model(base_model).eval()
    if model.ir_sha256!=c["ir_sha256"]: raise ValueError("base graph mismatch")
    model.load_state_dict(c["model"]); out.mkdir(parents=True,exist_ok=True)
    x=torch.from_numpy(np.random.default_rng(2027).random((1,3,640,640),dtype=np.float32))
    with torch.no_grad():
        torch.onnx.export(model,x,str(out/"armor.onnx"),opset_version=17,input_names=["images"],output_names=["output"],do_constant_folding=True)
    onnx.checker.check_model(onnx.load(str(out/"armor.onnx")))
    converted=ov.convert_model(str(out/"armor.onnx"))
    ov.save_model(converted,str(out/"armor.xml"),compress_to_fp16=False)
    compiled=_compile(out/"armor.xml"); checks=[]
    for seed in (0,2027):
        inp=np.random.default_rng(seed).random((1,3,640,640),dtype=np.float32)
        with torch.no_grad(): expected=model(torch.from_numpy(inp)).numpy()
        result=compiled([inp])[compiled.output(0)]; error=np.abs(expected-result)
        if result.shape!=(1,25200,22) or not np.isfinite(result).all() or not np.allclose(expected,result,atol=.005,rtol=.0001):
            raise AssertionError(f"export parity failure: max={error.max()}")
        checks.append({"seed":seed,"max_abs_error":float(error.max()),"mean_abs_error":float(error.mean())})
    metadata={"schema":"sp-armor-export-v1","input":{"shape":[1,3,640,640],"dtype":"float32","color":"RGB","scale":"1/255","padding":"top-left; black right/bottom"},
              "output":{"shape":[1,25200,22],"points":"TL BL BR TR in input pixels","objectness":"raw logit at 8","color_logits":"9:13 blue red gray purple","number_logits":"13:22 sentry 1 2 3 4 5 outpost Bs Bb","nms":False},
              "base_ir_sha256":model.ir_sha256,"dataset_sha256":c["dataset_sha256"],"epoch":c["epoch"],"placeholder_or_smoke":c["allow_placeholder"] or c["smoke_max_steps"] is not None,
              "requires_2027_geometry_and_class_mapping_review":True,"parity":checks,
              "versions":{p:importlib.metadata.version(p) for p in ("torch","numpy","onnx","openvino")},
              "files_sha256":{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file()}}
    (out/"metadata.json").write_text(json.dumps(metadata,indent=2)+"\n")
    # Fragment for a COPY of the active config; never rewrite the live runtime config.
    (out/"deployment-overlay.yaml").write_text("# Merge into a copy of the runtime YAML; review 2027 geometry/class filtering first.\nyolo_name: yolov5\nyolov5_model_path: "+json.dumps(str(out/"armor.xml"))+"\nuse_traditional: false\n")
    return metadata
