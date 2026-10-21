import argparse
import importlib.util
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
if not (ROOT/"assets/base_model/yolov5.xml").is_file():
    ROOT=Path(sys.prefix)/"share/sp-armor-training"


def main(argv=None):
    p=argparse.ArgumentParser(description="SP armor synthetic data / exact-model fine-tuning")
    sub=p.add_subparsers(dest="command",required=True)
    q=sub.add_parser("pipeline",help="plan the local building-stage pipeline")
    q.add_argument("--config",default=str(ROOT/"configs/pipeline.json"));q.add_argument("--execute",action="store_true")
    d=sub.add_parser("doctor"); d.add_argument("--output")
    g=sub.add_parser("generate"); g.add_argument("--asset",default=str(ROOT/"configs/placeholder.json")); g.add_argument("--output",required=True)
    g.add_argument("--count",type=int,default=64); g.add_argument("--seed",type=int,default=2027); g.add_argument("--shard",type=int,default=0)
    g.add_argument("--width",type=int,default=640); g.add_argument("--height",type=int,default=480); g.add_argument("--backgrounds")
    for name in ("validate","prepare"):
        q=sub.add_parser(name); q.add_argument("--data",required=True); q.add_argument("--allow-placeholder",action="store_true")
    q=sub.add_parser("preview"); q.add_argument("--data",required=True); q.add_argument("--output",required=True); q.add_argument("--count",type=int,default=12)
    q=sub.add_parser("import"); q.add_argument("--source",required=True); q.add_argument("--output",required=True)
    q=sub.add_parser("verify-base"); q.add_argument("--base-model",default=str(ROOT/"assets/base_model/yolov5.xml")); q.add_argument("--output")
    q=sub.add_parser("train"); q.add_argument("--data",required=True); q.add_argument("--config",default=str(ROOT/"configs/train.json")); q.add_argument("--run",required=True)
    q.add_argument("--base-model",default=str(ROOT/"assets/base_model/yolov5.xml")); q.add_argument("--device",default="cuda:0"); q.add_argument("--resume")
    q.add_argument("--allow-placeholder",action="store_true"); q.add_argument("--max-steps",type=int)
    q=sub.add_parser("export"); q.add_argument("--checkpoint",required=True); q.add_argument("--output",required=True)
    q.add_argument("--base-model",default=str(ROOT/"assets/base_model/yolov5.xml")); q.add_argument("--allow-placeholder",action="store_true")
    q=sub.add_parser("evaluate"); q.add_argument("--data",required=True); q.add_argument("--checkpoint",required=True); q.add_argument("--split",choices=["val","test"],default="test")
    q.add_argument("--device",default="cuda:0"); q.add_argument("--base-model",default=str(ROOT/"assets/base_model/yolov5.xml")); q.add_argument("--allow-placeholder",action="store_true"); q.add_argument("--output")
    args=p.parse_args(argv); result=None
    if args.command=="pipeline":
        from .pipeline import run_pipeline
        result=run_pipeline(args.config,args.execute)
    elif args.command=="doctor":
        import platform,shutil
        result={"python":platform.python_version(),"packages":{k:bool(importlib.util.find_spec(k)) for k in ("numpy","cv2","torch","onnx","openvino")},"blender":shutil.which("blender"),"base_model":(ROOT/"assets/base_model/yolov5.bin").is_file()}
        if result["packages"]["torch"]:
            import torch
            result.update(torch=torch.__version__,cuda=torch.cuda.is_available(),gpus=torch.cuda.device_count())
        if args.output: Path(args.output).write_text(json.dumps(result,indent=2)+"\n")
    elif args.command=="generate":
        from .synthetic import generate
        generate(args.output,args.asset,args.count,args.seed,args.shard,args.width,args.height,args.backgrounds); result={"generated":args.count,"output":args.output}
    elif args.command in ("validate","prepare"):
        from .dataset import validate_dataset,prepare
        result=prepare(args.data,args.allow_placeholder) if args.command=="prepare" else validate_dataset(args.data,args.allow_placeholder)[1]
    elif args.command=="preview":
        from .dataset import preview
        preview(args.data,args.output,args.count); result={"preview":args.output}
    elif args.command=="import":
        from .dataset import import_frames
        result=import_frames(args.source,args.output)
    elif args.command=="verify-base":
        from .export import verify_base
        result=verify_base(args.base_model,args.output)
    elif args.command=="train":
        from .train import run
        run(args.data,args.config,args.run,args.base_model,args.device,args.resume,args.allow_placeholder,args.max_steps)
    elif args.command=="export":
        from .export import export_model
        result=export_model(args.checkpoint,args.output,args.base_model,args.allow_placeholder)
    elif args.command=="evaluate":
        import torch
        from torch.utils.data import DataLoader
        from .dataset import ArmorDataset,collate
        from .model import build_model
        from .train import evaluate
        torch.set_num_threads(2); c=torch.load(args.checkpoint,map_location="cpu",weights_only=True); m=build_model(args.base_model)
        if m.ir_sha256!=c["ir_sha256"]: raise ValueError("checkpoint/base model mismatch")
        if c["allow_placeholder"] and not args.allow_placeholder: raise ValueError("placeholder checkpoint requires --allow-placeholder")
        m.load_state_dict(c["model"]); m.to(args.device)
        result=evaluate(m,DataLoader(ArmorDataset(args.data,args.split,args.allow_placeholder),batch_size=1,collate_fn=collate),args.device)
        if args.output: Path(args.output).write_text(json.dumps(result,indent=2)+"\n")
    if result is not None: print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=="__main__": main()
