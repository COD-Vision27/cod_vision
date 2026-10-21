"""Building-stage local DAG. Planning is local-safe; execution stays on the current machine."""
import json
import os
import subprocess
import sys
from pathlib import Path


def load_plan(config):
    config=Path(config).resolve(); root=config.parent.parent; c=json.loads(config.read_text())
    if c.get("schema")!="sp-armor-pipeline-v1": raise ValueError("unknown pipeline schema")
    execution=c["execution"]
    if c["training"].get("initialization")!="project_ir_weights":
        raise ValueError("only project_ir_weights initialization is currently wired; alternative training initialization is a building-stage task")
    if execution["location"]!="local" or execution["allow_local_fallback"] is not False:
        raise ValueError("rendering/training run on the local machine")
    if execution["train_device"]=="cpu" or execution["render_device"]=="CPU":
        raise ValueError("production pipeline requires configured local GPU backends")
    if c["sampling"]["chunk_size"]<1 or c["sampling"]["views_per_scene"]<1 or c["sampling"]["scenes"]<1:
        raise ValueError("invalid sampling counts")
    def path(value):
        return str((root/value).resolve()) if value else None
    registry=path(c["assets"]["registry"])
    reasons=[]
    scene=path(c["assets"]["scene_blend"])
    if not scene: reasons.append("2027 scene/armor geometry has not been supplied")
    elif not Path(scene).is_file(): reasons.append("scene file is unavailable on this host")
    if not registry or not Path(registry).is_file(): reasons.append("asset registry is unavailable")
    else:
        r=json.loads(Path(registry).read_text())
        if r.get("schema")!="sp-armor-registry-v1": raise ValueError("unknown registry schema")
        if r.get("status")!="verified" or not r.get("armors"):
            reasons.append("2027 appearance, landmark binding and classes await reference assets")
    raw=path(c["artifacts"]["raw"]); dataset=path(c["artifacts"]["dataset"])
    run=path(c["artifacts"]["training_run"]); export=path(c["artifacts"]["export"])
    train=[sys.executable,"-m","armor_training","train","--data",dataset,"--config",path(c["training"]["config"]),"--run",run,
           "--base-model",path(c["training"]["base_model"]),"--device",execution["train_device"]]
    if c["training"]["resume"]: train.extend(["--resume",path(c["training"]["resume"])])
    stages=[
        {"id":"render_and_label","command":["blenderproc","run",str(Path(__file__).with_name("blenderproc_worker.py")),"--config",str(config)],"produces":"RGB + synchronized scene ground truth"},
        {"id":"dataset_import","command":[sys.executable,"-m","armor_training","import","--source",raw,"--output",dataset],"produces":"project-specific four-point YOLO labels"},
        {"id":"dataset_prepare","command":[sys.executable,"-m","armor_training","prepare","--data",dataset],"produces":"scene-separated train/val/test manifest"},
        {"id":"train","command":train,"produces":"project-compatible model checkpoint"},
        {"id":"export","command":[sys.executable,"-m","armor_training","export","--checkpoint",str(Path(run)/"best.pt"),"--output",export,"--base-model",path(c["training"]["base_model"])],"produces":"ONNX/OpenVINO with unchanged output contract"}
    ]
    return {"phase":c["phase"],"execution":"local_only","ready_to_execute":not reasons,"waiting_for":reasons,"stages":stages},root,c


def run_pipeline(config, execute=False):
    plan,root,c=load_plan(config)
    if not execute: return plan
    if os.environ.get("SP_ARMOR_EXECUTION_ROLE")!=c["execution"]["required_role"]:
        raise RuntimeError("execution is restricted to the explicitly enabled local training process")
    if plan["waiting_for"]: raise RuntimeError("pipeline awaits real assets: "+"; ".join(plan["waiting_for"]))
    env=dict(os.environ,PYTHONPATH=str(root))
    for stage in plan["stages"]:
        print(json.dumps({"stage":stage["id"],"state":"starting"}),flush=True)
        subprocess.run(stage["command"],cwd=root,env=env,check=True)
    return {"phase":"local_job_finished","stages":[s["id"] for s in plan["stages"]]}
