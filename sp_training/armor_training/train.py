"""Single GPU/CPU and torchrun DDP training, resumable checkpoints and holdout metrics."""
import hashlib
import json
import math
import os
import random
from pathlib import Path
import cv2
import numpy as np
import torch
import torch.distributed as dist
from torch.nn.parallel import DistributedDataParallel
from torch.utils.data import DataLoader, DistributedSampler
from .dataset import ArmorDataset, collate
from .loss import armor_loss
from .model import build_model


def metric_counts(pred, targets, threshold=.5):
    # TP requires IoU>=.5, correct number/color, and <10% diagonal mean corner error.
    stats=np.zeros(8,float)  # tp, fp, fn, bbox matches, color correct, num correct, corner px sum, objects
    for rows,truth in zip(pred.detach().float().cpu().numpy(),targets):
        truth=truth.cpu().numpy(); scores=1/(1+np.exp(-np.clip(rows[:,8],-80,80)))
        keep=np.flatnonzero(scores>=threshold); p=rows[keep,:8].reshape(-1,4,2)
        boxes=np.c_[p.min(1),p.max(1)-p.min(1)] if len(p) else np.empty((0,4))
        indices=np.asarray(cv2.dnn.NMSBoxes(boxes.tolist(),scores[keep].tolist(),threshold,.3)).reshape(-1) if len(p) else []
        chosen=keep[indices] if len(indices) else []; used=set(); tp=0
        for index in sorted(chosen,key=lambda i:-scores[i]):
            q=rows[index,:8].reshape(4,2); lo,hi=q.min(0),q.max(0); best=None
            for j,t in enumerate(truth):
                if j in used: continue
                gt=t[2:].reshape(4,2); a,b=gt.min(0),gt.max(0)
                inter=np.maximum(0,np.minimum(hi,b)-np.maximum(lo,a)).prod()
                iou=inter/max(1e-9,np.maximum(hi-lo,0).prod()+(b-a).prod()-inter)
                if iou>=.5 and (best is None or iou>best[0]): best=(iou,j,gt)
            if best is None: continue
            _,j,gt=best; used.add(j); error=np.linalg.norm(q-gt,axis=1).mean()
            color=int(rows[index,9:13].argmax())==int(truth[j,0]); number=int(rows[index,13:22].argmax())==int(truth[j,1])
            stats[3:7]+=[1,color,number,error]
            tp+=bool(color and number and error<=.1*np.linalg.norm(gt.max(0)-gt.min(0)))
        stats[0]+=tp; stats[1]+=len(chosen)-tp; stats[2]+=len(truth)-tp; stats[7]+=len(truth)
    return stats


def summarize(stats):
    tp,fp,fn,m,c,n,e,objects=stats
    return {"precision":float(tp/max(tp+fp,1)),"recall":float(tp/max(tp+fn,1)),"bbox_matches":int(m),
            "color_accuracy":float(c/max(m,1)),"number_accuracy":float(n/max(m,1)),"mean_corner_error_px":float(e/max(m,1)),"objects":int(objects)}


def evaluate(model, loader, device, max_steps=None):
    model.eval(); losses=[]; stats=np.zeros(8,float)
    with torch.no_grad():
        for step,(images,targets) in enumerate(loader):
            if max_steps is not None and step>=max_steps: break
            images=images.to(device); targets=[t.to(device) for t in targets]
            output=model(images); loss,_=armor_loss(output,targets); losses.append(loss.item())
            stats+=metric_counts(output,targets)
    if not losses: raise ValueError("empty validation loader")
    return {"loss":float(np.mean(losses)),**summarize(stats)}


def save_checkpoint(path, payload):
    temporary=path.with_suffix(".tmp")
    torch.save(payload,temporary); temporary.replace(path)


def run(data, config, run_dir, base_model, device="cpu", resume=None, allow_placeholder=False, max_steps=None):
    c=json.loads(Path(config).read_text()); out=Path(run_dir)
    if max_steps is not None and max_steps<1: raise ValueError("max_steps must be positive")
    for k in ("epochs","batch_size","threads"):
        if type(c[k]) is not int or c[k]<1: raise ValueError(f"invalid {k}")
    if c["workers"]<0 or c["learning_rate"]<=0: raise ValueError("invalid workers/lr")
    torch.set_num_threads(c["threads"])
    world=int(os.environ.get("WORLD_SIZE","1")); rank=int(os.environ.get("RANK","0")); local=int(os.environ.get("LOCAL_RANK","0"))
    distributed=world>1
    if distributed:
        if device=="cpu": raise ValueError("DDP training requires --device cuda")
        device=f"cuda:{local}"; torch.cuda.set_device(local); dist.init_process_group("nccl")
    device=torch.device(device)
    if device.type=="cuda" and not torch.cuda.is_available(): raise RuntimeError("CUDA is unavailable in this Python environment")
    random.seed(c["seed"]+rank); np.random.seed(c["seed"]+rank); torch.manual_seed(c["seed"]+rank)
    torch.backends.cudnn.benchmark=False
    training=ArmorDataset(data,"train",allow_placeholder); validation=ArmorDataset(data,"val",allow_placeholder)
    fingerprint=hashlib.sha256(json.dumps([(r["image_sha256"],r["objects"],r["split"]) for r in training.records+validation.records],sort_keys=True).encode()).hexdigest()
    model=build_model(base_model).to(device); ir_hash=model.ir_sha256
    optimizer=torch.optim.AdamW(model.parameters(),lr=c["learning_rate"],weight_decay=c["weight_decay"])
    scaler=torch.amp.GradScaler("cuda",enabled=bool(c["amp"] and device.type=="cuda"))
    start=0; best=float("inf")
    if resume:
        checkpoint=torch.load(resume,map_location="cpu",weights_only=True)
        if checkpoint["ir_sha256"]!=ir_hash or checkpoint["dataset_sha256"]!=fingerprint: raise ValueError("resume architecture or dataset mismatch")
        model.load_state_dict(checkpoint["model"]); optimizer.load_state_dict(checkpoint["optimizer"]); scaler.load_state_dict(checkpoint["scaler"])
        start=checkpoint["epoch"]+1; best=checkpoint["best_val_loss"]
        # torch.load map_location=cpu: move optimizer state with the model.
        for state in optimizer.state.values():
            for k,v in state.items():
                if torch.is_tensor(v): state[k]=v.to(device)
    elif out.exists() and any(out.iterdir()): raise FileExistsError("run directory is nonempty; use a new directory or --resume")
    if start>=c["epochs"]: raise ValueError("resume already reached configured epochs; increase epochs to continue")
    if rank==0:
        out.mkdir(parents=True,exist_ok=True)
        (out/"config.json").write_text(json.dumps({**c,"base_model_sha256":ir_hash,"dataset_sha256":fingerprint,"world_size":world,"allow_placeholder":allow_placeholder,"smoke_max_steps":max_steps},indent=2)+"\n")
    if distributed: dist.barrier(); model=DistributedDataParallel(model,device_ids=[local])
    plain=model.module if distributed else model
    sampler=DistributedSampler(training,num_replicas=world,rank=rank,shuffle=True,seed=c["seed"]) if distributed else None
    val_loader=DataLoader(validation,batch_size=c["batch_size"],shuffle=False,num_workers=c["workers"],collate_fn=collate)
    for epoch in range(start,c["epochs"]):
        torch.manual_seed(c["seed"]+epoch+rank)
        if sampler: sampler.set_epoch(epoch)
        loader=DataLoader(training,batch_size=c["batch_size"],shuffle=sampler is None,sampler=sampler,num_workers=c["workers"],collate_fn=collate,
                          generator=torch.Generator().manual_seed(c["seed"]+epoch))
        lr=c["learning_rate"]*(.1+.9*(1+math.cos(math.pi*epoch/c["epochs"]))/2)
        for group in optimizer.param_groups: group["lr"]=lr
        model.train(); totals=torch.zeros(6,device=device)
        for step,(images,targets) in enumerate(loader):
            if max_steps is not None and step>=max_steps: break
            images=images.to(device); targets=[t.to(device) for t in targets]; optimizer.zero_grad(set_to_none=True)
            with torch.autocast(device_type=device.type,enabled=scaler.is_enabled()):
                prediction=model(images); loss,parts=armor_loss(prediction,targets)
            if not torch.isfinite(loss): raise FloatingPointError("nonfinite training loss")
            scaler.scale(loss).backward(); scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(),c["gradient_clip"],error_if_nonfinite=True)
            scaler.step(optimizer); scaler.update()
            totals+=torch.tensor([parts[k].item() for k in ("total","point","object","color","number")]+[1],device=device)
        if distributed: dist.all_reduce(totals)
        # Each rank validates the same complete holdout using the plain graph: no DDP collectives.
        metrics=evaluate(plain,val_loader,device,max_steps)
        if rank==0:
            improved=metrics["loss"]<best; best=min(best,metrics["loss"])
            report={"epoch":epoch,"lr":lr,"train":{k:float(v/max(totals[-1].item(),1)) for k,v in zip(("total","point","object","color","number"),totals[:5].tolist())},"validation":metrics}
            with (out/"metrics.jsonl").open("a") as f: f.write(json.dumps(report)+"\n")
            payload={"schema":"sp-armor-checkpoint-v1","epoch":epoch,"model":plain.state_dict(),"optimizer":optimizer.state_dict(),"scaler":scaler.state_dict(),"config":c,
                     "ir_sha256":ir_hash,"dataset_sha256":fingerprint,"best_val_loss":best,"allow_placeholder":allow_placeholder,"smoke_max_steps":max_steps}
            save_checkpoint(out/"last.pt",payload)
            if improved: save_checkpoint(out/"best.pt",payload)
            print(json.dumps(report),flush=True)
        if distributed: dist.barrier()
    if distributed: dist.destroy_process_group()
