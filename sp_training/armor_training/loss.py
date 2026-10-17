"""Anchor assignment and fine-tuning losses for the recovered decoded 22-channel head."""
import torch
from torch.nn import functional as F

from .contract import ANCHORS, STRIDES


def assignments(targets, device):
    """Best anchor at target center plus its axial neighbors; resolve cell collisions by size fit."""
    selected={}; offset=[0,19200,24000]
    for batch,rows in enumerate(targets):
        for t in rows:
            points=t[2:].reshape(4,2); lo=points.min(0).values; hi=points.max(0).values
            wh=(hi-lo).clamp(min=1); center=(hi+lo)/2
            candidates=[]
            for level,anchors in enumerate(ANCHORS):
                for anchor,a in enumerate(anchors):
                    ratio=torch.maximum(wh/wh.new_tensor(a),wh.new_tensor(a)/wh).max().item()
                    candidates.append((ratio,level,anchor))
            ratio,level,anchor=min(candidates)
            stride=STRIDES[level]; size=640//stride; cx,cy=(center/stride).tolist(); x,y=int(cx),int(cy)
            cells={(x,y)}
            if cx-x<0.5: cells.add((x-1,y))
            else: cells.add((x+1,y))
            if cy-y<0.5: cells.add((x,y-1))
            else: cells.add((x,y+1))
            for x,y in cells:
                if 0<=x<size and 0<=y<size:
                    index=offset[level]+anchor*size*size+y*size+x; key=(batch,index)
                    if key not in selected or ratio<selected[key][0]: selected[key]=(ratio,t,wh)
    if not selected: return None
    keys=list(selected)
    bi=torch.tensor([x[0] for x in keys],device=device); ni=torch.tensor([x[1] for x in keys],device=device)
    ts=torch.stack([selected[k][1] for k in keys]).to(device)
    wh=torch.stack([selected[k][2] for k in keys]).to(device)
    return bi,ni,ts,wh


def armor_loss(output, targets):
    if output.ndim!=3 or output.shape[1:]!=(25200,22): raise ValueError("SP output must be B x 25200 x 22")
    target_obj=torch.zeros_like(output[:,:,8]); matched=assignments(targets,output.device)
    zero=output.sum()*0
    point_loss=color_loss=number_loss=zero
    if matched is not None:
        bi,ni,t,wh=matched; pred=output[bi,ni]; target_obj[bi,ni]=1
        # Normalize errors by each target's size, so distant objects retain weight.
        norm=wh.repeat(1,4).clamp(min=8)
        point_loss=F.smooth_l1_loss((pred[:,:8]-t[:,2:])/norm,torch.zeros_like(pred[:,:8]),beta=.1)
        color_loss=F.cross_entropy(pred[:,9:13],t[:,0].long())
        number_loss=F.cross_entropy(pred[:,13:22],t[:,1].long())
    # Separately average positive and negative BCE; empty frames remain useful.
    logits=output[:,:,8]; bce=F.binary_cross_entropy_with_logits(logits,target_obj,reduction="none")
    prob=logits.sigmoid(); focal=(target_obj*(1-prob)+(1-target_obj)*prob).pow(2)
    positive=target_obj.bool(); negative=~positive
    obj_loss=(bce*focal)[negative].mean() if negative.any() else zero
    if positive.any(): obj_loss=obj_loss+(bce*focal)[positive].mean()
    total=4*point_loss+obj_loss+color_loss+number_loss
    return total,{"total":total.detach(),"point":point_loss.detach(),"object":obj_loss.detach(),"color":color_loss.detach(),"number":number_loss.detach()}
