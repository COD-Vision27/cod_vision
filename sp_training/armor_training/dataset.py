import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from .contract import SCHEMA, split_for, parse_label, label_line, POINT_ORDER


def contained(root, path):
    p=(root/path).resolve()
    if not p.is_relative_to(root.resolve()): raise ValueError("dataset path escapes its root")
    return p


def validate_dataset(root, allow_placeholder=False):
    root=Path(root).resolve(); counts={k:0 for k in ("train","val","test")}; negatives=0
    scenes={}; sample_ids=set(); records=[]; image_paths=set(); label_paths=set()
    for path in sorted((root/"meta").glob("*/*.json")):
        r=json.loads(path.read_text()); split=r["split"]
        if r["schema"]!=SCHEMA or split not in counts or path.parent.name!=split: raise ValueError(f"schema/split mismatch: {path}")
        if r["sample_id"] in sample_ids: raise ValueError("duplicate sample id")
        sample_ids.add(r["sample_id"])
        if split_for(r["scene_id"],r["split_seed"])!=split: raise ValueError("invalid scene split")
        if scenes.setdefault(r["scene_id"],split)!=split: raise ValueError("scene leakage between splits")
        if r["provenance"]["asset_status"]!="verified" and not allow_placeholder: raise ValueError("placeholder asset data: use --allow-placeholder only for pipeline smoke tests")
        p=contained(root,r["image"]); lp=contained(root,r["label"])
        if p in image_paths or lp in label_paths: raise ValueError("duplicate image or label path")
        image_paths.add(p); label_paths.add(lp)
        image=cv2.imread(str(p))
        if image is None or image.shape[:2]!=(r["height"],r["width"]): raise ValueError(f"bad image: {p}")
        if hashlib.sha256(p.read_bytes()).hexdigest()!=r["image_sha256"]: raise ValueError("image checksum mismatch")
        lines=lp.read_text().splitlines()
        if len(lines)!=len(r["objects"]): raise ValueError("object/label count mismatch")
        for obj,line in zip(r["objects"],lines):
            parse_label(line)
            if line!=label_line(obj): raise ValueError("label differs from canonical metadata")
        counts[split]+=1; negatives+=not lines; records.append(r)
    if not records: raise ValueError("empty dataset")
    if set((root/"images").glob("*/*.png"))!=image_paths or set((root/"labels").glob("*/*.txt"))!=label_paths:
        raise ValueError("orphan image/label without committed metadata")
    return records,{"samples":len(records),"splits":counts,"negative_frames":negatives,"scenes":len(scenes)}


def prepare(root, allow_placeholder=False):
    records,report=validate_dataset(root,allow_placeholder)
    root=Path(root)
    for split in ("train","val","test"):
        # Relative paths are resolved by this package, so moving the full directory is safe.
        (root/f"{split}.txt").write_text("".join(r["image"]+"\n" for r in records if r["split"]==split))
    (root/"manifest.jsonl").write_text("".join(json.dumps(r,ensure_ascii=False)+"\n" for r in records))
    (root/"dataset.json").write_text(json.dumps({"schema":SCHEMA,"point_order":POINT_ORDER,"class_encoding":"color_id*9+number_id","label_columns":13,"summary":report},indent=2)+"\n")
    return report


def preview(root, output, count=12):
    root=Path(root); paths=sorted((root/"meta").glob("*/*.json"))[:count]
    if not paths: raise ValueError("empty dataset")
    thumbs=[]
    for path in paths:
        r=json.loads(path.read_text()); image=cv2.imread(str(contained(root,r["image"])))
        if image is None: raise ValueError("bad preview image")
        h,w=image.shape[:2]
        for o in r["objects"]:
            p=np.round(np.asarray(o["points"])*[w,h]).astype(int)
            cv2.polylines(image,[p],True,(0,255,0),1)
            for i,v in enumerate(p): cv2.circle(image,tuple(v),3,(0,255,255),-1); cv2.putText(image,str(i),tuple(v+3),0,.4,(255,255,255),1)
        image=cv2.resize(image,(320,240)); cv2.putText(image,r["split"]+" "+r["sample_id"],(4,15),0,.35,(0,255,255),1)
        thumbs.append(image)
    while len(thumbs)%3: thumbs.append(np.zeros_like(thumbs[0]))
    sheet=np.vstack([np.hstack(thumbs[i:i+3]) for i in range(0,len(thumbs),3)])
    Path(output).parent.mkdir(parents=True,exist_ok=True)
    if not cv2.imwrite(str(output),sheet): raise OSError("preview write failed")


def import_frames(source, root):
    """Import renderer-neutral same-frame records, never predicted YOLO detections.

    source/*.json: schema, frame_id, image_frame_id, annotation_frame_id, scene_id,
    image (relative), objects, provenance (including asset_id/status), split_seed.
    The explicit IDs prevent asynchronous RGB/ground-truth mispairing.
    """
    from .contract import write_sample
    source=Path(source).resolve(); total=0
    for path in sorted(source.glob("*.json")):
        r=json.loads(path.read_text())
        if r["schema"]!=SCHEMA or not (r["frame_id"]==r["image_frame_id"]==r["annotation_frame_id"]):
            raise ValueError(f"RGB/annotation frame mismatch: {path}")
        img=cv2.imread(str(contained(source,r["image"])))
        if img is None: raise ValueError(f"bad source image: {path}")
        write_sample(root,r["sample_id"],r["scene_id"],img,r["objects"],r["provenance"],r.get("split_seed",2027)); total+=1
    if total==0: raise ValueError("no source records")
    return {"imported":total}


class ArmorDataset:
    def __init__(self, root, split, allow_placeholder=False):
        self.root=Path(root).resolve()
        records,_=validate_dataset(self.root,allow_placeholder)
        self.records=[r for r in records if r["split"]==split]
        if not self.records: raise ValueError(f"no samples in {split}")
    def __len__(self): return len(self.records)
    def __getitem__(self,index):
        import torch
        r=self.records[index]; image=cv2.imread(str(self.root/r["image"]))
        h,w=image.shape[:2]; scale=min(640/h,640/w); nh,nw=int(h*scale),int(w*scale)
        canvas=np.zeros((640,640,3),np.uint8); canvas[:nh,:nw]=cv2.resize(image,(nw,nh))
        # Match C++ exactly: RGB /255, top-left resize + black right/bottom padding.
        image=torch.from_numpy(canvas[:,:,::-1].copy().transpose(2,0,1)).float()/255
        targets=[]
        for o in r["objects"]:
            p=np.asarray(o["points"])*[w*scale,h*scale]
            targets.append([o["color_id"],o["number_id"],*p.flatten()])
        return image,torch.tensor(targets,dtype=torch.float32).reshape(-1,10)


def collate(batch):
    import torch
    return torch.stack([b[0] for b in batch]),[b[1] for b in batch]
