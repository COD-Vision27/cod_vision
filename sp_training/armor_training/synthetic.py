"""Fast CPU pinhole simulation for pipeline checks / flat reference textures."""
import json
import hashlib
import math
from pathlib import Path
import cv2
import numpy as np
from .contract import write_sample, validate_object


def load_asset(path):
    path = Path(path).resolve()
    a = json.loads(path.read_text())
    if a["status"] not in ("placeholder", "verified") or not a["asset_id"]:
        raise ValueError("asset needs an id and status placeholder/verified")
    w, h = a["width_m"], a["height_m"]
    k = np.asarray(a["keypoints_m"], float)
    if w <= 0 or h <= 0 or k.shape != (4, 3) or not np.isfinite(k).all():
        raise ValueError("positive dimensions and four finite metric keypoints required")
    if not a["allowed_colors"] or not a["allowed_numbers"] or any(type(i) is not int or i not in range(4) for i in a["allowed_colors"]) or any(type(i) is not int or i not in range(9) for i in a["allowed_numbers"]):
        raise ValueError("invalid allowed color/number ids")
    for name in ("texture", "mesh"):
        if a.get(name):
            a[name] = str((path.parent / a[name]).resolve())
            if not Path(a[name]).is_file():
                raise FileNotFoundError(a[name])
    if a["status"] == "verified" and not (a.get("texture") or a.get("mesh")):
        raise ValueError("verified assets require a reference texture or mesh")
    if a["status"] == "verified" and (len(a["allowed_colors"]) != 1 or len(a["allowed_numbers"]) != 1):
        raise ValueError("verified assets must declare one exact color/number variant per config")
    if a.get("texture") and (len(a["allowed_colors"]) != 1 or len(a["allowed_numbers"]) != 1):
        raise ValueError("a fixed texture requires exactly one allowed color and number; use separate variant asset configs")
    digest = hashlib.sha256(path.read_bytes())
    for name in ("mesh", "texture"):
        if a.get(name): digest.update(Path(a[name]).read_bytes())
    a["asset_sha256"] = digest.hexdigest()
    a["config_path"] = str(path)
    return a


def rotation(yaw, pitch, roll):
    x,y,z = np.deg2rad([pitch,yaw,roll]); cx,sx=np.cos(x),np.sin(x); cy,sy=np.cos(y),np.sin(y); cz,sz=np.cos(z),np.sin(z)
    return np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]]) @ np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]]) @ np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])


def texture(asset, color, number):
    if asset.get("texture"):
        img=cv2.imread(asset["texture"],cv2.IMREAD_UNCHANGED)
        if img is None or img.ndim != 3 or img.shape[2] not in (3,4):
            raise ValueError("texture must be RGB/RGBA")
        return img if img.shape[2] == 4 else cv2.cvtColor(img,cv2.COLOR_BGR2BGRA)
    img=np.full((250,280,4),(22,22,22,255),np.uint8)
    c=[(255,95,35,255),(40,45,255,255),(95,95,95,255),(225,50,210,255)][color]
    # Deliberately generic placeholder; no unverified 2027 dot pattern is invented.
    cv2.rectangle(img,(4,4),(275,245),(60,60,60,255),3)
    cv2.rectangle(img,(7,65),(20,185),c,-1); cv2.rectangle(img,(259,65),(272,185),c,-1)
    txt=["G","1","2","3","4","5","O","B","B"][number]
    cv2.putText(img,txt,(100,160),cv2.FONT_HERSHEY_SIMPLEX,2.5,(220,220,220,255),5,cv2.LINE_AA)
    return img


def generate(root, asset_path, count=64, seed=2027, shard=0, width=640, height=480, backgrounds=None):
    asset=load_asset(asset_path)
    if asset.get("mesh"):
        raise ValueError("mesh assets require the Blender backend; CPU backend is planar")
    if count < 1 or shard < 0 or min(width,height) < 64:
        raise ValueError("positive count, nonnegative shard, dimensions >=64 required")
    backgrounds=sorted(Path(backgrounds).glob("*")) if backgrounds else []
    for index in range(count):
        rng=np.random.default_rng(np.random.SeedSequence([seed,shard,index]))
        scene_id=f"cpu-{seed}-{shard}-{index}"
        image=rng.integers(8,60,(height,width,3),dtype=np.uint8)
        if backgrounds:
            bg=cv2.imread(str(backgrounds[int(rng.integers(len(backgrounds)))]))
            if bg is None: raise ValueError("unreadable background")
            image=cv2.resize(bg,(width,height))
        for _ in range(12):
            x,y=int(rng.integers(width)),int(rng.integers(height))
            cv2.rectangle(image,(x,y),(min(width-1,x+int(rng.integers(5,100))),min(height-1,y+int(rng.integers(5,70)))),tuple(int(x) for x in rng.integers(15,120,3)),-1)
        objects=[]; placements=[]
        # Whole frame is rejected on overlap, truncation or invalid landmarks, so visible targets never become unlabelled negatives.
        desired=0 if rng.random()<0.15 else int(rng.integers(1,4))
        attempts=0
        while len(objects)<desired and attempts<150:
            attempts+=1
            color=int(rng.choice(asset["allowed_colors"])); number=int(rng.choice(asset["allowed_numbers"]))
            r=rotation(rng.uniform(-60,60),rng.uniform(-25,25),rng.uniform(-16,16))
            f=width*1.05; z=rng.uniform(0.65,4.0)
            t=np.array([rng.uniform(-0.38,0.38)*z,rng.uniform(-0.28,0.28)*z,z])
            def project(p):
                v=np.asarray(p)@r.T+t
                return np.column_stack([f*v[:,0]/v[:,2]+width/2,height/2-f*v[:,1]/v[:,2]])
            w,h=asset["width_m"]/2,asset["height_m"]/2
            outer=project([[-w,h,0],[-w,-h,0],[w,-h,0],[w,h,0]]).astype(np.float32)
            lo,hi=outer.min(0),outer.max(0)
            if (lo<3).any() or hi[0]>width-3 or hi[1]>height-3 or min(hi-lo)<10: continue
            if any(np.all(hi+3>l) and np.all(lo-3<u) for l,u in placements): continue
            p=project(asset["keypoints_m"])/[width,height]
            obj={"color_id":color,"number_id":number,"points":p.tolist(),"visible":[True]*4}
            try: validate_object(obj)
            except ValueError: continue
            tex=texture(asset,color,number); th,tw=tex.shape[:2]
            matrix=cv2.getPerspectiveTransform(np.float32([[0,0],[0,th-1],[tw-1,th-1],[tw-1,0]]),outer)
            warped=cv2.warpPerspective(tex,matrix,(width,height))
            alpha=warped[:,:,3:4].astype(float)/255
            image=(warped[:,:,:3]*alpha+image*(1-alpha)).astype(np.uint8)
            objects.append(obj); placements.append((lo,hi))
        gain=float(rng.uniform(0.55,1.5)); image=np.clip(image.astype(float)*gain+rng.normal(0,rng.uniform(0,5),image.shape),0,255).astype(np.uint8)
        if rng.random()<0.35: image=cv2.GaussianBlur(image,(3,3),float(rng.uniform(0.4,1.3)))
        write_sample(root,scene_id,scene_id,image,objects,{"backend":"cpu-planar", "asset_id":asset["asset_id"],"asset_status":asset["status"],"asset_sha256":asset["asset_sha256"],"seed":seed,"shard":shard,"index":index,"calibration":"pinhole fx=fy=1.05*width; no distortion"},seed)
