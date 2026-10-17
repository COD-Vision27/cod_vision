"""Renderer-owned ground truth, independent of every detection model. Requires bpy only in Blender."""
import hashlib
import json
from pathlib import Path

ORDER=("top_left","bottom_left","bottom_right","top_right")


class AmbiguousFrame(ValueError):
    """Reject a full rendered frame rather than hide an unlabelled foreground armor."""


def bind_registry(scene, registry):
    if registry.get("schema")!="sp-armor-registry-v1" or registry.get("status")!="verified" or not registry.get("armors") or not registry.get("revision"):
        raise ValueError("annotation awaits a verified 2027 asset registry")
    bindings=[]; used=set()
    for item in registry["armors"]:
        if item["status"]!="verified" or not item.get("asset_id"): raise ValueError("unverified armor binding")
        if type(item["color_id"]) is not int or item["color_id"] not in range(4) or type(item["number_id"]) is not int or item["number_id"] not in range(9):
            raise ValueError("invalid project head color/number binding")
        root=scene.objects.get(item["object"])
        if root is None or root.name in used: raise ValueError("missing or duplicate armor root")
        used.add(root.name)
        points=[scene.objects.get(item["keypoints"][k]) for k in ORDER]
        if any(p is None for p in points) or len({p.name for p in points})!=4: raise ValueError("four distinct landmark objects are required")
        descendants={root,*root.children_recursive}
        if not all(p in descendants for p in points): raise ValueError("landmarks must belong to their armor root")
        meshes={ob.name for ob in descendants if ob.type=="MESH"}
        if not meshes: raise ValueError("armor binding has no render mesh")
        bindings.append((item,root,points,meshes))
    return bindings


def capture(scene, bindings, frame_id, scene_id, sample_id, seed, registry):
    import bpy
    from bpy_extras.object_utils import world_to_camera_view
    camera=scene.camera
    if camera is None: raise ValueError("annotation requires the render camera")
    deps=bpy.context.evaluated_depsgraph_get(); origin=camera.matrix_world.translation; objects=[]
    for item,root,points,meshes in bindings:
        world=[p.evaluated_get(deps).matrix_world.translation.copy() for p in points]
        projected=[world_to_camera_view(scene,camera,p) for p in world]
        inside=[p.z>0 and 0<=p.x<=1 and 0<=p.y<=1 for p in projected]
        if not any(inside):
            # The four points may surround the image despite none being inside. Only skip an entirely disjoint panel.
            if all(p.z<=0 for p in projected): continue
            if all(p.z>0 for p in projected) and (max(p.x for p in projected)<0 or min(p.x for p in projected)>1 or max(p.y for p in projected)<0 or min(p.y for p in projected)>1): continue
            raise AmbiguousFrame("armor surrounds/straddles the image or camera plane")
        if not all(inside): raise AmbiguousFrame("partial armor truncation")
        visible=[]
        for point in world:
            ray=point-origin
            hit,location,normal,face,ob,matrix=scene.ray_cast(deps,origin,ray.normalized(),distance=ray.length+.002)
            visible.append(bool(hit and ob.original.name in meshes and abs((location-origin).length-ray.length)<.003))
        if not all(visible): raise AmbiguousFrame("armor landmarks are occluded, back-facing or misbound")
        xy=[[float(p.x),float(1-p.y)] for p in projected]
        cross=[]
        for i in range(4):
            a,b,c=xy[i],xy[(i+1)%4],xy[(i+2)%4]
            cross.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
        if not all(x< -1e-9 for x in cross): raise AmbiguousFrame("landmark winding/front-face mismatch")
        objects.append({"color_id":item["color_id"],"number_id":item["number_id"],"points":xy,"visible":[True]*4,"asset_id":item["asset_id"]})
    return {"schema":"sp-armor-v5-v1","sample_id":sample_id,"scene_id":scene_id,"split_seed":seed,"frame_id":frame_id,"image_frame_id":frame_id,"annotation_frame_id":frame_id,
            "image":sample_id+".png","objects":objects,
            "provenance":{"backend":"BlenderProc/SP Armor Labels","asset_id":registry["revision"],"asset_status":"verified",
                          "registry_sha256":hashlib.sha256(json.dumps(registry,sort_keys=True).encode()).hexdigest(),"annotation_source":"scene_geometry","keypoint_order":ORDER}}


def commit(record, raw_dir):
    """Called only after the same rendered frame's PNG is on disk."""
    root=Path(raw_dir); image=root/record["image"]; target=root/(record["sample_id"]+".json")
    if not image.is_file(): raise ValueError("RGB must exist before committing its annotation")
    if target.exists(): raise FileExistsError(target)
    label=root/(record["sample_id"]+".txt")
    if label.exists(): raise FileExistsError(label)
    lines=[]
    for obj in record["objects"]:
        xy=obj["points"];lo=[min(p[i] for p in xy) for i in (0,1)];hi=[max(p[i] for p in xy) for i in (0,1)]
        values=[(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,hi[0]-lo[0],hi[1]-lo[1],*[x for p in xy for x in p]]
        lines.append(str(obj["color_id"]*9+obj["number_id"])+" "+" ".join(f"{v:.8f}" for v in values))
    label.write_text("\n".join(lines)+("\n" if lines else ""))
    record["project_label"]=label.name
    temporary=target.with_suffix(".json.tmp"); temporary.write_text(json.dumps(record,ensure_ascii=False,indent=2)+"\n"); temporary.replace(target)
