"""Early experimental standalone renderer; production path is blenderproc_worker.py.

blender -b -t 4 --python armor_training/blender_worker.py -- --asset configs/placeholder.json --output data/blender-raw --count 1000
Then: python -m armor_training import --source data/blender-raw --output data/armor
"""
import argparse
import hashlib
import json
import math
import random
import sys
from pathlib import Path


def main():
    import bpy
    from mathutils import Vector
    from bpy_extras.object_utils import world_to_camera_view
    p=argparse.ArgumentParser(); p.add_argument("--asset",required=True); p.add_argument("--output",required=True)
    p.add_argument("--count",type=int,default=64); p.add_argument("--seed",type=int,default=2027); p.add_argument("--shard",type=int,default=0)
    p.add_argument("--width",type=int,default=640); p.add_argument("--height",type=int,default=480)
    p.add_argument("--engine",choices=["CYCLES","BLENDER_EEVEE_NEXT"],default="CYCLES"); p.add_argument("--samples",type=int,default=16)
    p.add_argument("--backgrounds",help="optional directory of photos used as world background")
    a=p.parse_args(sys.argv[sys.argv.index("--")+1:])
    if a.count<1 or a.shard<0 or min(a.width,a.height)<64: raise ValueError("invalid count, shard or resolution")
    asset_path=Path(a.asset).resolve(); asset=json.loads(asset_path.read_text()); out=Path(a.output).resolve(); out.mkdir(parents=True,exist_ok=True)
    if asset["status"] not in ("placeholder","verified") or len(asset["keypoints_m"])!=4: raise ValueError("invalid asset contract")
    if asset["status"] == "verified" and not (asset.get("texture") or asset.get("mesh")):
        raise ValueError("verified assets require a reference texture or mesh")
    if asset["status"] == "verified" and (len(asset["allowed_colors"])!=1 or len(asset["allowed_numbers"])!=1):
        raise ValueError("verified assets require one exact color/number variant per config")
    if asset.get("texture") and (len(asset["allowed_colors"])!=1 or len(asset["allowed_numbers"])!=1):
        raise ValueError("a fixed texture must declare exactly one color/number; use one asset configuration per display variant")
    backgrounds=sorted(x for x in Path(a.backgrounds).glob("*") if x.suffix.lower() in (".jpg",".jpeg",".png")) if a.backgrounds else []
    bpy.ops.object.select_all(action="SELECT"); bpy.ops.object.delete(use_global=False)
    scene=bpy.context.scene; scene.render.engine=a.engine; scene.render.resolution_x=a.width; scene.render.resolution_y=a.height; scene.render.resolution_percentage=100
    scene.render.image_settings.file_format="PNG"; scene.render.image_settings.color_mode="RGB"; scene.render.film_transparent=False
    scene.cycles.samples=a.samples; scene.cycles.use_denoising=True
    scene.view_settings.view_transform="Standard"; scene.world.use_nodes=True
    world=scene.world.node_tree; bg=world.nodes.get("Background"); bg.inputs[0].default_value=(.06,.06,.06,1)
    bpy.ops.object.camera_add(); camera=bpy.context.object; camera.data.lens=40; camera.data.sensor_width=36; scene.camera=camera
    bpy.ops.object.light_add(type="AREA",location=(1,1,3)); light=bpy.context.object; light.data.energy=150; light.data.shape="DISK"; light.data.size=3
    root=bpy.data.objects.new("armor_asset_root",None); scene.collection.objects.link(root)
    before=set(bpy.data.objects)
    if asset.get("mesh"):
        mesh=(asset_path.parent/asset["mesh"]).resolve()
        if not mesh.is_file(): raise FileNotFoundError(mesh)
        if mesh.suffix.lower() in (".glb",".gltf"): bpy.ops.import_scene.gltf(filepath=str(mesh))
        elif mesh.suffix.lower()==".obj": bpy.ops.wm.obj_import(filepath=str(mesh))
        elif mesh.suffix.lower()==".stl": bpy.ops.wm.stl_import(filepath=str(mesh))
        else: raise ValueError("mesh must be GLB, glTF, OBJ or STL; export CAD STEP to a mesh first")
        imported=set(bpy.data.objects)-before
        for ob in imported:
            if ob.parent not in imported: ob.parent=root
        mesh_objects=[ob for ob in imported if ob.type=="MESH"]
        if not mesh_objects: raise ValueError("asset has no mesh")
        # Scale and orientation apply to imported geometry; keypoints remain in the authored metric asset frame.
        geometry=bpy.data.objects.new("import_transform",None); scene.collection.objects.link(geometry); geometry.parent=root
        for ob in imported:
            if ob.parent==root: ob.parent=geometry
        geometry.scale=(asset.get("mesh_scale",1),)*3
        geometry.rotation_euler=[math.radians(v) for v in asset.get("mesh_rotation_deg",[0,0,0])]
    else:
        bpy.ops.mesh.primitive_plane_add(size=2); plane=bpy.context.object; plane.name="placeholder_body"; plane.parent=root
        plane.scale=(asset["width_m"]/2,asset["height_m"]/2,1); mesh_objects=[plane]
    names={ob.name for ob in mesh_objects}
    material=None
    if asset.get("display_material"):
        material=bpy.data.materials.get(asset["display_material"])
        if material is None: raise ValueError("display_material is absent from imported mesh")
    elif not asset.get("mesh"):
        material=bpy.data.materials.new("placeholder_display"); mesh_objects[0].data.materials.append(material)
    if asset.get("texture"):
        tex=(asset_path.parent/asset["texture"]).resolve()
        if material is None: raise ValueError("texture on a mesh requires display_material")
        material.use_nodes=True; nodes=material.node_tree.nodes; bsdf=nodes.get("Principled BSDF")
        texnode=nodes.new("ShaderNodeTexImage"); texnode.image=bpy.data.images.load(str(tex))
        material.node_tree.links.new(texnode.outputs["Color"],bsdf.inputs["Base Color"])
        if "Emission Color" in bsdf.inputs: material.node_tree.links.new(texnode.outputs["Color"],bsdf.inputs["Emission Color"])
    asset_digest=hashlib.sha256(asset_path.read_bytes())
    for key in ("mesh","texture"):
        if asset.get(key): asset_digest.update((asset_path.parent/asset[key]).resolve().read_bytes())
    for index in range(a.count):
        rng=random.Random(f"{a.seed}:{a.shard}:{index}"); sample_id=f"blender-{a.seed}-{a.shard}-{index}"
        path=out/f"{sample_id}.json"; image_path=out/f"{sample_id}.png"
        if path.exists() or image_path.exists(): raise FileExistsError(f"refusing to overwrite {sample_id}")
        scene.frame_set(index+1); scene.cycles.seed=(a.seed+a.shard*100000+index)%(2**31)
        negative=rng.random()<.15
        for ob in mesh_objects: ob.hide_render=negative; ob.hide_viewport=negative
        color=rng.choice(asset["allowed_colors"]); number=rng.choice(asset["allowed_numbers"])
        if material:
            material.use_nodes=True; bsdf=material.node_tree.nodes.get("Principled BSDF")
            if bsdf and not asset.get("texture"):
                c=[(.03,.18,1,1),(1,.03,.03,1),(.15,.15,.15,1),(.8,.03,.8,1)][color]
                bsdf.inputs["Base Color"].default_value=c
                if "Emission Color" in bsdf.inputs: bsdf.inputs["Emission Color"].default_value=c
            if bsdf and "Emission Strength" in bsdf.inputs: bsdf.inputs["Emission Strength"].default_value=rng.uniform(.2,3)
        if not asset.get("mesh") and not asset.get("texture"):
            # Generic text belongs only to the placeholder; never assert it is the official 2027 pattern.
            old=bpy.data.objects.get("placeholder_digit")
            if old:
                old_curve=old.data
                bpy.data.objects.remove(old,do_unlink=True)
                if old_curve.users==0: bpy.data.curves.remove(old_curve)
            curve=bpy.data.curves.new("placeholder_digit_curve","FONT"); curve.body=["G","1","2","3","4","5","O","B","B"][number]; curve.align_x="CENTER"; curve.align_y="CENTER"; curve.size=asset["height_m"]*.5
            digit=bpy.data.objects.new("placeholder_digit",curve); scene.collection.objects.link(digit); digit.parent=root; digit.location.z=.0003
            digit.hide_render=negative; digit.hide_viewport=negative; names.add(digit.name)
        light.data.energy=rng.uniform(50,350); light.location=(rng.uniform(-2,2),rng.uniform(-2,2),3)
        scene.view_settings.exposure=rng.uniform(-1,1); bg.inputs[1].default_value=rng.uniform(.2,.9)
        if backgrounds:
            env=world.nodes.get("random_background") or world.nodes.new("ShaderNodeTexEnvironment"); env.name="random_background"
            if env.image: bpy.data.images.remove(env.image)
            env.image=bpy.data.images.load(str(rng.choice(backgrounds))); world.links.new(env.outputs["Color"],bg.inputs[0])
        objects=[]
        for attempt in range(100):
            distance=rng.uniform(.7,4.8); camera.location=(0,0,distance); camera.rotation_euler=(0,0,0)
            root.location=(rng.uniform(-.25,.25)*distance,rng.uniform(-.18,.18)*distance,0)
            root.rotation_euler=[math.radians(rng.uniform(-25,25)),math.radians(rng.uniform(-60,60)),math.radians(rng.uniform(-15,15))]
            bpy.context.view_layer.update()
            if negative: break
            matrix=root.matrix_world
            corners=[matrix@Vector(p) for p in asset["keypoints_m"]]
            projected=[world_to_camera_view(scene,camera,p) for p in corners]
            if any(v.z<=0 or not .01<v.x<.99 or not .01<v.y<.99 for v in projected): continue
            # Reject frames whose *outer panel* is truncated, even when landmarks fit.
            w,h=asset["width_m"]/2,asset["height_m"]/2
            outer=[world_to_camera_view(scene,camera,matrix@Vector(v)) for v in ((-w,-h,0),(-w,h,0),(w,h,0),(w,-h,0))]
            if any(v.z<=0 or not 0<v.x<1 or not 0<v.y<1 for v in outer): continue
            visible=True; deps=bpy.context.evaluated_depsgraph_get()
            for point in corners:
                ray=point-camera.location; distance_to_point=ray.length
                hit,location,normal,face,ob,mat=scene.ray_cast(deps,camera.location,ray.normalized(),distance=distance_to_point+.002)
                if not hit or ob.original.name not in names or abs((location-camera.location).length-distance_to_point)>.003:
                    visible=False; break
            if not visible: continue
            objects=[{"color_id":color,"number_id":number,"points":[[v.x,1-v.y] for v in projected],"visible":[True]*4}]; break
        else: raise RuntimeError("could not obtain an unambiguous frame; check asset axes, physical keypoints and mesh surface")
        scene.render.filepath=str(image_path); bpy.ops.render.render(write_still=True)
        path.write_text(json.dumps({"schema":"sp-armor-v5-v1","sample_id":sample_id,"scene_id":sample_id,"frame_id":index,
                                   "image_frame_id":index,"annotation_frame_id":index,"image":image_path.name,"objects":objects,"split_seed":a.seed,
                                   "provenance":{"backend":"blender","asset_id":asset["asset_id"],"asset_status":asset["status"],"asset_sha256":asset_digest.hexdigest(),
                                                 "seed":a.seed,"shard":a.shard,"engine":a.engine,"samples":a.samples,"blender_version":bpy.app.version_string}},indent=2)+"\n")
        print(f"SP_DATASET_FRAME {index+1}/{a.count} {sample_id}",flush=True)


if __name__=="__main__": main()
