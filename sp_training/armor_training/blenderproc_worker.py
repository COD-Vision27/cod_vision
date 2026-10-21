"""Selected server renderer. This file must be launched by BlenderProc, not local Python."""
import blenderproc as bproc
import argparse
import importlib.util
import json
import math
import os
import random
import sys
from pathlib import Path


def main():
    import bpy
    import numpy as np
    from mathutils import Matrix, Vector
    from PIL import Image
    p=argparse.ArgumentParser();p.add_argument("--config",required=True);a=p.parse_args()
    config=Path(a.config).resolve();root=config.parent.parent;c=json.loads(config.read_text())
    if os.environ.get("SP_ARMOR_EXECUTION_ROLE")!=c["execution"]["required_role"]:
        raise RuntimeError("render job is restricted to the explicitly enabled local training process")
    if c["execution"]["allow_local_fallback"] is not False:raise ValueError("local render fallback must stay disabled")
    if not c["assets"]["scene_blend"]:raise ValueError("awaiting future 2027 scene/model assets")
    scene_file=(root/c["assets"]["scene_blend"]).resolve()
    registry=json.loads((root/c["assets"]["registry"]).read_text())
    spec=importlib.util.spec_from_file_location("sp_armor_ground_truth",root/"blender_addon/sp_armor_labels/ground_truth.py")
    gt=importlib.util.module_from_spec(spec);spec.loader.exec_module(gt)
    bproc.init()
    bproc.loader.load_blend(str(scene_file),obj_types=["mesh","empty","light","camera","curve","armature"])
    scene=bpy.context.scene
    cameras=[o for o in scene.objects if o.type=="CAMERA"]
    if len(cameras)!=1:raise ValueError("scene template must declare exactly one render camera")
    scene.camera=cameras[0]
    bindings=gt.bind_registry(scene,registry)
    lights=[o for o in scene.objects if o.type=="LIGHT"]
    if not lights:raise ValueError("scene template must contain randomized illumination lights")
    width,height=c["sampling"]["resolution"];scene.render.resolution_x=width;scene.render.resolution_y=height;scene.render.resolution_percentage=100
    scene.render.use_border=False;scene.render.use_compositing=False
    scene.render.engine="CYCLES"
    bproc.renderer.set_max_amount_of_samples(c["sampling"]["samples"])
    device=c["execution"]["render_device"]
    if device not in ("OPTIX","CUDA","HIP"):raise ValueError("local GPU render backend must be explicit")
    # Strict device selection: no silent CPU fallback when a worker lacks the selected GPU backend.
    prefs=bpy.context.preferences.addons['cycles'].preferences;prefs.compute_device_type=device;prefs.get_devices()
    devices=[d for d in prefs.devices if d.type==device]
    if not devices:raise RuntimeError(f"local machine has no available {device} render device")
    for d in prefs.devices:d.use=d.type==device
    scene.cycles.device='GPU'
    raw=(root/c["artifacts"]["raw"]).resolve();raw.mkdir(parents=True,exist_ok=True)
    settings=c["sampling"];seed=settings["seed"];rng=random.Random(seed);chunk=settings["chunk_size"]
    samples=[(s,v) for s in range(settings["scenes"]) for v in range(settings["views_per_scene"])]
    scene_prefix=f"render-{seed}"
    for start in range(0,len(samples),chunk):
        bproc.utility.reset_keyframes();records=[]
        for frame,(scene_index,view_index) in enumerate(samples[start:start+chunk]):
            scene.frame_set(frame)
            camera=scene.camera;cam=settings["camera"];target=Vector(cam["target"])
            for attempt in range(100):
                radius=rng.uniform(*cam["radius_m"]);az=math.radians(rng.uniform(*cam["azimuth_deg"]));el=math.radians(rng.uniform(*cam["elevation_deg"]))
                location=target+Vector((radius*math.sin(az)*math.cos(el),radius*math.sin(el),radius*math.cos(az)*math.cos(el)))
                rotation=(target-location).to_track_quat('-Z','Y').to_matrix()
                pose=Matrix.Translation(location)@rotation.to_4x4();camera.matrix_world=pose;camera.data.lens=rng.uniform(*cam["lens_mm"])
                for light in lights:light.data.energy=rng.uniform(*settings["lighting"]["energy"])
                scene.view_settings.exposure=rng.uniform(*settings["lighting"]["exposure"])
                bpy.context.view_layer.update();sample_id=f"{scene_prefix}-{scene_index}-{view_index}"
                if (raw/(sample_id+".png")).exists() or (raw/(sample_id+".json")).exists():raise FileExistsError(sample_id)
                try:record=gt.capture(scene,bindings,view_index,f"{scene_prefix}-{scene_index}",sample_id,seed,registry)
                except gt.AmbiguousFrame:continue
                # Preserve exactly the camera/illumination state used for this annotation in its render keyframe.
                bproc.camera.add_camera_pose(np.array(pose),frame=frame);camera.data.keyframe_insert(data_path="lens",frame=frame)
                for light in lights:light.data.keyframe_insert(data_path="energy",frame=frame)
                scene.view_settings.keyframe_insert(data_path="exposure",frame=frame)
                records.append(record);break
            else:raise RuntimeError("cannot bind an unambiguous camera frame; scene/landmark integration is unfinished")
        scene.frame_start=0;scene.frame_end=len(records)
        data=bproc.renderer.render()
        if len(data["colors"])!=len(records):raise RuntimeError("renderer frame count differs from captured annotations")
        # Blender writes the returned frame pixels without additional resampling or cropping.
        for image,record in zip(data["colors"],records):
            if image.dtype!=np.uint8:raise ValueError("expected BlenderProc uint8 RGB, refusing an implicit color/range transform")
            Image.fromarray(image[:,:,:3]).save(raw/record["image"])
            gt.commit(record,raw)


if __name__=="__main__":main()
