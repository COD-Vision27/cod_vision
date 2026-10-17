"""Building-stage Blender add-on: one action renders and exports same-frame armor truth."""
bl_info={"name":"SP Armor Labels","author":"SP armor pipeline","version":(0,1,0),"blender":(4,2,0),"location":"View3D > Sidebar > SP Armor","category":"Import-Export","description":"Render and label registered armor landmarks without any detector"}

import json
from pathlib import Path
import bpy
from .ground_truth import bind_registry,capture,commit


class SPARMOR_OT_export(bpy.types.Operator):
    bl_idname="sp_armor.export_frame"
    bl_label="渲染并自动标注当前帧"
    bl_description="从场景中绑定的真实四点导出RGB和项目标签中间记录，不调用识别模型"

    def execute(self,context):
        scene=context.scene
        try:
            if not scene.sp_armor_registry or not scene.sp_armor_raw_dir: raise ValueError("registry and raw output bindings are required")
            registry=json.loads(Path(bpy.path.abspath(scene.sp_armor_registry)).read_text())
            bindings=bind_registry(scene,registry)
            if not scene.sp_armor_sequence: raise ValueError("a scene/sequence identity is required")
            sample_id=f"{scene.sp_armor_sequence}-{scene.frame_current:08d}"
            if any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in sample_id): raise ValueError("sequence id must be an ASCII filename stem")
            record=capture(scene,bindings,scene.frame_current,scene.sp_armor_sequence,sample_id,2027,registry)
            out=Path(bpy.path.abspath(scene.sp_armor_raw_dir));out.mkdir(parents=True,exist_ok=True)
            if (out/record["image"]).exists() or (out/(sample_id+".json")).exists(): raise FileExistsError(sample_id)
            old=(scene.render.filepath,scene.render.image_settings.file_format,scene.render.image_settings.color_mode,scene.render.use_file_extension)
            try:
                scene.render.filepath=str(out/record["image"]); scene.render.image_settings.file_format="PNG";scene.render.image_settings.color_mode="RGB";scene.render.use_file_extension=True
                bpy.ops.render.render(write_still=True);commit(record,out)
            finally:
                scene.render.filepath,scene.render.image_settings.file_format,scene.render.image_settings.color_mode,scene.render.use_file_extension=old
            self.report({'INFO'},"当前帧RGB与几何真值已导出")
            return {'FINISHED'}
        except Exception as error:
            self.report({'ERROR'},str(error));return {'CANCELLED'}


class SPARMOR_PT_panel(bpy.types.Panel):
    bl_label="SP Armor Labels · building"
    bl_idname="SPARMOR_PT_annotation"
    bl_space_type='VIEW_3D';bl_region_type='UI';bl_category='SP Armor'
    def draw(self,context):
        layout=self.layout;scene=context.scene
        layout.label(text="标注来自场景几何，不依赖旧模型")
        layout.prop(scene,"sp_armor_registry",text="资产绑定")
        layout.prop(scene,"sp_armor_sequence",text="场景/轨迹")
        layout.prop(scene,"sp_armor_raw_dir",text="标注输出")
        layout.operator("sp_armor.export_frame")


CLASSES=(SPARMOR_OT_export,SPARMOR_PT_panel)

def register():
    for cls in CLASSES:bpy.utils.register_class(cls)
    bpy.types.Scene.sp_armor_registry=bpy.props.StringProperty(subtype='FILE_PATH')
    bpy.types.Scene.sp_armor_raw_dir=bpy.props.StringProperty(subtype='DIR_PATH')
    bpy.types.Scene.sp_armor_sequence=bpy.props.StringProperty(default="")

def unregister():
    for prop in ("sp_armor_registry","sp_armor_raw_dir","sp_armor_sequence"):delattr(bpy.types.Scene,prop)
    for cls in reversed(CLASSES):bpy.utils.unregister_class(cls)
