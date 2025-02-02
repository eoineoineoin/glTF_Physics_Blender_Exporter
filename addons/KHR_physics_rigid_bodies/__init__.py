import bpy

from .blender.com import gltf2_blender_rigid_bodies_ui as rb_extra_ui
from .blender.exp.gltf2_blender_rigid_bodies import glTF2ExportUserExtension
from .blender.imp.gltf2_blender_rigid_bodies import glTF2ImportUserExtension
from .io.com.gltf2_io_rigid_bodies import (
    rigidBody_Extension_Name
)

bl_info = {
    "name": "KHR_physics_rigid_bodies",
    "category": "Import-Export",
    "version": (0, 2, 2),
    "blender": (4, 2, 0),
    "location": "File > Export > glTF 2.0",
    "description": "Extension for adding rigid body information to exported glTF file",
    "tracker_url": "https://github.com/eoineoineoin/glTF_Physics_Blender_Exporter/issues",
    "isDraft": True,
    "developer": "Eoin Mcloughlin (Havok)",
    "url": "https://github.com/eoineoineoin/glTF_Physics_Blender_Exporter",
}

def draw_export(context, layout):
    exportProps = bpy.context.scene.khr_physics_exporter_props
    header, body = layout.panel("KHR_physics_rigid_bodies_exporter", default_closed=True)
    header.use_property_split = False
    header.prop(exportProps, "enabled", text="")
    header.label(text=rigidBody_Extension_Name)
    if body:
        body.active = exportProps.enabled
        body.prop(exportProps, "reparent_bones")

def draw_import(context, layout):
    importProps = bpy.context.scene.khr_physics_importer_props
    header, body = layout.panel("KHR_physics_rigid_bodies_importer", default_closed=False)
    header.use_property_split = False
    header.prop(importProps, 'enabled')
    header.active = importProps.enabled

def register():
    rb_extra_ui.register_ui()

def unregister():
    rb_extra_ui.unregister_ui()

