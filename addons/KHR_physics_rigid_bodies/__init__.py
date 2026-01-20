import bpy

bl_info = {
    "name": "KHR_physics_rigid_bodies",
    "category": "Import-Export",
    "version": (0, 3, 0),
    "blender": (4, 3, 0),
    "location": "File > Export > glTF 2.0",
    "description": "Extension for adding rigid body information to exported glTF file",
    "tracker_url": "https://github.com/eoineoineoin/glTF_Physics_Blender_Exporter/issues",
    "isDraft": True,
    "developer": "Eoin Mcloughlin (Havok)",
    "url": "https://github.com/eoineoineoin/glTF_Physics_Blender_Exporter",
}


def draw_export(context, layout):
    exportProps = bpy.context.scene.khr_physics_exporter_props
    header, body = layout.panel(
        "KHR_physics_rigid_bodies_exporter", default_closed=False
    )
    header.use_property_split = False
    header.prop(exportProps, "enabled")
    header.active = exportProps.enabled
    if body != None:
        body.use_property_split = False
        body.prop(exportProps, "reparent_bones")


def draw_import(context, layout):
    importProps = bpy.context.scene.khr_physics_importer_props
    header, body = layout.panel(
        "KHR_physics_rigid_bodies_importer", default_closed=False
    )
    header.use_property_split = False
    header.prop(importProps, "enabled")
    header.active = importProps.enabled


from .blender.com import gltf2_blender_rigid_bodies_ui as rb_extra_ui
from .blender.com import gltf2_blender_rigid_bodies_ops as rb_ops
from .blender.exp.gltf2_blender_rigid_bodies import glTF2ExportUserExtension
from .blender.imp.gltf2_blender_rigid_bodies import glTF2ImportUserExtension

# This duplicates the name in io.com.gltf2_io_rigid_bodies, but we can't import that yet
rigidBody_Extension_Name = "KHR_physics_rigid_bodies"


def register():
    rb_ops.register_ops()
    rb_extra_ui.register_ui()
    try:
        from io_scene_gltf2 import (
            exporter_extension_layout_draw,
            importer_extension_layout_draw,
        )

        exporter_extension_layout_draw[rigidBody_Extension_Name] = draw_export
        importer_extension_layout_draw[rigidBody_Extension_Name] = draw_import
    except ModuleNotFoundError:
        pass  # io_scene_gltf2 not loaded/enabled; can't register draw functions


def unregister():
    rb_ops.unregister_ops()
    rb_extra_ui.unregister_ui()
    try:
        from io_scene_gltf2 import (
            exporter_extension_layout_draw,
            importer_extension_layout_draw,
        )

        del exporter_extension_layout_draw[rigidBody_Extension_Name]
        del importer_extension_layout_draw[rigidBody_Extension_Name]
    except ModuleNotFoundError:
        pass  # io_scene_gltf2 not loaded/enabled; can't unregister draw functions
