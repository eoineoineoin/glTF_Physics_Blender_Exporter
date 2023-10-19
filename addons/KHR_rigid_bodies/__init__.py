import bpy

from .blender.com import gltf2_blender_rigid_bodies_ui as rb_extra_ui
from .blender.exp.gltf2_blender_rigid_bodies import glTF2ExportUserExtension
from .blender.imp.gltf2_blender_rigid_bodies import glTF2ImportUserExtension

bl_info = {
    "name": "KHR_rigid_bodies",
    "category": "Import-Export",
    "version": (0, 0, 2),
    "blender": (3, 6, 0),
    "location": "File > Export > glTF 2.0",
    "description": "Extension for adding rigid body information to exported glTF file",
    "tracker_url": "https://github.com/eoineoineoin/glTF_Physics_Blender_Exporter/issues",
    "isDraft": True,
    "developer": "Eoin Mcloughlin (Havok)",
    "url": "https://github.com/eoineoineoin/glTF_Physics_Blender_Exporter",
}


def register():
    rb_extra_ui.register_ui()


def register_panel():
    # Register the panel on demand, we need to be sure to only register it once
    # This is necessary because the panel is a child of the extensions panel,
    # which may not be registered when we try to register this extension
    try:
        bpy.utils.register_class(GLTF_PT_KHR_Rigid_Bodies_ImportExtensionPanel)
        bpy.utils.register_class(GLTF_PT_KHR_Rigid_Bodies_ExportExtensionPanel)
    except Exception:
        pass

    # If the glTF exporter is disabled, we need to unregister the extension panel
    # Just return a function to the exporter so it can unregister the panel
    return unregister_panel


def unregister_panel():
    # Since panel is registered on demand, it is possible it is not registered
    for p in (
        GLTF_PT_KHR_Rigid_Bodies_ExportExtensionPanel,
        GLTF_PT_KHR_Rigid_Bodies_ImportExtensionPanel,
    ):
        try:
            bpy.utils.unregister_class(p)
        except Exception:
            pass


def unregister():
    unregister_panel()
    rb_extra_ui.unregister_ui()


class GLTF_PT_KHR_Rigid_Bodies_ExportExtensionPanel(bpy.types.Panel):
    bl_space_type = "FILE_BROWSER"
    bl_region_type = "TOOL_PROPS"
    bl_label = "Enabled"
    bl_parent_id = "GLTF_PT_export_user_extensions"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        sfile = context.space_data
        operator = sfile.active_operator
        return operator.bl_idname == "EXPORT_SCENE_OT_gltf"

    def draw_header(self, context):
        props = bpy.context.scene.khr_physics_exporter_props
        self.layout.prop(props, "enabled")

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False  # No animation.

        props = bpy.context.scene.khr_physics_exporter_props
        layout.active = props.enabled
        layout.prop(props, "reparent_bones")


class GLTF_PT_KHR_Rigid_Bodies_ImportExtensionPanel(bpy.types.Panel):
    bl_space_type = "FILE_BROWSER"
    bl_region_type = "TOOL_PROPS"
    bl_label = "Enabled"
    bl_parent_id = "GLTF_PT_import_user_extensions"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        sfile = context.space_data
        operator = sfile.active_operator
        return operator.bl_idname == "IMPORT_SCENE_OT_gltf"

    def draw_header(self, context):
        props = bpy.context.scene.khr_physics_importer_props
        self.layout.prop(props, "enabled")

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = False
        layout.use_property_decorate = False  # No animation.

        props = bpy.context.scene.khr_physics_importer_props
        layout.active = props.enabled
