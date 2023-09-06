import bpy

from .blender.com.gltf2_blender_rigid_bodies_ui import *
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

draw_handler = None  # <todo.eoin Clean this up


def register():
    bpy.utils.register_class(KHR_rigid_body_exporter_properties)
    bpy.utils.register_class(KHR_rigid_body_importer_properties)
    bpy.utils.register_class(KHR_rigid_body_scene_properties)
    bpy.utils.register_class(KHR_rigid_body_node_properties)
    bpy.utils.register_class(KHR_MT_rigid_body_visualizer)
    bpy.utils.register_class(KHR_PT_rigid_body_panel)
    bpy.utils.register_class(KHR_PT_rigid_body_motion)
    bpy.utils.register_class(KHR_PT_rigid_body_shape)
    bpy.utils.register_class(KHR_PT_rigid_body_mass)
    bpy.types.Scene.khr_physics_exporter_props = bpy.props.PointerProperty(
        type=KHR_rigid_body_exporter_properties
    )
    bpy.types.Scene.khr_physics_importer_props = bpy.props.PointerProperty(
        type=KHR_rigid_body_importer_properties
    )
    bpy.types.Scene.khr_physics_scene_viewer_props = bpy.props.PointerProperty(
        type=KHR_rigid_body_scene_properties
    )
    bpy.types.Object.khr_physics_extra_props = bpy.props.PointerProperty(
        type=KHR_rigid_body_node_properties
    )
    global draw_handler
    draw_handler = bpy.types.SpaceView3D.draw_handler_add(
        viewportRenderHelper.drawExtraPhysicsProperties, (), "WINDOW", "POST_VIEW"
    )


def register_panel():
    # Register the panel on demand, we need to be sure to only register it once
    # This is necessary because the panel is a child of the extensions panel,
    # which may not be registered when we try to register this extension
    try:
        bpy.utils.register_class(KHR_Rigid_Bodies_ImportExtensionPanel)
        bpy.utils.register_class(KHR_Rigid_Bodies_ExportExtensionPanel)
    except Exception:
        pass

    # If the glTF exporter is disabled, we need to unregister the extension panel
    # Just return a function to the exporter so it can unregister the panel
    return unregister_panel


def unregister_panel():
    # Since panel is registered on demand, it is possible it is not registered
    for p in (KHR_Rigid_Bodies_ExportExtensionPanel, KHR_Rigid_Bodies_ImportExtensionPanel):
        try:
            bpy.utils.unregister_class(p)
        except Exception:
            pass


def unregister():
    unregister_panel()
    bpy.utils.unregister_class(KHR_rigid_body_exporter_properties)
    bpy.utils.unregister_class(KHR_rigid_body_importer_properties)
    bpy.utils.unregister_class(KHR_rigid_body_scene_properties)
    bpy.utils.unregister_class(KHR_rigid_body_node_properties)
    bpy.utils.unregister_class(KHR_MT_rigid_body_visualizer)
    bpy.utils.unregister_class(KHR_PT_rigid_body_panel)
    bpy.utils.unregister_class(KHR_PT_rigid_body_motion)
    bpy.utils.unregister_class(KHR_PT_rigid_body_shape)
    bpy.utils.unregister_class(KHR_PT_rigid_body_mass)
    del bpy.types.Scene.khr_physics_exporter_props
    del bpy.types.Scene.khr_physics_scene_viewer_props
    del bpy.types.Object.khr_physics_extra_props

    global draw_handler
    bpy.types.SpaceView3D.draw_handler_remove(draw_handler, "WINDOW")
    draw_handler = None


class KHR_Rigid_Bodies_ExportExtensionPanel(bpy.types.Panel):
    bl_space_type = "FILE_BROWSER"
    bl_region_type = "TOOL_PROPS"
    bl_label = "Enabled"
    bl_parent_id = "GLTF_PT_export_user_extensions"
    bl_options = set()

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
        layout.use_property_split = False
        layout.use_property_decorate = False  # No animation.

        props = bpy.context.scene.khr_physics_exporter_props
        layout.active = props.enabled


class KHR_Rigid_Bodies_ImportExtensionPanel(bpy.types.Panel):
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
