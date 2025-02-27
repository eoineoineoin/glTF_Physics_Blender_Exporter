import bpy
from .gltf2_blender_rigid_bodies_util import *


class CalculateConeCapsuleParams(bpy.types.Operator):
    """Derive cone/capsule parameters from a mesh"""

    bl_idname = "khr_physics_rigid_bodies.calculate_cone_capsule_params"
    bl_label = "Calculate Cone/Capsule Parameters"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        node = bpy.context.active_object
        applyModifiers = True  # Possible a user does not want modifers applied?
        with accessMeshData(node, applyModifiers) as meshData:
            height, radiusTop, radiusBottom = calculate_cone_capsule_params(
                node, meshData
            )
            extra_props = node.khr_physics_extra_props
            extra_props.cone_capsule_height = height
            extra_props.cone_capsule_radius_bottom = radiusBottom
            extra_props.cone_capsule_radius_top = radiusTop
        return {"FINISHED"}


def register_ops():
    bpy.utils.register_class(CalculateConeCapsuleParams)


def unregister_ops():
    bpy.utils.unregister_class(CalculateConeCapsuleParams)
