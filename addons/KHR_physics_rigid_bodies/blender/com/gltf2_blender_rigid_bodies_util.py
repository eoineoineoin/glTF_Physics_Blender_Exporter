import bpy
from typing import Tuple
from mathutils import Vector

def accessMeshData(node, apply_modifiers):
    """RAII-style function to access mesh data with modifiers attached"""

    class ScopedMesh:
        def __init__(self, node, apply_modifiers: bool):
            self.node = node
            self.apply_modifiers = apply_modifiers
            self.modifiedNode = None

        def __enter__(self):
            if self.apply_modifiers:
                depsGraph = bpy.context.evaluated_depsgraph_get()
                self.modifiedNode = node.evaluated_get(depsGraph)
                return self.modifiedNode.to_mesh(
                    preserve_all_data_layers=True, depsgraph=depsGraph
                )
            else:
                return self.node.data

        def __exit__(self, *args):
            if self.modifiedNode:
                self.modifiedNode.to_mesh_clear()

    return ScopedMesh(node, apply_modifiers)

def calculate_cone_capsule_params(node, meshData) -> Tuple[float, float, float]:
    """Given a mesh data, calculate suitable cone/capsule parameters.
    This attempts to mimic Blender's behaviour when generating these shapes,
    which does not generate an optimal shape for a given mesh. Returns a
    tuple of height, top radius and bottom radius."""
    # User hasn't overridden shape params, so we need to calculate them
    # Maybe there's a way to extract them from Blender?
    primaryAxis = Vector(
        (0, 0, 1)
    )  # Use blender's up axis, instead of glTF (and transform later)
    maxHalfHeight = 0
    maxRadiusSquared = 0
    for v in meshData.vertices:
        maxHalfHeight = max(maxHalfHeight, abs(v.co.dot(primaryAxis)))
        radiusSquared = (
            v.co - primaryAxis * v.co.dot(primaryAxis)
        ).length_squared
        maxRadiusSquared = max(maxRadiusSquared, radiusSquared)
    height = maxHalfHeight * 2
    radiusBottom = maxRadiusSquared**0.5
    radiusTop = (
        radiusBottom if node.rigid_body.collision_shape != "CONE" else 0
    )
    if node.rigid_body.collision_shape == "CAPSULE":
        height = height - radiusBottom * 2
    return height, radiusTop, radiusBottom

