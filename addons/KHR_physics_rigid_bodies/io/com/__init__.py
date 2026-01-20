# These are safe to import here, as this file is only imported from
# code that is run by io_scene_gltf2, so it must be loaded already
from io_scene_gltf2.io.com.gltf2_io import from_union, from_dict, from_list
from io_scene_gltf2.io.com.gltf2_io import from_float, from_none
from mathutils import Quaternion, Vector


def from_vec(x):
    """Utility to convert a vector, in the style of gltf2_io"""
    assert isinstance(x, Vector)
    return from_list(from_float, list(x.to_tuple()))


def from_quat(x):
    """Utility to convert a quaternion, in the style of gltf2_io"""
    assert isinstance(x, Quaternion)
    return from_list(from_float, [x.x, x.y, x.z, x.w])
