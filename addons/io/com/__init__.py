from io_scene_gltf2.io.com.gltf2_io import from_union, from_dict, from_list
from io_scene_gltf2.io.com.gltf2_io import from_float, from_none
from mathutils import Quaternion, Vector


class gltfProperty:
    def __init__(self):
        self.extensions = None
        self.extras = None

    def to_dict(self):
        result = {}
        result["extensions"] = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            self.extensions,
        )
        result["extras"] = self.extras
        return result


def from_vec(x):
    """Utility to convert a vector, in the style of gltf2_io"""
    assert isinstance(x, Vector)
    return from_list(from_float, list(x.to_tuple()))


def from_quat(x):
    """Utility to convert a quaternion, in the style of gltf2_io"""
    assert isinstance(x, Quaternion)
    return from_list(from_float, list(x))
