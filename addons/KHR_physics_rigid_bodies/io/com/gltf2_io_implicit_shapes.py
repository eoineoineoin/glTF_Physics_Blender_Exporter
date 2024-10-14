from . import from_vec
from io_scene_gltf2.io.com.gltf2_io import from_union, from_none, from_float
from io_scene_gltf2.io.com.gltf2_io import from_str, from_list, from_dict
from io_scene_gltf2.io.com.gltf2_io import to_class, from_extension, from_extra
from mathutils import Vector
from typing import Optional, Dict, Any

implicitShapes_Extension_Name = "KHR_implicit_shapes"


class Sphere:
    radius: Optional[float] = 0.5
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

    def __init__(self, radius=0.5):
        self.radius = radius

    def to_dict(self):
        result = {}
        result["radius"] = self.radius
        result["extensions"] = from_union(
            [lambda x: from_dict(from_extension, x), from_none], self.extensions
        )
        result["extras"] = from_extra(self.extras)
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        if obj == None:
            return None
        radius = from_union([from_float, from_none], obj.get("radius"))
        result = Sphere(radius)
        result.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        result.extras = obj.get("extras")
        return result


class Box:
    size: Optional[Vector] = Vector((1.0, 1.0, 1.0))
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

    def __init__(self, size=Vector((1.0, 1.0, 1.0))):
        self.size = size

    def to_dict(self):
        result = {}
        result["size"] = from_union([from_vec, from_none], self.size)
        result["extensions"] = from_union(
            [lambda x: from_dict(from_extension, x), from_none], self.extensions
        )
        result["extras"] = from_extra(self.extras)
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        if obj == None:
            return None
        size = from_union(
            [lambda x: Vector(from_list(from_float, x)), from_none], obj.get("size")
        )
        result = Box(size)
        result.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        result.extras = obj.get("extras")
        return result


class Capsule:
    height: Optional[float] = 0.5
    radius_bottom: Optional[float] = 0.25
    radius_top: Optional[float] = 0.25
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

    def __init__(self, height=0.5, radiusBottom=0.25, radiusTop=0.25):
        self.height = height
        self.radiusBottom = radiusBottom
        self.radiusTop = radiusTop

    def to_dict(self):
        result = {}
        result["height"] = from_union([from_float, from_none], self.height)
        result["radiusBottom"] = from_union([from_float, from_none], self.radiusBottom)
        result["radiusTop"] = from_union([from_float, from_none], self.radiusTop)
        result["extensions"] = from_union(
            [lambda x: from_dict(from_extension, x), from_none], self.extensions
        )
        result["extras"] = from_extra(self.extras)
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        if obj == None:
            return None
        height = from_union([from_float, from_none], obj.get("height"))
        radiusBottom = from_union([from_float, from_none], obj.get("radiusBottom"))
        radiusTop = from_union([from_float, from_none], obj.get("radiusTop"))
        result = Capsule(height, radiusBottom, radiusTop)
        result.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        result.extras = obj.get("extras")
        return result


class Cylinder:
    height: Optional[float] = 0.5
    radiusBottom: Optional[float] = 0.25
    radiusTop: Optional[float] = 0.25
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

    def __init__(self, height=0.5, radiusBottom=0.25, radiusTop=0.25):
        self.height = height
        self.radiusBottom = radiusBottom
        self.radiusTop = radiusTop

    def to_dict(self):
        result = {}
        result["height"] = from_union([from_float, from_none], self.height)
        result["radiusBottom"] = from_union([from_float, from_none], self.radiusBottom)
        result["radiusTop"] = from_union([from_float, from_none], self.radiusTop)
        result["extensions"] = from_union(
            [lambda x: from_dict(from_extension, x), from_none], self.extensions
        )
        result["extras"] = from_extra(self.extras)
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        if obj == None:
            return None
        height = from_union([from_float, from_none], obj.get("height"))
        radiusBottom = from_union([from_float, from_none], obj.get("radiusBottom"))
        radiusTop = from_union([from_float, from_none], obj.get("radiusTop"))
        result = Cylinder(height, radiusBottom, radiusTop)
        result.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        result.extras = obj.get("extras")
        return result

class Shape:
    type: Optional[str] = None
    sphere: Optional[Sphere] = None
    box: Optional[Box] = None
    capsule: Optional[Capsule] = None
    cylinder: Optional[Cylinder] = None
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

    def to_dict(self):
        result = {}
        result["type"] = from_union([from_str, from_none], self.type)
        result["sphere"] = from_union(
            [lambda x: to_class(Sphere, x), from_none], self.sphere
        )
        result["box"] = from_union([lambda x: to_class(Box, x), from_none], self.box)
        result["capsule"] = from_union(
            [lambda x: to_class(Capsule, x), from_none], self.capsule
        )
        result["cylinder"] = from_union(
            [lambda x: to_class(Cylinder, x), from_none], self.cylinder
        )
        result["extensions"] = from_union(
            [lambda x: from_dict(from_extension, x), from_none], self.extensions
        )
        result["extras"] = from_extra(self.extras)
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        result = Shape()
        result.type = from_union([from_str, from_none], obj.get("type"))
        result.sphere = from_union([Sphere.from_dict, from_none], obj.get("sphere"))
        result.box = from_union([Box.from_dict, from_none], obj.get("box"))
        result.capsule = from_union([Capsule.from_dict, from_none], obj.get("capsule"))
        result.cylinder = from_union(
            [Cylinder.from_dict, from_none], obj.get("cylinder")
        )
        result.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        result.extras = obj.get("extras")
        return result


class ImplicitShapesGlTFExtension:
    shapes: list[Shape] = []
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

    def should_export(self):
        return len(self.shapes) > 0

    def to_dict(self):
        result = {}
        if len(self.shapes):
            result["shapes"] = from_list(lambda x: to_class(Shape, x), self.shapes)
        result["extensions"] = from_union(
            [lambda x: from_dict(from_extension, x), from_none], self.extensions
        )
        result["extras"] = from_extra(self.extras)
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        result = ImplicitShapesGlTFExtension()
        result.shapes = from_union(
            [lambda x: from_list(Shape.from_dict, x), from_none], obj.get("shapes")
        )
        result.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        result.extras = obj.get("extras")
        return result
