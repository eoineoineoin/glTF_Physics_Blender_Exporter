from . import from_vec
from io_scene_gltf2.io.com.gltf2_io import from_union, from_none, from_float, from_bool
from io_scene_gltf2.io.com.gltf2_io import from_str, from_list, from_int, from_dict
from io_scene_gltf2.io.com.gltf2_io import to_class, from_extension, from_extra
from mathutils import Vector
from typing import Optional, Dict, Any

collisionGeom_Extension_Name = "KHR_collision_shapes"


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


class Mesh:
    mesh: Optional[int] = None
    convexHull: Optional[bool] = None
    skin: Optional[int] = None
    # Todo: looks like blender never sets instance weights, always creating a new
    # mesh instance instead. Not sure this can ever be populated. Verify.
    weights: Optional[list[float]] = None
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

    def __init__(self, mesh, convexHull = None):
        self.mesh = mesh
        self.convexHull = convexHull

    def to_dict(self):
        result = {}
        result["mesh"] = self.mesh
        result["convexHull"] = from_union([from_bool, from_none], self.convexHull)
        result["skin"] = self.skin
        result["weights"] = from_union([ lambda x: from_list(lambda x: from_float(x), x), from_none], self.weights)
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
        mesh = from_union([from_int, from_none], obj.get("mesh"))
        convexHull = from_union([from_bool, from_none], obj.get("convexHull"))
        result = Mesh(mesh, convexHull=convexHull)
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
    mesh: Optional[Mesh] = None
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
        result["mesh"] = from_union(
            [lambda x: to_class(Mesh, x), from_none], self.mesh
        )
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
        result.mesh = from_union([Mesh.from_dict, from_none], obj.get("mesh"))
        result.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        result.extras = obj.get("extras")
        return result


class CollisionShapesGlTFExtension:
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
        result = CollisionShapesGlTFExtension()
        result.shapes = from_union(
            [lambda x: from_list(Shape.from_dict, x), from_none], obj.get("shapes")
        )
        result.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        result.extras = obj.get("extras")
        return result
