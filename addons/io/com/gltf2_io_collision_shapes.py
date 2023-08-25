from . import gltfProperty, from_vec
from io_scene_gltf2.io.com.gltf2_io import from_union, from_none, from_float
from io_scene_gltf2.io.com.gltf2_io import from_str, from_list, from_int
from io_scene_gltf2.io.com.gltf2_io import to_class
from mathutils import Vector
from typing import Optional

collisionGeom_Extension_Name = "KHR_collision_shapes"


class Sphere(gltfProperty):
    radius: Optional[float] = 0.5

    def __init__(self, radius=0.5):
        super().__init__()
        self.radius = radius

    def to_dict(self):
        result = super().to_dict()
        result["radius"] = self.radius
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        if obj == None:
            return None
        radius = from_union([from_float, from_none], obj.get("radius"))
        return Sphere(radius)


class Box(gltfProperty):
    size: Optional[Vector] = Vector((1.0, 1.0, 1.0))

    def __init__(self, size=Vector((1.0, 1.0, 1.0))):
        super().__init__()
        self.size = size

    def to_dict(self):
        result = {}
        result["size"] = from_union([from_vec, from_none], self.size)
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        if obj == None:
            return None
        size = from_union(
            [lambda x: Vector(from_list(from_float, x)), from_none], obj.get("size")
        )
        return Box(size)


class Capsule(gltfProperty):
    height: Optional[float] = 0.5
    radius_bottom: Optional[float] = 0.25
    radius_top: Optional[float] = 0.25

    def __init__(self, height=0.5, radiusBottom=0.25, radiusTop=0.25):
        super().__init__()
        self.height = height
        self.radiusBottom = radiusBottom
        self.radiusTop = radiusTop

    def to_dict(self):
        result = super().to_dict()
        result["height"] = from_union([from_float, from_none], self.height)
        result["radiusBottom"] = from_union([from_float, from_none], self.radiusBottom)
        result["radiusTop"] = from_union([from_float, from_none], self.radiusTop)
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        if obj == None:
            return None
        height = from_union([from_float, from_none], obj.get("height"))
        radiusBottom = from_union([from_float, from_none], obj.get("radiusBottom"))
        radiusTop = from_union([from_float, from_none], obj.get("radiusTop"))
        return Capsule(height, radiusBottom, radiusTop)


class Cylinder(gltfProperty):
    height: Optional[float] = 0.5
    radiusBottom: Optional[float] = 0.25
    radiusTop: Optional[float] = 0.25

    def __init__(self, height=0.5, radiusBottom=0.25, radiusTop=0.25):
        super().__init__()
        self.height = height
        self.radiusBottom = radiusBottom
        self.radiusTop = radiusTop

    def to_dict(self):
        result = super().to_dict()
        result["height"] = from_union([from_float, from_none], self.height)
        result["radiusBottom"] = from_union([from_float, from_none], self.radiusBottom)
        result["radiusTop"] = from_union([from_float, from_none], self.radiusTop)
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        if obj == None:
            return None
        height = from_union([from_float, from_none], obj.get("height"))
        radiusBottom = from_union([from_float, from_none], obj.get("radiusBottom"))
        radiusTop = from_union([from_float, from_none], obj.get("radiusTop"))
        return Cylinder(height, radiusBottom, radiusTop)


class Convex(gltfProperty):
    mesh: Optional[int] = None

    def __init__(self, mesh):
        super().__init__()
        self.mesh = mesh

    def to_dict(self):
        result = super().to_dict()
        result["mesh"] = self.mesh
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        if obj == None:
            return None
        mesh = from_union([from_int, from_none], obj.get("mesh"))
        return Convex(mesh)


class TriMesh(gltfProperty):
    mesh: Optional[int] = None

    def __init__(self, mesh):
        super().__init__()
        self.mesh = mesh

    def to_dict(self):
        result = super().to_dict()
        result["mesh"] = self.mesh
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        if obj == None:
            return None
        mesh = from_union([from_int, from_none], obj.get("mesh"))
        return TriMesh(mesh)


class Shape(gltfProperty):
    type: Optional[str] = None
    sphere: Optional[Sphere] = None
    box: Optional[Box] = None
    capsule: Optional[Capsule] = None
    cylinder: Optional[Cylinder] = None
    convex: Optional[Convex] = None
    trimesh: Optional[TriMesh] = None

    def __init__(self):
        super().__init__()

    def to_dict(self):
        result = super().to_dict()
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
        result["convex"] = from_union(
            [lambda x: to_class(Convex, x), from_none], self.convex
        )
        result["trimesh"] = from_union(
            [lambda x: to_class(TriMesh, x), from_none], self.trimesh
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
        result.convex = from_union([Convex.from_dict, from_none], obj.get("convex"))
        result.trimesh = from_union([TriMesh.from_dict, from_none], obj.get("trimesh"))
        return result


class CollisionShapesGlTFExtension:
    shapes: list[Shape] = []

    def should_export(self):
        return len(self.shapes) > 0

    def to_dict(self):
        result = {}
        if len(self.shapes):
            result["shapes"] = from_list(lambda x: to_class(Shape, x), self.shapes)
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        result = CollisionShapesGlTFExtension()
        result.shapes = from_union(
            [lambda x: from_list(Shape.from_dict, x), from_none], obj.get("shapes")
        )
        return result
