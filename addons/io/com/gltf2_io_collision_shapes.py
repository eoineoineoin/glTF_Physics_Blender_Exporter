from . import gltfProperty, from_vec
from io_scene_gltf2.io.com.gltf2_io import from_union, from_none, from_float
from io_scene_gltf2.io.com.gltf2_io import from_str, from_list, from_int
from io_scene_gltf2.io.com.gltf2_io import to_class
from mathutils import Vector

collisionGeom_Extension_Name = "KHR_collision_shapes"


class Shape(gltfProperty):
    def __init__(self):
        super().__init__()
        self.type = None
        self.sphere = None
        self.box = None
        self.capsule = None
        self.cylinder = None
        self.convex = None
        self.trimesh = None

    def to_dict(self):
        result = super().to_dict()
        result["type"] = from_union([from_str, from_none], self.type)
        result["sphere"] = from_union(
            [lambda x: to_class(Shape.Sphere, x), from_none], self.sphere
        )
        result["box"] = from_union(
            [lambda x: to_class(Shape.Box, x), from_none], self.box
        )
        result["capsule"] = from_union(
            [lambda x: to_class(Shape.Capsule, x), from_none], self.capsule
        )
        result["cylinder"] = from_union(
            [lambda x: to_class(Shape.Cylinder, x), from_none], self.cylinder
        )
        result["convex"] = from_union(
            [lambda x: to_class(Shape.Convex, x), from_none], self.convex
        )
        result["trimesh"] = from_union(
            [lambda x: to_class(Shape.TriMesh, x), from_none], self.trimesh
        )
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        result = Shape()
        result.type = from_union([from_str, from_none], obj.get("type"))
        result.sphere = from_union(
            [Shape.Sphere.from_dict, from_none], obj.get("sphere")
        )
        result.box = from_union([Shape.Box.from_dict, from_none], obj.get("box"))
        result.capsule = from_union(
            [Shape.Capsule.from_dict, from_none], obj.get("capsule")
        )
        result.cylinder = from_union(
            [Shape.Cylinder.from_dict, from_none], obj.get("cylinder")
        )
        result.convex = from_union(
            [Shape.Convex.from_dict, from_none], obj.get("convex")
        )
        result.trimesh = from_union(
            [Shape.TriMesh.from_dict, from_none], obj.get("trimesh")
        )
        return result

    class Sphere(gltfProperty):
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
            return Shape.Sphere(radius)

    class Box(gltfProperty):
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
            return Shape.Box(size)

    class Capsule(gltfProperty):
        def __init__(self, height=0.5, radiusBottom=0.25, radiusTop=0.25):
            super().__init__()
            self.height = height
            self.radiusBottom = radiusBottom
            self.radiusTop = radiusTop

        def to_dict(self):
            result = super().to_dict()
            result["height"] = from_union([from_float, from_none], self.height)
            result["radiusBottom"] = from_union(
                [from_float, from_none], self.radiusBottom
            )
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
            return Shape.Capsule(height, radiusBottom, radiusTop)

    class Cylinder(gltfProperty):
        def __init__(self, height=0.5, radiusBottom=0.25, radiusTop=0.25):
            super().__init__()
            self.height = height
            self.radiusBottom = radiusBottom
            self.radiusTop = radiusTop

        def to_dict(self):
            result = super().to_dict()
            result["height"] = from_union([from_float, from_none], self.height)
            result["radiusBottom"] = from_union(
                [from_float, from_none], self.radiusBottom
            )
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
            return Shape.Cylinder(height, radiusBottom, radiusTop)

    class Convex(gltfProperty):
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
            return Shape.Convex(mesh)

    class TriMesh(gltfProperty):
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
            return Shape.TriMesh(mesh)


class CollisionGeomGlTFExtension:
    def __init__(self):
        self.shapes = []

    def should_export(self):
        return len(self.shapes) > 0

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        result = CollisionGeomGlTFExtension()
        result.shapes = from_union(
            [lambda x: from_list(Shape.from_dict, x), from_none], obj.get("shapes")
        )
        return result
