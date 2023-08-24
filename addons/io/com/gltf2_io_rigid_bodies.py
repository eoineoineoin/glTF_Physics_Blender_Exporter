from . import gltfProperty, from_vec, from_quat
from io_scene_gltf2.io.com.gltf2_io import from_union, from_list, from_none
from io_scene_gltf2.io.com.gltf2_io import from_str, from_float, from_int, from_bool
from io_scene_gltf2.io.com.gltf2_io import to_class
from mathutils import Vector, Quaternion

rigidBody_Extension_Name = "KHR_rigid_bodies"

# Enum values for friction/restitution combine modes
physics_material_combine_types = [
    ("AVERAGE", "Average", "", 0),
    ("MINIMUM", "Minimum", "", 1),
    ("MAXIMUM", "Maximum", "", 2),
    ("MULTIPLY", "Multiply", "", 3),
]


class PhysicsMaterial(gltfProperty):
    def __init__(self):
        super().__init__()
        self.static_friction = None
        self.dynamic_friction = None
        self.restitution = None
        self.friction_combine = None
        self.restitution_combine = None

    def to_dict(self):
        result = super().to_dict()
        result["staticFriction"] = from_union(
            [from_float, from_none], self.static_friction
        )
        result["dynamicFriction"] = from_union(
            [from_float, from_none], self.dynamic_friction
        )
        result["restitution"] = from_union([from_float, from_none], self.restitution)
        result["frictionCombine"] = from_union(
            [from_str, from_none], self.friction_combine
        )
        result["restitutionCombine"] = from_union(
            [from_str, from_none], self.restitution_combine
        )
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        result = PhysicsMaterial()
        result.static_friction = from_union(
            [from_float, from_none], obj.get("staticFriction")
        )
        result.dynamic_friction = from_union(
            [from_float, from_none], obj.get("dynamicFriction")
        )
        result.restitution = from_union([from_float, from_none], obj.get("restitution"))
        result.friction_combine = from_union(
            [from_str, from_none], obj.get("frictionCombine")
        )
        result.restitution_combine = from_union(
            [from_str, from_none], obj.get("restitutionCombine")
        )
        return result


class CollisionFilter(gltfProperty):
    def __init__(self):
        super().__init__()
        self.collision_systems = None
        self.collide_with_systems = None
        self.not_collide_systems = None

    def to_dict(self):
        result = super().to_dict()
        result["collisionSystems"] = from_union(
            [lambda x: from_list(from_str, x), from_none], self.collision_systems
        )
        result["collideWithSystems"] = from_union(
            [lambda x: from_list(from_str, x), from_none], self.collide_with_systems
        )
        result["notCollideWithSystems"] = from_union(
            [lambda x: from_list(from_str, x), from_none], self.not_collide_systems
        )
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        result = CollisionFilter()
        result.collision_systems = from_union(
            [lambda x: from_list(from_str, x), from_none], obj.get("collisionSystems")
        )
        result.collide_with_systems = from_union(
            [lambda x: from_list(from_str, x), from_none], obj.get("collideWithSystems")
        )
        result.not_collide_systems = from_union(
            [lambda x: from_list(from_str, x), from_none],
            obj.get("notCollideWithSystems"),
        )
        return result


class RigidMotion(gltfProperty):
    def __init__(self):
        super().__init__()
        self.is_kinematic = None
        self.mass = None
        self.center_of_mass = None
        self.inertia_diagonal = None
        self.inertia_orientation = None
        self.linear_velocity = None
        self.angular_velocity = None
        self.gravity_factor = None

    def to_dict(self):
        result = super().to_dict()
        result["isKinematic"] = from_union([from_bool, from_none], self.is_kinematic)
        result["mass"] = from_union([from_float, from_none], self.mass)
        result["centerOfMass"] = from_union([from_vec, from_none], self.center_of_mass)
        result["inertiaDiagonal"] = from_union(
            [from_vec, from_none], self.inertia_diagonal
        )
        result["inertiaOrientation"] = from_union(
            [from_quat, from_none], self.inertia_orientation
        )
        result["linearVelocity"] = from_union(
            [from_vec, from_none], self.linear_velocity
        )
        result["angularVelocity"] = from_union(
            [from_vec, from_none], self.angular_velocity
        )
        result["gravityFactor"] = from_union(
            [from_float, from_none], self.gravity_factor
        )
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        if obj == None:
            return None
        result = RigidMotion()
        result.is_kinematic = from_union([from_bool, from_none], obj.get("isKinematic"))
        result.mass = from_union([from_float, from_none], obj.get("mass"))
        result.center_of_mass = from_union(
            [lambda x: Vector(from_list(from_float, x)), from_none],
            obj.get("centerOfMass"),
        )
        result.inertia_diagonal = from_union(
            [lambda x: Vector(from_list(from_float, x)), from_none],
            obj.get("inertiaDiagonal"),
        )
        result.inertia_orientation = from_union(
            [lambda x: Quaternion(from_list(from_float, x)), from_none],
            obj.get("inertiaRotation"),
        )
        result.linear_velocity = from_union(
            [lambda x: Vector(from_list(from_float, x)), from_none],
            obj.get("linearVelocity"),
        )
        result.angular_velocity = from_union(
            [lambda x: Vector(from_list(from_float, x)), from_none],
            obj.get("angularVelocity"),
        )
        result.gravity_factor = from_union(
            [from_float, from_none], obj.get("gravityFactor")
        )
        return result


class JointLimit(gltfProperty):
    def __init__(self):
        super().__init__()
        self.angular_axes = None
        self.linear_axes = None
        self.min_limit = None
        self.max_limit = None

    @staticmethod
    def Linear(axes, minLimit=None, maxLimit=None):
        result = JointLimit()
        result.linear_axes = axes
        result.min_limit = minLimit
        result.max_limit = maxLimit
        return result

    @staticmethod
    def Angular(axes, minLimit=None, maxLimit=None):
        result = JointLimit()
        result.angular_axes = axes
        result.min_limit = minLimit
        result.max_limit = maxLimit
        return result

    def to_dict(self):
        result = super().to_dict()
        result["linearAxes"] = from_union(
            [lambda x: from_list(from_int, x), from_none], self.linear_axes
        )
        result["angularAxes"] = from_union(
            [lambda x: from_list(from_int, x), from_none], self.angular_axes
        )
        result["min"] = from_union([from_float, from_none], self.min_limit)
        result["max"] = from_union([from_float, from_none], self.max_limit)
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        limit = JointLimit()
        limit.angular_axes = from_union(
            [lambda x: from_list(from_int, x), from_none], obj.get("angularAxes")
        )
        limit.linear_axes = from_union(
            [lambda x: from_list(from_int, x), from_none], obj.get("linearAxes")
        )
        limit.min_limit = from_union([from_float, from_none], obj.get("min"))
        limit.max_limit = from_union([from_float, from_none], obj.get("max"))
        return limit


class JointLimitSet(gltfProperty):
    def __init__(self, limits=None):
        super().__init__()
        self.joint_limits = limits if limits != None else list()

    def to_dict(self):
        result = super().to_dict()
        result["limits"] = from_union(
            [lambda x: from_list(lambda l: to_class(JointLimit, l), x), from_none],
            self.joint_limits,
        )
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        result = JointLimitSet()
        result.joint_limits = from_union(
            [lambda x: from_list(JointLimit.from_dict, x), from_none], obj.get("limits")
        )
        return result


class Joint(gltfProperty):
    def __init__(self):
        super().__init__()
        self.connected_node = -1
        self.joint_limits = -1
        self.enable_collision = True

    def to_dict(self):
        result = super().to_dict()
        result["connectedNode"] = self.connected_node
        result["jointLimits"] = self.joint_limits
        result["enableCollision"] = from_union(
            [from_bool, from_none], self.enable_collision
        )
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        if obj == None:
            return None
        joint = Joint()
        joint.connected_node = from_union(
            [from_int, from_none], obj.get("connectedNode")
        )
        joint.joint_limits = from_union([from_int, from_none], obj.get("jointLimits"))
        joint.enable_collision = from_union(
            [from_bool, from_none], obj.get("enableCollision")
        )
        return joint


class RigidBodiesNodeExtension(gltfProperty):
    def __init__(self):
        super().__init__()
        self.rigid_motion = None
        self.collider = None
        self.trigger = None
        self.physics_material = None
        self.joint = None

    def to_dict(self):
        result = super().to_dict()
        result["motion"] = from_union(
            [lambda x: to_class(RigidMotion, x), from_none], self.rigid_motion
        )
        result["collider"] = from_union(
            [lambda x: to_class(RigidBodiesNodeExtension.Collider, x), from_none],
            self.collider,
        )
        result["trigger"] = from_union(
            [lambda x: to_class(RigidBodiesNodeExtension.Trigger, x), from_none],
            self.trigger,
        )
        result["joint"] = from_union(
            [lambda x: to_class(Joint, x), from_none], self.joint
        )
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        result = (
            RigidBodiesNodeExtension()
        )  # <todo.eoin Need to handle extensions/extras in all from_dict() methods
        result.rigid_motion = from_union(
            [RigidMotion.from_dict, from_none], obj.get("motion")
        )
        result.collider = from_union(
            [RigidBodiesNodeExtension.Collider.from_dict, from_none],
            obj.get("collider"),
        )
        result.trigger = from_union(
            [RigidBodiesNodeExtension.Trigger.from_dict, from_none], obj.get("trigger")
        )
        result.joint = from_union([Joint.from_dict, from_none], obj.get("joint"))
        return result

    class Collider(gltfProperty):
        def __init__(self):
            super().__init__()
            self.shape = None
            self.physics_material = None
            self.collision_filter = None

        def to_dict(self):
            result = super().to_dict()
            result["shape"] = self.shape
            result["physicsMaterial"] = self.physics_material
            result["collisionFilter"] = self.collision_filter
            return result

        @staticmethod
        def from_dict(obj):
            assert isinstance(obj, dict)
            result = (
                RigidBodiesNodeExtension.Collider()
            )  # <todo.eoin Need to handle extensions/extras in all from_dict() methods
            result.shape = from_union([from_int, from_none], obj.get("shape"))
            result.physics_material = from_union(
                [from_int, from_none], obj.get("physicsMaterial")
            )
            result.collision_filter = from_union(
                [from_int, from_none], obj.get("collisionFilter")
            )
            return result

    class Trigger(gltfProperty):
        def __init__(self):
            super().__init__()
            self.shape = None
            self.collision_filter = None

        def to_dict(self):
            result = super().to_dict()
            result["shape"] = self.shape
            result["collisionFilter"] = self.collision_filter
            return result

        @staticmethod
        def from_dict(obj):
            assert isinstance(obj, dict)
            result = (
                RigidBodiesNodeExtension.Trigger()
            )  # <todo.eoin Need to handle extensions/extras in all from_dict() methods
            result.shape = from_union([from_int, from_none], obj.get("shape"))
            result.collision_filter = from_union(
                [from_int, from_none], obj.get("collisionFilter")
            )
            return result


class RigidBodiesGlTFExtension:
    def __init__(self):
        self.physics_materials = []
        self.physics_joint_limits = []
        self.collision_filters = []

    def should_export(self):
        return (
            len(self.physics_materials) > 0
            or len(self.physics_joint_limits) > 0
            or len(self.collision_filters) > 0
        )

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        result = RigidBodiesGlTFExtension()
        result.physics_materials = from_union(
            [lambda x: from_list(PhysicsMaterial.from_dict, x), from_none],
            obj.get("physicsMaterials"),
        )
        result.physics_joint_limits = from_union(
            [lambda x: from_list(JointLimitSet.from_dict, x), from_none],
            obj.get("physicsJointLimits"),
        )
        result.collision_filters = from_union(
            [lambda x: from_list(CollisionFilter.from_dict, x), from_none],
            obj.get("collisionFilters"),
        )
        return result
