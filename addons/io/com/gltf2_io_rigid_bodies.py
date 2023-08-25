from . import gltfProperty, from_vec, from_quat
from io_scene_gltf2.io.com.gltf2_io import from_union, from_list, from_none
from io_scene_gltf2.io.com.gltf2_io import from_str, from_float, from_int, from_bool
from io_scene_gltf2.io.com.gltf2_io import to_class
from io_scene_gltf2.io.com.gltf2_io import Node
from io_scene_gltf2.io.com.gltf2_io_extensions import ChildOfRootExtension
from mathutils import Vector, Quaternion
from typing import Optional, Union

rigidBody_Extension_Name = "KHR_rigid_bodies"

# Enum values for friction/restitution combine modes
physics_material_combine_types = [
    ("AVERAGE", "Average", "", 0),
    ("MINIMUM", "Minimum", "", 1),
    ("MAXIMUM", "Maximum", "", 2),
    ("MULTIPLY", "Multiply", "", 3),
]


class Material(gltfProperty):
    static_friction: Optional[float] = None
    dynamic_friction: Optional[float] = None
    restitution: Optional[float] = None
    friction_combine: Optional[str] = None
    restitution_combine: Optional[str] = None

    def __init__(self):
        super().__init__()

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
        result = Material()
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
    collision_systems: Optional[list[str]] = None
    collide_with_systems: Optional[list[str]] = None
    not_collide_systems: Optional[list[str]] = None

    def __init__(self):
        super().__init__()

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


class Motion(gltfProperty):
    is_kinematic: Optional[bool] = None
    mass: Optional[float] = None
    center_of_mass: Optional[Vector] = None
    inertia_diagonal: Optional[Vector] = None
    inertia_orientation: Optional[Quaternion] = None
    linear_velocity: Optional[Vector] = None
    angular_velocity: Optional[Vector] = None
    gravity_factor: Optional[float] = None

    def __init__(self):
        super().__init__()

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
        result = Motion()
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
    angular_axes: Optional[list[int]] = None
    linear_axes: Optional[list[int]] = None
    min_limit: Optional[float] = None
    max_limit: Optional[float] = None

    def __init__(self):
        super().__init__()

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
    limits: list[JointLimit]

    def __init__(self, limits: Optional[list[JointLimit]] = None):
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


class Collider(gltfProperty):
    shape: Optional[Union[int, ChildOfRootExtension]] = None
    physics_material: Optional[Union[int, ChildOfRootExtension]] = None
    collision_filter: Optional[Union[int, ChildOfRootExtension]] = None

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
            Collider()
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
    shape: Optional[Union[int, ChildOfRootExtension]] = None
    collision_filter: Optional[Union[int, ChildOfRootExtension]] = None

    def to_dict(self):
        result = super().to_dict()
        result["shape"] = self.shape
        result["collisionFilter"] = self.collision_filter
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        result = Trigger()  # <todo.eoin Need to handle extensions/extras
        result.shape = from_union([from_int, from_none], obj.get("shape"))
        result.collision_filter = from_union(
            [from_int, from_none], obj.get("collisionFilter")
        )
        return result


class Joint(gltfProperty):
    connected_node: Optional[Union[int, Node]] = None
    joint_limits: Optional[Union[int, ChildOfRootExtension]] = None
    enable_collision: Optional[bool] = None

    def __init__(self):
        super().__init__()

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
    motion: Optional[Motion] = None
    collider: Optional[Collider] = None
    trigger: Optional[Trigger] = None
    joint: Optional[Joint] = None

    def __init__(self):
        super().__init__()

    def to_dict(self):
        result = super().to_dict()
        result["motion"] = from_union(
            [lambda x: to_class(Motion, x), from_none], self.motion
        )
        result["collider"] = from_union(
            [lambda x: to_class(Collider, x), from_none],
            self.collider,
        )
        result["trigger"] = from_union(
            [lambda x: to_class(Trigger, x), from_none],
            self.trigger,
        )
        result["joint"] = from_union(
            [lambda x: to_class(Joint, x), from_none], self.joint
        )
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        result = RigidBodiesNodeExtension()
        # <todo.eoin Need to handle extensions/extras in all from_dict() methods
        result.motion = from_union([Motion.from_dict, from_none], obj.get("motion"))
        result.collider = from_union(
            [Collider.from_dict, from_none], obj.get("collider")
        )
        result.trigger = from_union([Trigger.from_dict, from_none], obj.get("trigger"))
        result.joint = from_union([Joint.from_dict, from_none], obj.get("joint"))
        return result


class RigidBodiesGlTFExtension:
    materials: list[Material] = []
    joints: list[JointLimitSet] = []
    collision_filters: list[CollisionFilter] = []

    def should_export(self):
        return (
            len(self.materials) > 0
            or len(self.joints) > 0
            or len(self.collision_filters) > 0
        )

    def to_dict(self):
        result = {}
        if len(self.materials):
            result["physicsMaterials"] = from_list(
                lambda x: to_class(Material, x), self.materials
            )
        if len(self.joints):
            result["physicsMaterials"] = from_list(
                lambda x: to_class(JointLimitSet, x), self.joints
            )
        if len(self.collision_filters):
            result["physicsMaterials"] = from_list(
                lambda x: to_class(CollisionFilter, x), self.joints
            )
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        result = RigidBodiesGlTFExtension()
        result.materials = from_union(
            [lambda x: from_list(Material.from_dict, x), from_none],
            obj.get("physicsMaterials"),
        )
        result.joints = from_union(
            [lambda x: from_list(JointLimitSet.from_dict, x), from_none],
            obj.get("physicsJointLimits"),
        )
        result.collision_filters = from_union(
            [lambda x: from_list(CollisionFilter.from_dict, x), from_none],
            obj.get("collisionFilters"),
        )
        return result
