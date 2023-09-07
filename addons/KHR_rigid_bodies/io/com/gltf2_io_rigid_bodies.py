from . import from_vec, from_quat
from io_scene_gltf2.io.com.gltf2_io import from_union, from_list, from_extension
from io_scene_gltf2.io.com.gltf2_io import from_str, from_float, from_int, from_bool
from io_scene_gltf2.io.com.gltf2_io import from_none, from_dict, to_class, from_extra
from io_scene_gltf2.io.com.gltf2_io import Node
from io_scene_gltf2.io.com.gltf2_io_extensions import ChildOfRootExtension
from mathutils import Vector, Quaternion
from typing import Optional, Union, Dict, Any

rigidBody_Extension_Name = "KHR_rigid_bodies"

# Enum values for friction/restitution combine modes
physics_material_combine_types = [
    ("AVERAGE", "Average", "", 0),
    ("MINIMUM", "Minimum", "", 1),
    ("MAXIMUM", "Maximum", "", 2),
    ("MULTIPLY", "Multiply", "", 3),
]


class Material:
    static_friction: Optional[float] = None
    dynamic_friction: Optional[float] = None
    restitution: Optional[float] = None
    friction_combine: Optional[str] = None
    restitution_combine: Optional[str] = None
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

    def __init__(self):
        super().__init__()

    def to_dict(self):
        result = {}
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
        result["extensions"] = from_union(
            [lambda x: from_dict(from_extension, x), from_none], self.extensions
        )
        result["extras"] = from_extra(self.extras)
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
        result.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        result.extras = obj.get("extras")
        return result


class CollisionFilter:
    collision_systems: Optional[list[str]] = None
    collide_with_systems: Optional[list[str]] = None
    not_collide_systems: Optional[list[str]] = None
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

    def __init__(self):
        super().__init__()

    def to_dict(self):
        result = {}
        result["collisionSystems"] = from_union(
            [lambda x: from_list(from_str, x), from_none], self.collision_systems
        )
        result["collideWithSystems"] = from_union(
            [lambda x: from_list(from_str, x), from_none], self.collide_with_systems
        )
        result["notCollideWithSystems"] = from_union(
            [lambda x: from_list(from_str, x), from_none], self.not_collide_systems
        )
        result["extensions"] = from_union(
            [lambda x: from_dict(from_extension, x), from_none], self.extensions
        )
        result["extras"] = from_extra(self.extras)
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
        result.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        result.extras = obj.get("extras")
        return result


class Motion:
    is_kinematic: Optional[bool] = None
    mass: Optional[float] = None
    center_of_mass: Optional[Vector] = None
    inertia_diagonal: Optional[Vector] = None
    inertia_orientation: Optional[Quaternion] = None
    linear_velocity: Optional[Vector] = None
    angular_velocity: Optional[Vector] = None
    gravity_factor: Optional[float] = None
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

    def __init__(self):
        super().__init__()

    def to_dict(self):
        result = {}
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
        result.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        result.extras = obj.get("extras")
        return result


class JointDrive:
    # TODO.eoin Should be 1/2/3/4D targets
    position_target: Optional[float] = None
    velocity_target: Optional[float] = None
    max_force: Optional[float] = None
    stiffness: Optional[float] = None
    damping: Optional[float] = None
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

    def to_dict(self):
        result = {}
        result["positionTarget"] = from_union(
            [from_float, from_none], self.position_target
        )
        result["velocityTarget"] = from_union(
            [from_float, from_none], self.velocity_target
        )
        result["maxForce"] = from_union([from_float, from_none], self.max_force)
        result["stiffness"] = from_union([from_float, from_none], self.stiffness)
        result["damping"] = from_union([from_float, from_none], self.damping)
        result["extensions"] = from_union(
            [lambda x: from_dict(from_extension, x), from_none], self.extensions
        )
        result["extras"] = from_extra(self.extras)
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        drive = JointDrive()
        drive.position_target = from_union(
            [from_float, from_none], obj.get("positionTarget")
        )
        drive.velocity_target = from_union(
            [from_float, from_none], obj.get("velocityTarget")
        )
        drive.max_force = from_union([from_float, from_none], obj.get("maxForce"))
        drive.stiffness = from_union([from_float, from_none], obj.get("stiffness"))
        drive.damping = from_union([from_float, from_none], obj.get("damping"))
        drive.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        drive.extras = obj.get("extras")
        return drive


class JointLimit:
    angular_axes: Optional[list[int]] = None
    linear_axes: Optional[list[int]] = None
    min_limit: Optional[float] = None
    max_limit: Optional[float] = None
    spring_constant: Optional[float] = None
    spring_damping: Optional[float] = None
    drive: Optional[JointDrive] = None
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

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
        result = {}
        result["linearAxes"] = from_union(
            [lambda x: from_list(from_int, x), from_none], self.linear_axes
        )
        result["angularAxes"] = from_union(
            [lambda x: from_list(from_int, x), from_none], self.angular_axes
        )
        result["min"] = from_union([from_float, from_none], self.min_limit)
        result["max"] = from_union([from_float, from_none], self.max_limit)
        result["springConstant"] = from_union(
            [from_float, from_none], self.spring_constant
        )
        result["springDamping"] = from_union(
            [from_float, from_none], self.spring_damping
        )
        result["drive"] = from_union(
            [lambda d: to_class(JointDrive, d), from_none], self.drive
        )
        result["extensions"] = from_union(
            [lambda x: from_dict(from_extension, x), from_none], self.extensions
        )
        result["extras"] = from_extra(self.extras)
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
        limit.spring_constant = from_union(
            [from_float, from_none], obj.get("springConstant")
        )
        limit.spring_damping = from_union(
            [from_float, from_none], obj.get("springDamping")
        )
        limit.drive = from_union([JointDrive.from_dict, from_none], obj.get("drive"))
        limit.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        limit.extras = obj.get("extras")
        return limit


class JointLimitSet:
    limits: list[JointLimit]
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

    def __init__(self, limits: Optional[list[JointLimit]] = None):
        super().__init__()
        self.joint_limits = limits if limits != None else list()

    def to_dict(self):
        result = {}
        result["limits"] = from_union(
            [lambda x: from_list(lambda l: to_class(JointLimit, l), x), from_none],
            self.joint_limits,
        )
        result["extensions"] = from_union(
            [lambda x: from_dict(from_extension, x), from_none], self.extensions
        )
        result["extras"] = from_extra(self.extras)
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        result = JointLimitSet()
        result.joint_limits = from_union(
            [lambda x: from_list(JointLimit.from_dict, x), from_none], obj.get("limits")
        )
        result.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        result.extras = obj.get("extras")
        return result


class Collider:
    shape: Optional[Union[int, ChildOfRootExtension]] = None
    physics_material: Optional[Union[int, ChildOfRootExtension]] = None
    collision_filter: Optional[Union[int, ChildOfRootExtension]] = None
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

    def to_dict(self):
        result = {}
        result["shape"] = self.shape
        result["physicsMaterial"] = self.physics_material
        result["collisionFilter"] = self.collision_filter
        result["extensions"] = from_union(
            [lambda x: from_dict(from_extension, x), from_none], self.extensions
        )
        result["extras"] = from_extra(self.extras)
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        result = Collider()
        result.shape = from_union([from_int, from_none], obj.get("shape"))
        result.physics_material = from_union(
            [from_int, from_none], obj.get("physicsMaterial")
        )
        result.collision_filter = from_union(
            [from_int, from_none], obj.get("collisionFilter")
        )
        result.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        result.extras = obj.get("extras")
        return result


class Trigger:
    shape: Optional[Union[int, ChildOfRootExtension]] = None
    collision_filter: Optional[Union[int, ChildOfRootExtension]] = None
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

    def to_dict(self):
        result = {}
        result["shape"] = self.shape
        result["collisionFilter"] = self.collision_filter
        result["extensions"] = from_union(
            [lambda x: from_dict(from_extension, x), from_none], self.extensions
        )
        result["extras"] = from_extra(self.extras)
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        result = Trigger()
        result.shape = from_union([from_int, from_none], obj.get("shape"))
        result.collision_filter = from_union(
            [from_int, from_none], obj.get("collisionFilter")
        )
        result.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        result.extras = obj.get("extras")
        return result


class Joint:
    connected_node: Optional[Union[int, Node]] = None
    joint_limits: Optional[Union[int, ChildOfRootExtension]] = None
    enable_collision: Optional[bool] = None
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

    def __init__(self):
        super().__init__()

    def to_dict(self):
        result = {}
        result["connectedNode"] = self.connected_node
        result["jointLimits"] = self.joint_limits
        result["enableCollision"] = from_union(
            [from_bool, from_none], self.enable_collision
        )
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
        joint = Joint()
        joint.connected_node = from_union(
            [from_int, from_none], obj.get("connectedNode")
        )
        joint.joint_limits = from_union([from_int, from_none], obj.get("jointLimits"))
        joint.enable_collision = from_union(
            [from_bool, from_none], obj.get("enableCollision")
        )
        joint.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        joint.extras = obj.get("extras")
        return joint


class RigidBodiesNodeExtension:
    motion: Optional[Motion] = None
    collider: Optional[Collider] = None
    trigger: Optional[Trigger] = None
    joint: Optional[Joint] = None
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

    def __init__(self):
        super().__init__()

    def to_dict(self):
        result = {}
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
        result["extensions"] = from_union(
            [lambda x: from_dict(from_extension, x), from_none], self.extensions
        )
        result["extras"] = from_extra(self.extras)
        return result

    @staticmethod
    def from_dict(obj):
        assert isinstance(obj, dict)
        result = RigidBodiesNodeExtension()
        result.motion = from_union([Motion.from_dict, from_none], obj.get("motion"))
        result.collider = from_union(
            [Collider.from_dict, from_none], obj.get("collider")
        )
        result.trigger = from_union([Trigger.from_dict, from_none], obj.get("trigger"))
        result.joint = from_union([Joint.from_dict, from_none], obj.get("joint"))
        result.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        result.extras = obj.get("extras")
        return result


class RigidBodiesGlTFExtension:
    materials: list[Material] = []
    joints: list[JointLimitSet] = []
    collision_filters: list[CollisionFilter] = []
    extensions: Optional[Dict[str, Any]] = None
    extras: Any = None

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
        result["extensions"] = from_union(
            [lambda x: from_dict(from_extension, x), from_none], self.extensions
        )
        result["extras"] = from_extra(self.extras)
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
        result.extensions = from_union(
            [lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none],
            obj.get("extensions"),
        )
        result.extras = obj.get("extras")
        return result
