import bpy
import gpu
from io_scene_gltf2.blender.exp import gltf2_blender_export_keys
from io_scene_gltf2.io.com.gltf2_io import Node
from gpu_extras.batch import batch_for_shader
from mathutils import Matrix, Quaternion, Vector, Euler
import os, sys, math, traceback

from io_scene_gltf2.io.com.gltf2_io import from_dict, from_union, from_none, from_float
from io_scene_gltf2.io.com.gltf2_io import from_str, from_list, from_bool, from_int
from io_scene_gltf2.io.com.gltf2_io import to_float, to_class

bl_info = {
    'name': 'MSFT_Physics',
    'category': 'Import-Export',
    'version': (0, 0, 1),
    'blender': (3, 3, 0),
    'location': 'File > Export > glTF 2.0',
    'description': 'Extension for adding rigid body information to exported glTF file',
    'tracker_url': 'https://github.com/eoineoineoin/glTF_Physics_Blender_Exporter/issues',
    'isDraft': True,
    'developer': 'Eoin Mcloughlin (Havok)',
    'url': 'https://github.com/eoineoineoin/glTF_Physics_Blender_Exporter',
}

# glTF extensions are named following a convention with known prefixes.
# See: https://github.com/KhronosGroup/glTF/tree/master/extensions#about-gltf-extensions
# also: https://github.com/KhronosGroup/glTF/blob/master/extensions/Prefixes.md
collisionGeom_Extension_Name = 'MSFT_collision_primitives'
rigidBody_Extension_Name = 'MSFT_rigid_bodies'

# Support for an extension is "required" if a typical glTF viewer cannot be expected
# to load a given model without understanding the contents of the extension.
# For example, a compression scheme or new image format (with no fallback included)
# would be "required", but physics metadata or app-specific settings could be optional.
extension_is_required = False

# Constant used to construct some quaternions when switching up axis
halfSqrt2 = 2 ** 0.5 * 0.5

# Enum values for friction/restitution combine modes
physics_material_combine_types = [
    ('AVERAGE', 'Average', '', 0),
    ('MINIMUM', 'Minimum', '', 1),
    ('MAXIMUM', 'Maximum', '', 2),
    ('MULTIPLY', 'Multiply', '', 3)
]

class Collider:
    def __init__(self):
        self.collision_systems = None
        self.collide_with_systems = None
        self.not_collide_systems = None
        self.sphere = None
        self.box = None
        self.capsule = None
        self.cylinder = None
        self.convex = None
        self.trimesh = None
        self.extensions = None
        self.extras = None

    def to_dict(self):
        result = {}
        result["collisionSystems"] = from_union([lambda x: from_list(from_str, x), from_none], self.collision_systems)
        result["collideWithSystems"] = from_union([lambda x: from_list(from_str, x), from_none], self.collide_with_systems)
        result["notCollideWithSystems"] = from_union([lambda x: from_list(from_str, x), from_none], self.not_collide_systems)

        result["sphere"] = from_union([lambda x: to_class(Collider.Sphere, x), from_none], self.sphere)
        result["box"] = from_union([lambda x: to_class(Collider.Box, x), from_none], self.box)
        result["capsule"] = from_union([lambda x: to_class(Collider.Capsule, x), from_none], self.capsule)
        result["cylinder"] = from_union([lambda x: to_class(Collider.Cylinder, x), from_none], self.cylinder)
        result["convex"] = from_union([lambda x: to_class(Collider.Convex, x), from_none], self.convex)
        result["trimesh"] = from_union([lambda x: to_class(Collider.TriMesh, x), from_none], self.trimesh)

        result["extensions"] = from_union([lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none], self.extensions)
        result["extras"] = self.extras
        return result

    class Sphere:
        def __init__(self, radius = 0.5):
            self.radius = radius
            self.extensions = None
            self.extras = None
    class Box:
        def __init__(self, size = Vector((1.0, 1.0, 1.0))):
            self.size = size
            self.extensions = None
            self.extras = None
    class Capsule:
        def __init__(self, height = 0.5, radius = 0.25):
            self.height = height
            self.radius = radius
            self.extensions = None
            self.extras = None
    class Cylinder:
        def __init__(self, height = 0.5, radius = 0.25):
            self.height = height
            self.radius = radius
            self.extensions = None
            self.extras = None
    class Convex:
        def __init__(self, mesh):
            self.mesh = mesh
            self.extensions = None
            self.extras = None
        def to_dict(self):
            result = {}
            print(self.mesh)
            #result["mesh"] = from_int(self.mesh)
            result["mesh"] = self.mesh
            result["extensions"] = from_union([lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none], self.extensions)
            result["extras"] = self.extras
            return result
    class TriMesh:
        def __init__(self, mesh):
            self.mesh = mesh
            self.extensions = None
            self.extras = None
        def to_dict(self):
            result = {}
            result["mesh"] = from_int(self.mesh)
            result["extensions"] = from_union([lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none], self.extensions)
            result["extras"] = self.extras
            return result

class PhysicsMaterial:
    def __init__(self):
        self.static_friction = None
        self.dynamic_friction = None
        self.restitution = None
        self.friction_combine = None
        self.restitution_combine = None
        self.extensions = None
        self.extras = None

    def to_dict(self):
        result = {}
        result["staticFriction"] = from_union([from_float, from_none], self.static_friction)
        result["dynamicFriction"] = from_union([from_float, from_none], self.dynamic_friction)
        result["restitution"] = from_union([from_float, from_none], self.restitution)
        result["frictionCombine"] = from_union([from_str, from_none], self.friction_combine)
        result["restitutionCombine"] = from_union([from_str, from_none], self.restitution_combine)
        result["extensions"] = from_union([lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none], self.extensions)
        result["extras"] = self.extras
        return result

class RigidBody:
    def __init__(self):
        self.is_kinematic = None
        self.mass = None
        self.center_of_mass = None
        self.inertia_tensor = None
        self.linear_velocity = None
        self.angular_velocity = None
        self.gravity_factor = None
        self.extensions = None
        self.extras = None

    def to_dict(self):
        result = {}
        result["isKinematic"] = from_union([from_bool, from_none], self.is_kinematic)
        result["mass"] = from_union([from_float, from_none], self.mass)
        result["centerOfMass"] = from_union([lambda x: from_list(from_float, x), from_none], self.center_of_mass)
        result["inertiaTensor"] = from_union([lambda x: from_list(from_float, x), from_none], self.inertia_tensor)
        result["linearVelocity"] = from_union([lambda x: from_list(from_float, x), from_none], self.linear_velocity)
        result["angularVelocity"] = from_union([lambda x: from_list(from_float, x), from_none], self.angular_velocity)
        result["gravityFactor"] = from_union([from_float, from_none], self.gravity_factor)
        result["extensions"] = from_union([lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none], self.extensions)
        result["extras"] = self.extras

        return result

class JointLimit:
    def __init__():
        self.angular_axes = None
        self.linear_axes = None
        self.min_limit = None
        self.max_limit = None

    @staticmethod
    def Linear(axes):
        result = JointLimit()
        result.linear_axes = axes

    @staticmethod
    def Angular(axes):
        result = JointLimit
        result.angular_axes = axes

    def to_dict():
        return {}

class JointLimitSet:
    def __init__(self):
        self.limits = []

    def to_dict(self):
        pass

class Joint:
    def __init__():
        self.connected_node = None
        self.joint_limits = None
        self.enable_collision = None
        self.extensions = None
        self.extras = None

    def to_dict():
        result = {}
        result["connectedNode"] = from_union([from_int, from_none], self.connected_node)
        result["jointLimits"] = from_union([from_int, from_none], self.jointLimits)
        result["enableCollision"] = from_union([from_bool, from_none], self.enable_collision)
        result["extensions"] = from_union([lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none], self.extensions)
        result["extras"] = self.extras
        return result


class RigidBodiesNodeExtension:
    def __init__(self):
        self.rigid_body = None
        self.collider = None
        self.physics_material = None
        self.joint = None
        self.extensions = None
        self.extras = None

    def to_dict(self):
        result = {}
        result["rigidBody"] = from_union([lambda x: to_class(RigidBody, x), from_none], self.rigid_body)
        result["collider"] = from_union([from_int, from_none], self.collider)
        result["physicsMaterial"] = from_union([from_int, from_none], self.physics_material)
        result["joint"] = from_union([lambda x: to_class(Joint, x), from_none], self.joint)
        result["extensions"] = from_union([lambda x: from_dict(lambda x: from_dict(lambda x: x, x), x), from_none], self.extensions)
        result["extras"] = self.extras
        return result

class RigidBodiesGlTFExtension:
    def __init__(self):
        self.physics_materials = []
        self.physics_joint_limits = []

    def should_export(self):
        return len(self.physics_materials) > 0 or len(self.physics_joint_limits) > 0

    def to_dict(self):
        result = {}
        if len(self.physics_materials) > 0:
            result["physicsMaterials"] = from_list(lambda x: to_class(PhysicsMaterial, x), self.physics_materials)
        if len(self.physics_joint_limits) > 0:
            result["physicsJointLimits"] = from_list(lambda x: to_class(JointLimitSet, x), self.physics_joint_limits)
        return result



class MSFTPhysicsSceneAdditionalSettings(bpy.types.PropertyGroup):
    draw_velocity: bpy.props.BoolProperty(name='Draw Velocities', default=False)
    draw_mass_props: bpy.props.BoolProperty(name='Draw Mass Properties', default=False)

class MSFTPhysicsBodyAdditionalSettings(bpy.types.PropertyGroup):
    is_trigger: bpy.props.BoolProperty(name='Is Trigger', default=False)
    gravity_factor: bpy.props.FloatProperty(name='Gravity Factor', default=1.0)
    linear_velocity: bpy.props.FloatVectorProperty(name='Linear Velocity', default=(0,0,0))
    angular_velocity: bpy.props.FloatVectorProperty(name='Angular Velocity', default=(0,0,0))

    enable_inertia_override: bpy.props.BoolProperty(name='Override Inertia Tensor', default=False)
    inertia_major_axis: bpy.props.FloatVectorProperty(name='Inertia Major Axis', default=(1,1,1))
    inertia_orientation: bpy.props.FloatVectorProperty(name='Inertia Orientaiton', subtype='EULER')

    enable_com_override: bpy.props.BoolProperty(name='Override Center of Mass', default=False)
    center_of_mass: bpy.props.FloatVectorProperty(name='Center of Mass', default=(0,0,0))

    friction_combine: bpy.props.EnumProperty(name='Friction Combine mode', items=physics_material_combine_types)
    restitution_combine: bpy.props.EnumProperty(name='Restitution Combine mode', items=physics_material_combine_types)

class MSFTPhysicsExporterProperties(bpy.types.PropertyGroup):
    enabled: bpy.props.BoolProperty(
        name=bl_info['name'],
        description='Include rigid body data in the exported glTF file.',
        default=True)


class MSFTPhysicsSettingsViewportRenderHelper:
    def __init__(self):
        self.shader = gpu.shader.from_builtin('3D_UNIFORM_COLOR')

    def _calcPerpNormalized(self, v):
        v4 = Vector(v.to_tuple() + (0.0,))
        d0 = v4.yxww
        d1 = v4.zwxw
        if d0.length_squared < d1.length_squared:
            return d1.xyz.normalized()
        return d0.xyz.normalized()

    def drawExtraPhysicsProperties(self):
        if not bpy.context.object:
            return
        if not bpy.context.object.rigid_body:
            return

        obj = bpy.context.object

        if bpy.context.scene.msft_physics_scene_viewer_props.draw_velocity:
            self.draw_velocity(obj)

        if bpy.context.scene.msft_physics_scene_viewer_props.draw_mass_props:
            self.draw_mass_props(obj)

    def draw_velocity(self, obj):
        linVel = Vector(obj.msft_physics_extra_props.linear_velocity)
        angVel = Vector(obj.msft_physics_extra_props.angular_velocity)
        coords = [(obj.matrix_world @ Vector((0, 0, 0))).to_tuple(),
                  (obj.matrix_world @ linVel).to_tuple()]
        batch = batch_for_shader(self.shader, 'LINES', {"pos": coords})
        self.shader.uniform_float("color", (1, 1, 0, 1))
        batch.draw(self.shader)

        # Draw some samples of angular Velocity. This doesn't look great,
        # maybe a more intiutive way to display this.
        numAngularSamples = 20
        coords = [coords[0]]
        avPerp = self._calcPerpNormalized(angVel)
        avAxis = angVel.normalized()
        avMag = angVel.length * math.pi
        for i in range(numAngularSamples):
            t = float(i) / numAngularSamples
            avQ = Quaternion(avAxis, avMag * t)
            sampleLocal = avQ @ (avPerp * t) + linVel * t
            coords.append((obj.matrix_world @ sampleLocal))
            coords.append(coords[-1])
        batch = batch_for_shader(self.shader, 'LINES', {"pos": coords})
        self.shader.uniform_float("color", (1, 1, 0, 1))
        batch.draw(self.shader)

    def draw_mass_props(self, obj):
        if obj.msft_physics_extra_props.enable_com_override:
            com = Vector(obj.msft_physics_extra_props.center_of_mass)

            star = [Vector((-1,  0,  0)), Vector((1, 0, 0)),
                    Vector(( 0, -1,  0)), Vector((0, 1, 0)),
                    Vector(( 0,  0, -1)), Vector((0, 0, 1))]
            star = [obj.matrix_world @ com + p * 0.1 for p in star]
            batch = batch_for_shader(self.shader, 'LINES', {"pos": star})
            self.shader.uniform_float("color", (1, 0, 1, 1))
            batch.draw(self.shader)
        else:
            com = Vector((0.0, 0.0, 0.0))

        unitBox = [Vector((-1, -1, -1)), Vector((-1, -1,  1)),
                   Vector((-1,  1, -1)), Vector((-1,  1,  1)),
                   Vector(( 1, -1, -1)), Vector(( 1, -1,  1)),
                   Vector(( 1,  1, -1)), Vector(( 1,  1,  1))]
        if obj.msft_physics_extra_props.enable_inertia_override:
            itLocal = Vector(obj.msft_physics_extra_props.inertia_major_axis)
            itOrientation = Euler(obj.msft_physics_extra_props.inertia_orientation).to_quaternion()
            itBox = [obj.matrix_world @ (com + itOrientation @ (v * itLocal)) for v in unitBox]
            itBox.append(itBox[0])
            itBox.append(itBox[2])
            itBox.append(itBox[1])
            itBox.append(itBox[3])

            itBox.append(itBox[4])
            itBox.append(itBox[6])
            itBox.append(itBox[5])
            itBox.append(itBox[7])

            itBox.append(itBox[0])
            itBox.append(itBox[4])
            itBox.append(itBox[1])
            itBox.append(itBox[5])
            itBox.append(itBox[2])
            itBox.append(itBox[6])
            itBox.append(itBox[3])
            itBox.append(itBox[7])

            batch = batch_for_shader(self.shader, 'LINES', {"pos": itBox})
            self.shader.uniform_float("color", (1, 0, 1, 1))
            batch.draw(self.shader)

viewportRenderHelper = MSFTPhysicsSettingsViewportRenderHelper()

class MSFTPhysicsSettingsViewportPanel(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'MSFT Physics'
    bl_label = 'MSFT Physics'
    bl_idname = "OBJECT_PT_MSFT_Physics_Viewport_Extensions"

    @classmethod
    def poll(cls, context):
        if context.object and context.object.rigid_body:
            return True
        return None

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.prop(context.scene.msft_physics_scene_viewer_props, 'draw_velocity')
        row = layout.row()
        row.prop(context.scene.msft_physics_scene_viewer_props, 'draw_mass_props')


class MSFTPhysicsSettingsPanel(bpy.types.Panel):
    bl_label = 'MSFT Physics Extensions'
    bl_idname = "OBJECT_PT_MSFT_Physics_Extensions"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = 'physics'

    @classmethod
    def poll(cls, context):
        if context.object and context.object.rigid_body:
            return True
        return None

    def draw(self, context):
        layout = self.layout

        obj = context.object

        #todo.eoin This feels a little different to Blender's usual UI.
        # Figure out how to add nice boxes/expanding headers/margins. (Seems to be nested Panels?)
        row = layout.row()
        row.prop(obj.msft_physics_extra_props, 'is_trigger')
        row = layout.row()
        row.prop(obj.msft_physics_extra_props, 'gravity_factor')
        row = layout.row()
        row.prop(obj.msft_physics_extra_props, 'linear_velocity')
        row = layout.row()
        row.prop(obj.msft_physics_extra_props, 'angular_velocity')

        row = layout.row()
        row.prop(obj.msft_physics_extra_props, 'enable_inertia_override')
        row = layout.row()
        row.enabled = obj.msft_physics_extra_props.enable_inertia_override
        row.prop(obj.msft_physics_extra_props, 'inertia_major_axis')
        row = layout.row()
        row.enabled = obj.msft_physics_extra_props.enable_inertia_override
        row.prop(obj.msft_physics_extra_props, 'inertia_orientation')

        row = layout.row()
        row.prop(obj.msft_physics_extra_props, 'enable_com_override')
        row = layout.row()
        row.prop(obj.msft_physics_extra_props, 'center_of_mass')
        row.enabled = obj.msft_physics_extra_props.enable_com_override

        row = layout.row()
        row.prop(obj.msft_physics_extra_props, 'friction_combine')
        row = layout.row()
        row.prop(obj.msft_physics_extra_props, 'restitution_combine')

draw_handler = None #<todo.eoin Clean this up
def register():
    bpy.utils.register_class(MSFTPhysicsExporterProperties)
    bpy.utils.register_class(MSFTPhysicsSceneAdditionalSettings)
    bpy.utils.register_class(MSFTPhysicsBodyAdditionalSettings)
    bpy.utils.register_class(MSFTPhysicsSettingsViewportPanel)
    bpy.utils.register_class(MSFTPhysicsSettingsPanel)
    bpy.types.Scene.msft_physics_exporter_props = bpy.props.PointerProperty(type=MSFTPhysicsExporterProperties)
    bpy.types.Scene.msft_physics_scene_viewer_props = bpy.props.PointerProperty(type=MSFTPhysicsSceneAdditionalSettings)
    bpy.types.Object.msft_physics_extra_props = bpy.props.PointerProperty(type=MSFTPhysicsBodyAdditionalSettings)
    global draw_handler
    draw_handler = bpy.types.SpaceView3D.draw_handler_add(viewportRenderHelper.drawExtraPhysicsProperties, (), 'WINDOW', 'POST_VIEW')


def register_panel():
    # Register the panel on demand, we need to be sure to only register it once
    # This is necessary because the panel is a child of the extensions panel,
    # which may not be registered when we try to register this extension
    try:
        bpy.utils.register_class(GLTF_PT_UserExtensionPanel)
    except Exception:
        pass

    # If the glTF exporter is disabled, we need to unregister the extension panel
    # Just return a function to the exporter so it can unregister the panel
    return unregister_panel


def unregister_panel():
    # Since panel is registered on demand, it is possible it is not registered
    for p in (GLTF_PT_UserExtensionPanel, MSFTPhysicsSettingsPanel):
        try:
            bpy.utils.unregister_class(p)
        except Exception:
            pass

def unregister():
    unregister_panel()
    bpy.utils.unregister_class(MSFTPhysicsExporterProperties)
    bpy.utils.unregister_class(MSFTPhysicsSceneAdditionalSettings)
    bpy.utils.unregister_class(MSFTPhysicsBodyAdditionalSettings)
    bpy.utils.unregister_class(MSFTPhysicsSettingsViewportPanel)
    del bpy.types.Scene.msft_physics_exporter_props
    del bpy.types.Scene.msft_physics_scene_viewer_props
    del bpy.types.Object.msft_physics_extra_props

    global draw_handler
    bpy.types.SpaceView3D.draw_handler_remove(draw_handler, "WINDOW")
    draw_handler = None

class GLTF_PT_UserExtensionPanel(bpy.types.Panel):
    bl_space_type = 'FILE_BROWSER'
    bl_region_type = 'TOOL_PROPS'
    bl_label = 'Enabled'
    bl_parent_id = 'GLTF_PT_export_user_extensions'
    bl_options = set()

    @classmethod
    def poll(cls, context):
        sfile = context.space_data
        operator = sfile.active_operator
        return operator.bl_idname == 'EXPORT_SCENE_OT_gltf'

    def draw_header(self, context):
        props = bpy.context.scene.msft_physics_exporter_props
        self.layout.prop(props, 'enabled')

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = False
        layout.use_property_decorate = False  # No animation.

        props = bpy.context.scene.msft_physics_exporter_props
        layout.active = props.enabled

class glTF2ExportUserExtension:
    def __init__(self):
        # We need to wait until we create the gltf2UserExtension to import the gltf2 modules
        # Otherwise, it may fail because the gltf2 may not be loaded yet
        from io_scene_gltf2.io.com.gltf2_io_extensions import Extension
        self.Extension = Extension
        self.properties = bpy.context.scene.msft_physics_exporter_props
        self.gltfExt = RigidBodiesGlTFExtension()

        # Maps the gltf node to collider data. Since we don't know what other extensions
        # might produce collider data, we'll potentially need to re-index colliders
        # we've already generated
        self.physicsColliders = {}

        # Supporting data allowing us to save joints correctly
        self.blenderJointObjects = []
        self.blenderNodeToGltfNode = {}

    def gather_gltf_extensions_hook(self, gltf2_plan, export_settings):
        if not self.properties.enabled:
            return

        if gltf2_plan.extensions is None:
            gltf2_plan.extensions = {}

        if self.gltfExt.should_export():
            physicsRootExtension = self.Extension(
                name=rigidBody_Extension_Name,
                extension=self.gltfExt.to_dict(),
                required=extension_is_required)
            gltf2_plan.extensions[rigidBody_Extension_Name] = physicsRootExtension

        #
        # Export and re-index any colliders we generated.
        # We may have to generate the extension data for the collision primitives.
        #
        if not collisionGeom_Extension_Name in gltf2_plan.extensions:
            cgExtension = self.Extension(
                name = collisionGeom_Extension_Name,
                extension = {},
                required = extension_is_required)
            gltf2_plan.extensions[collisionGeom_Extension_Name] = cgExtension

        if not 'colliders' in cgExtension.extension and len(self.physicsColliders) > 0:
            cgExtension.extension['colliders'] = []

        for gltfNode in self.physicsColliders:
            # Fixup the collider ID
            #<todo.eoin Is there a better way to do this? Node.children seems to do this automagically somewhere?
            gltfNode.extensions[rigidBody_Extension_Name]['collider'] = len(cgExtension.extension['colliders'])
            cgExtension.extension['colliders'].append(self.physicsColliders[gltfNode])

    def gather_scene_hook(self, gltf2_scene, blender_scene, export_settings):
        if not self.properties.enabled:
            return

        #
        # Export any joints we've seen. These joints may need additional gltf nodes
        # created, in order to supply the pivot transform
        #
        for joint_node in self.blenderJointObjects:
            gltf2_object = self.blenderNodeToGltfNode[joint_node]
            jointData = self._generateJointData(joint_node, gltf2_object, export_settings)
            # Blender allows a joint to be specified at any point in the scene
            # tree, and the joint points to bodyA/bodyB while the glTF_Physics
            # spec expects that the joint is attached to a child node of bodyA
            # (determining the joint space in bodyA) and points to a child node
            # of bodyB (defining the joint in bodyB space). Make new nodes to
            # contain those transforms.

            bodyA = joint_node.rigid_body_constraint.object1
            aFromWorld = bodyA.matrix_world.copy() if bodyA else Matrix()
            aFromWorld.invert()
            bodyB = joint_node.rigid_body_constraint.object2
            bFromWorld = bodyB.matrix_world.copy() if bodyB else Matrix()
            bFromWorld.invert()

            worldFromJoint = joint_node.matrix_world.copy()
            jointFromBodyA = aFromWorld @ worldFromJoint
            jointFromBodyB = bFromWorld @ worldFromJoint

            # gltf_A/B are the nodes connected to the constraint
            # jointInA/B are the pivots in the space of their connected node
            gltf_B = self.blenderNodeToGltfNode[bodyB]

            jointInB = self._constructNode('jointSpaceB',
                    jointFromBodyB.to_translation(),
                     jointFromBodyB.to_quaternion(), export_settings)
            gltf_B.children.append(jointInB)
            jointData['connectedNode'] = jointInB

            gltf_A = self.blenderNodeToGltfNode[bodyA]
            jointInA = self._constructNode('jointSpaceA', jointFromBodyA.to_translation(),
                    jointFromBodyA.to_quaternion(), export_settings)
            jointInA.extensions[rigidBody_Extension_Name] = self.Extension(
                name=rigidBody_Extension_Name,
                extension={'joint': jointData},
                required=extension_is_required)
            gltf_A.children.append(jointInA)

    def gather_node_hook(self, gltf2_object, blender_object, export_settings):
        if self.properties.enabled:
            self.blenderNodeToGltfNode[blender_object] = gltf2_object

            if gltf2_object.extensions is None:
                #<todo.eoin Pretty sure this is never hit, due to export_user_extensions()
                gltf2_object.extensions = {}

            extension_data = RigidBodiesNodeExtension()
            # Blender has no way to specify a shape without a rigid body. Instead, a single shape is
            # specified by being a child of a body whose collider type is "Compound Parent"
            if blender_object.rigid_body and blender_object.rigid_body.enabled and not self._isPartOfCompound(blender_object):
                rb = blender_object.rigid_body
                extraProps = blender_object.msft_physics_extra_props

                rigid_body = RigidBody()

                if rb.kinematic:
                    rigid_body.is_kinematic = rb.kinematic
                rigid_body.mass = rb.mass

                if extraProps.gravity_factor != 1.0:
                    rigid_body.gravity_factor = extraProps.gravity_factor

                lv = self.__convert_swizzle_location(Vector(extraProps.linear_velocity), export_settings)
                if lv.length_squared != 0:
                    rigid_body.linear_velocity = lv
                av = self.__convert_swizzle_location(Vector(extraProps.angular_velocity), export_settings)
                if av.length_squared != 0:
                    rigid_body.angular_velocity = av

                if extraProps.enable_com_override:
                    rigid_body.center_of_mass = Vector(extraProps.center_of_mass)

                if extraProps.enable_inertia_override:
                    inertiaOrientation = extraProps.inertia_orientation.to_matrix()
                    diag = self.__convert_swizzle_location(extraProps.inertia_major_axis, export_settings)
                    inertiaOrientation.col[0] *= diag
                    inertiaOrientation.col[1] *= diag
                    inertiaOrientation.col[2] *= diag
                    rigid_body.inertia_tensor = [x for col in inertiaOrientation for x in col]

                extension_data.rigid_body = rigid_body

            if blender_object.rigid_body:
                collider_data = self._generateColliderData(blender_object, gltf2_object, export_settings)
                if collider_data:
                    extension_data.collider = self._addCollider(gltf2_object, collider_data)

                extraProps = blender_object.msft_physics_extra_props
                if not extraProps.is_trigger:
                    extension_data.physics_material = len(self.gltfExt.physics_materials)
                    # Should we attempt to de-duplicate identical materials? This feels a little
                    # bit wasteful, but materials are not shared in Blender, and other tooling
                    # may want to change the material for one collider without affecting others.
                    mat = PhysicsMaterial()
                    mat.static_friction = blender_object.rigid_body.friction
                    mat.dynamic_friction = blender_object.rigid_body.friction
                    mat.restitution = blender_object.rigid_body.restitution

                    if extraProps.friction_combine != physics_material_combine_types[0][0]:
                        mat.friction_combine = extraProps.friction_combine
                    if extraProps.restitution_combine != physics_material_combine_types[0][0]:
                        mat.restitution_combine = extraProps.restitution_combine

                    self.gltfExt.physics_materials.append(mat)

            if blender_object.rigid_body_constraint:
                # Because joints refer to another node in the scene, which may not be processed yet,
                # We'll just save all the joint objects we see and process them later.
                self.blenderJointObjects.append(blender_object)

            gltf2_object.extensions[rigidBody_Extension_Name] = self.Extension(
                name=rigidBody_Extension_Name,
                extension=extension_data.to_dict(),
                required=extension_is_required
            )

    def _isPartOfCompound(self, node):
        if node.parent == None or node.parent.rigid_body == None:
            return False
        return node.parent.rigid_body.collision_shape == 'COMPOUND'

    def _generateJointData(self, node, glNode, export_settings):
        """Converts the concrete joint data on `node` to a generic 6DOF representation"""
        joint = node.rigid_body_constraint
        jointData = Joint()
        if not joint.disable_collisions:
            joint.enable_collision = not joint.disable_collisions

        if export_settings[gltf2_blender_export_keys.YUP]:
            X, Y, Z = (0, 2, 1)
        else:
            X, Y, Z = (0, 1, 2)

        limitSet = JointLimitSet()
        if joint.type == 'FIXED':
            limitSet.joint_limits.append(JointLimit.Linear([X, Y, Z]))
            limitSet.joint_limits.append(JointLimit.Angular([X, Y, Z]))
        elif joint.type == 'POINT':
            limitSet.joint_limits.append(JointLimit.Linear([X, Y, Z]))
        elif joint.type == 'HINGE':
            limitSet.joint_limits.append(JointLimit.Linear([X, Y, Z]))

            # Blender always specifies hinge about Z
            limitSet.joint_limits.append({'angularAxes': [X, Y]})

            if joint.use_limit_ang_z:
                angLimit = JointLimit.Angular([Z])
                angLimit.min_limit = joint.limit_ang_z_lower
                angLimit.max_limit = joint.limit_ang_z_upper
                limitSet.joint_limits.append(angLimit)
        elif joint.type == 'SLIDER':
            limitSet.joint_limits.append(JointLimit.Angular([X, Y, Z]))

            # Blender always specifies slider limit along X
            limitSet.joint_limits.append(JointLimit.Linear([Y, Z]))

            if joint.use_limit_lin_x:
                linLimit = JointLimit.Linear([X])
                linLimit.min_limit = joint.limit_lin_x_lower
                linLimit.max_limit = joint.limit_lin_x_upper
                limitSet.joint_limits.append(linLimit)
        elif joint.type == 'PISTON':
            # Blender always specifies slider limit along/around X
            limitSet.joint_limits.append(JointLimit.Angular([Y, Z]))
            limitSet.joint_limits.append(JointLimit.Linear([Y, Z]))

            if joint.use_limit_lin_x:
                linLimit = JointLimit.Linear([X])
                linLimit.min_limit = joint.limit_lin_x_lower
                linLimit.max_limit = joint.limit_lin_x_upper
                limitSet.joint_limits.append(linLimit)
            if joint.use_limit_ang_x:
                angLimit = JointLimit.Angular([X])
                angLimit.min_limit = joint.limit_ang_x_lower
                angLimit.max_limit = joint.limit_ang_x_upper
                limitSet.joint_limits.append(angLimit)
        elif joint.type == 'GENERIC':
            # Appears that Blender always uses 1D constraints
            if joint.use_limit_lin_x:
                linLimit = JointLimit.Linear([X])
                if joint.limit_lin_x_lower != 0 or joint.limit_lin_x_upper != 0:
                    linLimit.min_limit = joint.limit_lin_x_lower
                    linLimit.max_limit = joint.limit_lin_x_upper
                limitSet.joint_limits.append(linLimit)
            if joint.use_limit_lin_y:
                linLimit = JointLimit.Linear([Y])
                if joint.limit_lin_y_lower != 0 or joint.limit_lin_y_upper != 0:
                    if export_settings[gltf2_blender_export_keys.YUP]:
                        linLimit.min_limit = -joint.limit_lin_y_upper
                        linLimit.max_limit = -joint.limit_lin_y_lower
                    else:
                        linLimit.min_limit = joint.limit_lin_y_lower
                        linLimit.max_limit = joint.limit_lin_y_upper
                limitSet.joint_limits.append(linLimit)
            if joint.use_limit_lin_z:
                linLimit = JointLimit.Linear([Z])
                if joint.limit_lin_z_lower != 0 or joint.limit_lin_z_upper != 0:
                    linLimit.min_limit = joint.limit_lin_z_lower
                    linLimit.max_limit = joint.limit_lin_z_upper
                limitSet.joint_limits.append(linLimit)

            if joint.use_limit_ang_x:
                angLimit = JointLimit.Angular([X])
                if joint.limit_ang_x_lower != 0 or joint.limit_ang_x_upper != 0:
                    angLimit.min_limit = joint.limit_ang_x_lower
                    angLimit.max_limit = joint.limit_ang_x_upper
                limitSet.joint_limits.append(angLimit)
            if joint.use_limit_ang_y:
                angLimit = JointLimit.Angular([Y])
                if joint.limit_ang_y_lower != 0 or joint.limit_ang_y_upper != 0:
                    if export_settings[gltf2_blender_export_keys.YUP]:
                        angLimit.min_limit = -joint.limit_ang_y_upper
                        angLimit.max_limit = -joint.limit_ang_y_lower
                    else:
                        angLimit.min_limit = joint.limit_ang_y_lower
                        angLimit.max_limit = joint.limit_ang_y_upper
                limitSet.joint_limits.append(angLimit)
            if joint.use_limit_ang_z:
                angLimit = JointLimit.Angular([Z])
                if joint.limit_ang_z_lower != 0 or joint.limit_ang_z_upper != 0:
                    angLimit.min_limit = joint.limit_ang_z_lower
                    angLimit.max_limit = joint.limit_ang_z_upper
                limitSet.joint_limits.append(angLimit)

        self.gltfExt.physics_joint_limits.append(limitSet)
        jointData.joint_limits = len(self.gltfExt.physics_joint_limits) - 1
        return jointData

    def _generateColliderData(self, node, glNode, export_settings):
        if node.rigid_body == None or node.rigid_body.collision_shape == 'COMPOUND':
            return None
        colliderData = {}
        #collider = Collider()

        # Blender's native collision filtering has less functionality than the spec enables:
        #    * Children of COMPOUND_PARENT don't have a UI to configure filtering
        #    * An objects' "membership" is always equal to it's "collides with"
        #    * Seems there's no "user friendly" names
        collisionSystems = ["System_%i" % i for (i,enabled) in enumerate(node.rigid_body.collision_collections) if enabled]
        colliderData['collisionSystems'] = collisionSystems
        #collider.collision_systems = collisionSystems
        colliderData['collideWithSystems'] = collisionSystems
        #collider.collide_with_systems = collisionSystems

        if (node.rigid_body.collision_shape == 'CONE'
                or node.rigid_body.collision_shape == 'CONVEX_HULL'):
            colliderData['convex'] = {'mesh': glNode.mesh}
            #collider.convex = Collider.Convex(glNode.mesh)
        elif node.rigid_body.collision_shape == 'MESH':
            colliderData['trimesh'] = {'mesh': glNode.mesh}
            #collider.trimesh = Collider.Mesh(glNode.mesh)
        else:
            # If the shape is a geometric primitive, we may have to apply modifiers
            # to see the final geometry. (glNode has already had modifiers applied)
            with self._accessMeshData(node, export_settings) as meshData:
                if node.rigid_body.collision_shape == 'SPHERE':
                    maxRR = 0
                    for v in meshData.vertices:
                        maxRR = max(maxRR, v.co.length_squared)
                    colliderData['sphere'] = {'radius': maxRR ** 0.5}
                    #collider.sphere = Collider.Sphere(radius = maxRR ** 0.5)
                elif node.rigid_body.collision_shape == 'BOX':
                    maxHalfExtent = [0,0,0]
                    for v in meshData.vertices:
                        maxHalfExtent = [max(a,abs(b)) for a,b in zip(maxHalfExtent, v.co)]
                    colliderData['box'] = {'size': [he * 2 for he in self.__convert_swizzle_scale(maxHalfExtent, export_settings)]}
                    #collider.box = Collider.Box(size = self.__convert_swizzle_scale(maxHalfExtent, export_settings) * 2)
                #<TODO.eoin.Blender Cone shape feels underspecified? We need to do a proper calculation here
                elif (node.rigid_body.collision_shape == 'CAPSULE' or
                        node.rigid_body.collision_shape == 'CYLINDER'):
                    capsuleAxis = Vector((0,0,1)) # Use blender's up axis, instead of glTF (and transform later)
                    maxHeight = 0
                    maxRadiusSquared = 0
                    for v in meshData.vertices:
                        maxHeight = max(maxHeight, abs(v.co.dot(capsuleAxis)))
                        radiusSquared = (v.co - capsuleAxis * v.co.dot(capsuleAxis)).length_squared
                        maxRadiusSquared = max(maxRadiusSquared, radiusSquared)
                    height = maxHeight * 2
                    radius = maxRadiusSquared ** 0.5
                    if node.rigid_body.collision_shape == 'CAPSULE':
                        # (height, radius) describe a *cylinder* which totally encloses the mesh geometry
                        # However, the Blender physics tools and visualization generate a capsule no larger than the mesh
                        # This is not what I'd expect, as it seems inconsistent with other shapes (e.g. a convex hull
                        # encloses the mesh) but for the sake of consistency with the Blender UI, we'll match their
                        # behaviour.
                        height = max(0, height - radius * 2)
                        colliderData['capsule'] = {'height': height, 'radius': radius}
                        #collider.capsule = Collider.Capsule(height = height, radius = radius)
                    else:
                        colliderData['cylinder'] = {'height': height, 'radius': radius}
                        #collider.cylinder = Collider.Cylinder(height = height, radius = radius)

                    if not export_settings[gltf2_blender_export_keys.YUP]:
                        # Add an additional node to align the object, so the shape is oriented correctly when constructed along +Y
                        collider_alignment = self._constructNode('physicsAlignmentNode',
                                Vector((0,0,0)), Quaternion((halfSqrt2, 0, 0, halfSqrt2)), export_settings);
                        colliderAlignment.extensions[rigidBody_Extension_Name] = self.Extension(
                            name=rigidBody_Extension_Name,
                            extension={'collider': self._addCollider(colliderAlignment, colliderData)},
                            #extension={'collider': self._addCollider(colliderAlignment, collider)},
                            required=extension_is_required)
                        glNode.children.append(colliderAlignment)
                        # We've added the collider data to a child of glNode;
                        # return None so that the glNode doesn't get collider data,
                        return None
        return colliderData
        #return collider

    def _addCollider(self, gltfNode, collider):
        self.physicsColliders[gltfNode] = collider
        return len(self.physicsColliders) - 1

    def _accessMeshData(self, node, export_settings):
        """RAII-style function to access mesh data with modifiers attached"""
        class ScopedMesh:
            def __init__(self, node, export_settings):
                self.node = node
                self.export_settings = export_settings
                self.modifiedNode = None

            def __enter__(self):
                if self.export_settings[gltf2_blender_export_keys.APPLY]:
                    depsGraph = bpy.context.evaluated_depsgraph_get()
                    self.modifiedNode = node.evaluated_get(depsGraph)
                    return self.modifiedNode.to_mesh(preserve_all_data_layers=True, depsgraph=depsGraph)
                else:
                    return self.node.data

            def __exit__(self, *args):
                if self.modifiedNode:
                    self.modifiedNode.to_mesh_clear()
        return ScopedMesh(node, export_settings)

    def _constructNode(self, name, translation, rotation, export_settings):
        return Node(name = name,
                translation = [x for x in self.__convert_swizzle_location(translation, export_settings)],
                rotation = self._serializeQuaternion(self.__convert_swizzle_rotation(rotation, export_settings)),
                matrix = [], camera = None, children = [], extensions = {}, extras = None, mesh = None,
                scale = None, skin = None, weights = None)

    # Copy-pasted from the glTF exporter; are they accessible some other way, without having to duplicate?
    def __convert_swizzle_location(self, loc, export_settings):
        """Convert a location from Blender coordinate system to glTF coordinate system."""
        if export_settings[gltf2_blender_export_keys.YUP]:
            return Vector((loc[0], loc[2], -loc[1]))
        else:
            return Vector((loc[0], loc[1], loc[2]))

    # Copy-pasted from the glTF exporter; are they accessible some other way, without having to duplicate?
    def __convert_swizzle_scale(self, scale, export_settings):
        """Convert a scale from Blender coordinate system to glTF coordinate system."""
        if export_settings[gltf2_blender_export_keys.YUP]:
            return Vector((scale[0], scale[2], scale[1]))
        else:
            return Vector((scale[0], scale[1], scale[2]))

    # Copy-pasted from the glTF exporter; are they accessible some other way, without having to duplicate?
    def __convert_swizzle_rotation(self, rot, export_settings):
        """
        Convert a quaternion rotation from Blender coordinate system to glTF coordinate system.
        'w' is still at first position.
        """
        if export_settings[gltf2_blender_export_keys.YUP]:
            return Quaternion((rot[0], rot[1], rot[3], -rot[2]))
        else:
            return Quaternion((rot[0], rot[1], rot[2], rot[3]))

    def _serializeQuaternion(self, q):
        """Converts a quaternion to a type which can be serialized, with components in correct order"""
        return [q.x, q.y, q.z, q.w]
