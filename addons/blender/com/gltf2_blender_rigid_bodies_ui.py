import bpy
import gpu
from gpu_extras.batch import batch_for_shader
from mathutils import Quaternion, Vector, Euler
import math

from ...io.com.gltf2_io_rigid_bodies import physics_material_combine_types


class KHRPhysicsSceneAdditionalSettings(bpy.types.PropertyGroup):
    draw_velocity: bpy.props.BoolProperty(name="Draw Velocities", default=False)
    draw_mass_props: bpy.props.BoolProperty(name="Draw Mass Properties", default=False)


class KHRPhysicsBodyAdditionalSettings(bpy.types.PropertyGroup):
    is_trigger: bpy.props.BoolProperty(name="Is Trigger", default=False)
    gravity_factor: bpy.props.FloatProperty(name="Gravity Factor", default=1.0)
    linear_velocity: bpy.props.FloatVectorProperty(
        name="Linear Velocity", default=(0, 0, 0)
    )
    angular_velocity: bpy.props.FloatVectorProperty(
        name="Angular Velocity", default=(0, 0, 0)
    )

    infinite_mass: bpy.props.BoolProperty(name="Infinite Mass", default=False)
    enable_inertia_override: bpy.props.BoolProperty(
        name="Override Inertia Tensor", default=False
    )
    inertia_major_axis: bpy.props.FloatVectorProperty(
        name="Inertia Major Axis", default=(1, 1, 1)
    )
    inertia_orientation: bpy.props.FloatVectorProperty(
        name="Inertia Orientation", subtype="EULER"
    )

    enable_com_override: bpy.props.BoolProperty(
        name="Override Center of Mass", default=False
    )
    center_of_mass: bpy.props.FloatVectorProperty(
        name="Center of Mass", default=(0, 0, 0)
    )

    friction_combine: bpy.props.EnumProperty(
        name="Friction Combine mode", items=physics_material_combine_types
    )
    restitution_combine: bpy.props.EnumProperty(
        name="Restitution Combine mode", items=physics_material_combine_types
    )

    # We want some extra properties to control capsule/cylinder/cone shapes.
    cone_capsule_override: bpy.props.BoolProperty(
        name="Override shape params", default=False
    )
    cone_capsule_radius_bottom: bpy.props.FloatProperty(
        name="Radius Bottom", default=1.0, min=0
    )
    cone_capsule_radius_top: bpy.props.FloatProperty(
        name="Radius Top", default=1.0, min=0
    )
    cone_capsule_height: bpy.props.FloatProperty(name="Height", default=1.0, min=0)


class KHRPhysicsExporterProperties(bpy.types.PropertyGroup):
    enabled: bpy.props.BoolProperty(
        name="KHR_rigid_bodies",
        description="Include rigid body data in the exported glTF file.",
        default=True,
    )


class KHRPhysicsImporterProperties(bpy.types.PropertyGroup):
    enabled: bpy.props.BoolProperty(
        name="KHR_rigid_bodies",
        description="Include rigid body data from the imported glTF file.",
        default=True,
    )


class KHRPhysicsSettingsViewportRenderHelper:
    def __init__(self):
        self.shader = gpu.shader.from_builtin("3D_UNIFORM_COLOR")

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

        if bpy.context.scene.khr_physics_scene_viewer_props.draw_velocity:
            self.draw_velocity(obj)

        if bpy.context.scene.khr_physics_scene_viewer_props.draw_mass_props:
            self.draw_mass_props(obj)

        if obj.rigid_body.collision_shape in ("CAPSULE", "CYLINDER", "CONE"):
            if obj.khr_physics_extra_props.cone_capsule_override:
                self.draw_custom_shape(obj)

    def draw_custom_shape(self, obj):
        ep = obj.khr_physics_extra_props
        outlinePoints = []
        if obj.rigid_body.collision_shape == "CAPSULE":
            numSegments = 40

            centers = [
                Vector((0, 0, -ep.cone_capsule_height * 0.5)),
                Vector((0, 0, ep.cone_capsule_height * 0.5)),
            ]
            cutoff = 0  # Remains zero in degenerate case
            maxR = max(ep.cone_capsule_radius_bottom, ep.cone_capsule_radius_top)
            minR = min(ep.cone_capsule_radius_bottom, ep.cone_capsule_radius_top)
            scaleZ = (
                1 if ep.cone_capsule_radius_bottom > ep.cone_capsule_radius_top else -1
            )

            if (
                ep.cone_capsule_radius_bottom != ep.cone_capsule_radius_top
                and ep.cone_capsule_height > 0
            ):
                r = maxR - minR
                px = r * r / ep.cone_capsule_height
                v = px * ep.cone_capsule_height - px * px
                if v > 0:
                    py = (px * ep.cone_capsule_height - px * px) ** 0.5
                    cutoff = math.atan(py / -px)
            elif ep.cone_capsule_radius_bottom == ep.cone_capsule_radius_top:
                cutoff = math.pi * -0.5

            for i in range(numSegments):
                a = 2 * i * math.pi / (numSegments - 1) - math.pi
                onCircle = Vector((math.sin(a), 0, math.cos(a)))
                if a < cutoff or a > -cutoff:
                    onCapsule = onCircle * maxR + centers[0]
                else:
                    onCapsule = onCircle * minR + centers[1]
                onCapsule.z *= scaleZ
                outlinePoints.append(onCapsule)
                outlinePoints.append(onCapsule)

            # Also rotate these points a quarter turn around the capsule axis
            # to match how capsules with a single radius are displayed
            outlinePoints.extend([Vector((0, q.x, q.z)) for q in outlinePoints])
            outlinePoints.insert(0, outlinePoints[0])

        if obj.rigid_body.collision_shape in ("CYLINDER", "CONE"):
            numAxialSegments = 15
            outlinePoints = [
                Vector(
                    (0, ep.cone_capsule_radius_bottom, -ep.cone_capsule_height * 0.5)
                ),
                Vector((0, ep.cone_capsule_radius_top, ep.cone_capsule_height * 0.5)),
            ]
            for i in range(1, numAxialSegments):
                a = 2 * i * math.pi / (numAxialSegments - 1)
                p = Vector((math.sin(a), math.cos(a), 0))
                pBottom = p * ep.cone_capsule_radius_bottom - Vector(
                    (0, 0, ep.cone_capsule_height * 0.5)
                )
                pTop = p * ep.cone_capsule_radius_top + Vector(
                    (0, 0, ep.cone_capsule_height * 0.5)
                )

                prevBottom = outlinePoints[-2]
                prevTop = outlinePoints[-1]
                outlinePoints.extend(
                    [prevBottom, pBottom, prevTop, pTop, pBottom, pTop]
                )

        # Now apply the object's transform (surely we don't have to bake this
        # into the points, and can just supply a uniform to the shader?)
        for i in range(len(outlinePoints)):
            outlinePoints[i] = obj.matrix_world @ outlinePoints[i]

        batch = batch_for_shader(self.shader, "LINES", {"pos": outlinePoints})
        self.shader.uniform_float("color", (0, 0, 0, 1))
        batch.draw(self.shader)

    def draw_velocity(self, obj):
        linVel = Vector(obj.khr_physics_extra_props.linear_velocity)
        angVel = Vector(obj.khr_physics_extra_props.angular_velocity)
        coords = [
            (obj.matrix_world @ Vector((0, 0, 0))).to_tuple(),
            (obj.matrix_world @ linVel).to_tuple(),
        ]
        batch = batch_for_shader(self.shader, "LINES", {"pos": coords})
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
        batch = batch_for_shader(self.shader, "LINES", {"pos": coords})
        self.shader.uniform_float("color", (1, 1, 0, 1))
        batch.draw(self.shader)

    def draw_mass_props(self, obj):
        if obj.khr_physics_extra_props.enable_com_override:
            com = Vector(obj.khr_physics_extra_props.center_of_mass)

            star = [
                Vector((-1, 0, 0)),
                Vector((1, 0, 0)),
                Vector((0, -1, 0)),
                Vector((0, 1, 0)),
                Vector((0, 0, -1)),
                Vector((0, 0, 1)),
            ]
            star = [obj.matrix_world @ com + p * 0.1 for p in star]
            batch = batch_for_shader(self.shader, "LINES", {"pos": star})
            self.shader.uniform_float("color", (1, 0, 1, 1))
            batch.draw(self.shader)
        else:
            com = Vector((0.0, 0.0, 0.0))

        unitBox = [
            Vector((-1, -1, -1)),
            Vector((-1, -1, 1)),
            Vector((-1, 1, -1)),
            Vector((-1, 1, 1)),
            Vector((1, -1, -1)),
            Vector((1, -1, 1)),
            Vector((1, 1, -1)),
            Vector((1, 1, 1)),
        ]
        if obj.khr_physics_extra_props.enable_inertia_override:
            itLocal = Vector(obj.khr_physics_extra_props.inertia_major_axis)
            itOrientation = Euler(
                obj.khr_physics_extra_props.inertia_orientation
            ).to_quaternion()
            itBox = [
                obj.matrix_world @ (com + itOrientation @ (v * itLocal))
                for v in unitBox
            ]
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

            batch = batch_for_shader(self.shader, "LINES", {"pos": itBox})
            self.shader.uniform_float("color", (1, 0, 1, 1))
            batch.draw(self.shader)


viewportRenderHelper = KHRPhysicsSettingsViewportRenderHelper()


class KHRPhysicsSettingsViewportPanel(bpy.types.Panel):
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "KHR Physics"
    bl_label = "KHR Physics"
    bl_idname = "OBJECT_PT_KHR_Physics_Viewport_Extensions"

    @classmethod
    def poll(cls, context):
        if context.object and context.object.rigid_body:
            return True
        return None

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.prop(context.scene.khr_physics_scene_viewer_props, "draw_velocity")
        row = layout.row()
        row.prop(context.scene.khr_physics_scene_viewer_props, "draw_mass_props")


class KHRPhysicsSettingsPanel(bpy.types.Panel):
    bl_label = "KHR Physics Extensions"
    bl_idname = "OBJECT_PT_KHR_Physics_Extensions"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "physics"

    @classmethod
    def poll(cls, context):
        if context.object and context.object.rigid_body:
            return True
        return None

    def draw(self, context):
        layout = self.layout

        obj = context.object

        # todo.eoin This feels a little different to Blender's usual UI.
        # Figure out how to add nice boxes/expanding headers/margins. (Seems to be nested Panels?)

        # Some extra shape parameterizations
        if context.object.rigid_body.collision_shape in ("CAPSULE", "CYLINDER", "CONE"):
            # It would be nice to have a "intitialize from exising mesh" button here
            row = layout.row()
            row.prop(obj.khr_physics_extra_props, "cone_capsule_override")
            row = layout.row()
            row.prop(obj.khr_physics_extra_props, "cone_capsule_radius_bottom")
            row.prop(obj.khr_physics_extra_props, "cone_capsule_height")
            row.prop(obj.khr_physics_extra_props, "cone_capsule_radius_top")

        row = layout.row()
        row.prop(obj.khr_physics_extra_props, "is_trigger")
        row = layout.row()
        row.prop(obj.khr_physics_extra_props, "gravity_factor")
        row = layout.row()
        row.prop(obj.khr_physics_extra_props, "linear_velocity")
        row = layout.row()
        row.prop(obj.khr_physics_extra_props, "angular_velocity")

        row = layout.row()
        row.prop(obj.khr_physics_extra_props, "infinite_mass")
        row = layout.row()
        row.prop(obj.khr_physics_extra_props, "enable_inertia_override")
        row = layout.row()
        row.enabled = obj.khr_physics_extra_props.enable_inertia_override
        row.prop(obj.khr_physics_extra_props, "inertia_major_axis")
        row = layout.row()
        row.enabled = obj.khr_physics_extra_props.enable_inertia_override
        row.prop(obj.khr_physics_extra_props, "inertia_orientation")

        row = layout.row()
        row.prop(obj.khr_physics_extra_props, "enable_com_override")
        row = layout.row()
        row.prop(obj.khr_physics_extra_props, "center_of_mass")
        row.enabled = obj.khr_physics_extra_props.enable_com_override

        row = layout.row()
        row.prop(obj.khr_physics_extra_props, "friction_combine")
        row = layout.row()
        row.prop(obj.khr_physics_extra_props, "restitution_combine")
