import bpy
import gpu
from gpu_extras.batch import batch_for_shader
from mathutils import Quaternion, Vector, Euler
import math

from ...io.com.gltf2_io_rigid_bodies import (
    physics_material_combine_types,
    physics_drive_mode_types,
)


class KHR_rigid_body_node_properties(bpy.types.PropertyGroup):
    # non_renderable is a bit of a hack. Hopefully temporary! Issue is that in Blender,
    # the collision shape is generated from the mesh associated with a node. Normally,
    # users would then set the "hide_render" on this object to make it invisible.
    # However, the glTF export doesn't exlude these objects unless the "use_visible" option
    # is set on export. This option has slightly different semantics, though - use of
    # this option will exclude the whole node from export - we won't even see it in our
    # plugin, so we won't be able to generate extension data. Setting this non_renderable
    # flag is a workaround for this behaviour and will cause the renderable mesh to be
    # removed from this node when exporting.
    non_renderable: bpy.props.BoolProperty(
        name="Non-Renderable",
        default=False,
        description="Exclude visibile mesh during export",
    )
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


class KHR_rigid_body_constraint_node_properties(bpy.types.PropertyGroup):
    use_ang_drive_x: bpy.props.BoolProperty(name="X Axis Drive", default=False)
    ang_x_drive_mode: bpy.props.EnumProperty(
        name="Force mode", items=physics_drive_mode_types
    )
    ang_x_drive_pos_target: bpy.props.FloatProperty(name="Position Target", default=0)
    ang_x_drive_vel_target: bpy.props.FloatProperty(name="Velocity Target", default=0)
    ang_x_drive_force_limited: bpy.props.BoolProperty(
        name="Force limited", default=False
    )
    ang_x_drive_max_force: bpy.props.FloatProperty(name="Max force", default=0)
    ang_x_drive_stiffness: bpy.props.FloatProperty(name="Stiffness", default=0)
    ang_x_drive_damping: bpy.props.FloatProperty(name="Damping", default=0)

    use_ang_drive_y: bpy.props.BoolProperty(name="Y Axis Drive", default=False)
    ang_y_drive_mode: bpy.props.EnumProperty(
        name="Force mode", items=physics_drive_mode_types
    )
    ang_y_drive_pos_target: bpy.props.FloatProperty(name="Position Target", default=0)
    ang_y_drive_vel_target: bpy.props.FloatProperty(name="Velocity Target", default=0)
    ang_y_drive_force_limited: bpy.props.BoolProperty(
        name="Force limited", default=False
    )
    ang_y_drive_max_force: bpy.props.FloatProperty(name="Max force", default=0)
    ang_y_drive_stiffness: bpy.props.FloatProperty(name="Stiffness", default=0)
    ang_y_drive_damping: bpy.props.FloatProperty(name="Damping", default=0)

    use_ang_drive_z: bpy.props.BoolProperty(name="Z Axis Drive", default=False)
    ang_z_drive_mode: bpy.props.EnumProperty(
        name="Force mode", items=physics_drive_mode_types
    )
    ang_z_drive_pos_target: bpy.props.FloatProperty(name="Position Target", default=0)
    ang_z_drive_vel_target: bpy.props.FloatProperty(name="Velocity Target", default=0)
    ang_z_drive_force_limited: bpy.props.BoolProperty(
        name="Force limited", default=False
    )
    ang_z_drive_max_force: bpy.props.FloatProperty(name="Max force", default=0)
    ang_z_drive_stiffness: bpy.props.FloatProperty(name="Stiffness", default=0)
    ang_z_drive_damping: bpy.props.FloatProperty(name="Damping", default=0)

    use_lin_drive_x: bpy.props.BoolProperty(name="X Axis Drive", default=False)
    lin_x_drive_mode: bpy.props.EnumProperty(
        name="Force mode", items=physics_drive_mode_types
    )
    lin_x_drive_pos_target: bpy.props.FloatProperty(name="Position Target", default=0)
    lin_x_drive_vel_target: bpy.props.FloatProperty(name="Velocity Target", default=0)
    lin_x_drive_force_limited: bpy.props.BoolProperty(
        name="Force limited", default=False
    )
    lin_x_drive_max_force: bpy.props.FloatProperty(name="Max force", default=0)
    lin_x_drive_stiffness: bpy.props.FloatProperty(name="Stiffness", default=0)
    lin_x_drive_damping: bpy.props.FloatProperty(name="Damping", default=0)

    use_lin_drive_y: bpy.props.BoolProperty(name="Y Axis Drive", default=False)
    lin_y_drive_mode: bpy.props.EnumProperty(
        name="Force mode", items=physics_drive_mode_types
    )
    lin_y_drive_pos_target: bpy.props.FloatProperty(name="Position Target", default=0)
    lin_y_drive_vel_target: bpy.props.FloatProperty(name="Velocity Target", default=0)
    lin_y_drive_force_limited: bpy.props.BoolProperty(
        name="Force limited", default=False
    )
    lin_y_drive_max_force: bpy.props.FloatProperty(name="Max force", default=0)
    lin_y_drive_stiffness: bpy.props.FloatProperty(name="Stiffness", default=0)
    lin_y_drive_damping: bpy.props.FloatProperty(name="Damping", default=0)

    use_lin_drive_z: bpy.props.BoolProperty(name="Z Axis Drive", default=False)
    lin_z_drive_mode: bpy.props.EnumProperty(
        name="Force mode", items=physics_drive_mode_types
    )
    lin_z_drive_pos_target: bpy.props.FloatProperty(name="Position Target", default=0)
    lin_z_drive_vel_target: bpy.props.FloatProperty(name="Velocity Target", default=0)
    lin_z_drive_force_limited: bpy.props.BoolProperty(
        name="Force limited", default=False
    )
    lin_z_drive_max_force: bpy.props.FloatProperty(name="Max force", default=0)
    lin_z_drive_stiffness: bpy.props.FloatProperty(name="Stiffness", default=0)
    lin_z_drive_damping: bpy.props.FloatProperty(name="Damping", default=0)


class KHR_rigid_body_exporter_properties(bpy.types.PropertyGroup):
    enabled: bpy.props.BoolProperty(
        name="KHR_physics_rigid_bodies",
        description="Include rigid body data in the exported glTF file.",
        default=True,
    )

    reparent_bones: bpy.props.BoolProperty(
        name="Reparent RB Bones",
        description="Reparent constrained bones to children of rigid bodies",
        default=True,
    )


class KHR_rigid_body_importer_properties(bpy.types.PropertyGroup):
    enabled: bpy.props.BoolProperty(
        name="KHR_physics_rigid_bodies",
        description="Include rigid body data from the imported glTF file.",
        default=True,
    )


class KHR_rigid_body_viewport_render:
    def __init__(self, draw_velocity, draw_mass_props):
        self.should_draw_velocity = draw_velocity
        self.should_draw_mass_props = draw_mass_props
        if not bpy.app.background:
            shaderType = (
                "3D_UNIFORM_COLOR" if bpy.app.version[0] < 4 else "UNIFORM_COLOR"
            )
            self.shader = gpu.shader.from_builtin(shaderType)
        else:
            self.shader = None

    def _calcPerpNormalized(self, v):
        v4 = Vector(v.to_tuple() + (0.0,))
        d0 = v4.yxww
        d1 = v4.zwxw
        if d0.length_squared < d1.length_squared:
            return d1.xyz.normalized()
        return d0.xyz.normalized()

    def drawExtraPhysicsProperties(self):
        if not self.shader:
            return
        if not bpy.context.object:
            return
        if not bpy.context.object.rigid_body:
            return

        obj = bpy.context.object

        if self.should_draw_velocity:
            self.draw_velocity(obj)

        if self.should_draw_mass_props:
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


class _modalDrawOperator(bpy.types.Operator):
    def modal(self, context, event):
        context.area.tag_redraw()

        if event.type == "LEFTMOUSE":
            bpy.types.SpaceView3D.draw_handler_remove(self._handle, "WINDOW")
            return {"FINISHED"}
        elif event.type in {"RIGHTMOUSE", "ESC"}:
            bpy.types.SpaceView3D.draw_handler_remove(self._handle, "WINDOW")
            return {"CANCELLED"}

        return {"RUNNING_MODAL"}

    def invoke(self, context, event):
        if context.area.type == "VIEW_3D":
            handlerFunc = self.getRenderHandler()
            self._handle = bpy.types.SpaceView3D.draw_handler_add(
                handlerFunc, (), "WINDOW", "POST_VIEW"
            )

            context.window_manager.modal_handler_add(self)
            return {"RUNNING_MODAL"}
        else:
            self.report({"WARNING"}, "View3D not found, cannot run operator")
            return {"CANCELLED"}


class KHR_OT_DrawVelocityOperator(_modalDrawOperator):
    """Draws velocity info in viewport"""

    bl_idname = "view3d.khr_draw_velocity"
    bl_label = "Display Rigid Body Physics Velocity"

    def getRenderHandler(self):
        self._handler = KHR_rigid_body_viewport_render(
            draw_velocity=True, draw_mass_props=False
        )
        return self._handler.drawExtraPhysicsProperties


class KHR_OT_DrawMassPropsOperator(_modalDrawOperator):
    """Draws mass properties in viewport"""

    bl_idname = "view3d.khr_draw_mass_props"
    bl_label = "Display Rigid Body Physics Mass Properties"

    def getRenderHandler(self):
        self._handler = KHR_rigid_body_viewport_render(
            draw_velocity=False, draw_mass_props=True
        )
        return self._handler.drawExtraPhysicsProperties


class KHR_PT_rigid_body_visualizer(bpy.types.Panel):
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
        row.operator(KHR_DrawVelocityOperator.bl_idname)
        row = layout.row()
        row.operator(KHR_DrawMassPropsOperator.bl_idname)


class KHR_PT_rigid_body_panel_base(bpy.types.Panel):
    bl_label = "KHR Physics Extensions"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "physics"

    @classmethod
    def rigid_body_selected(cls, context):
        if context.object and context.object.rigid_body:
            return True
        return None


class KHR_PT_rigid_body_panel(KHR_PT_rigid_body_panel_base):
    bl_idname = "KHR_PT_rigid_body_panel"

    @classmethod
    def poll(cls, context):
        return KHR_PT_rigid_body_panel_base.rigid_body_selected(context)

    def draw(self, context):
        pass


class KHR_PT_rigid_body_motion(KHR_PT_rigid_body_panel_base):
    bl_label = "Motion"
    bl_parent_id = "KHR_PT_rigid_body_panel"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "physics"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return (
            KHR_PT_rigid_body_panel_base.rigid_body_selected(context)
            and context.object.rigid_body.enabled
        )  # "Dynamic" checked

    def draw(self, context):
        obj = context.object
        layout = self.layout
        layout.use_property_split = True
        flow = layout.grid_flow(
            row_major=True, columns=0, even_columns=True, even_rows=False, align=True
        )

        col = flow.column()
        col.prop(obj.khr_physics_extra_props, "linear_velocity")

        col = flow.column()
        col.prop(obj.khr_physics_extra_props, "angular_velocity")

        col = flow.column()
        col.prop(obj.khr_physics_extra_props, "gravity_factor")


class KHR_PT_rigid_body_mass(KHR_PT_rigid_body_panel_base):
    bl_label = "Mass properties"
    bl_parent_id = "KHR_PT_rigid_body_panel"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "physics"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return (
            KHR_PT_rigid_body_panel_base.rigid_body_selected(context)
            and context.object.rigid_body.enabled
        )  # "Dynamic" checked

    def draw(self, context):
        obj = context.object
        layout = self.layout
        layout.use_property_split = True
        flow = layout.grid_flow(
            row_major=True, columns=0, even_columns=True, even_rows=False, align=True
        )

        col = flow.column()
        col.prop(obj.khr_physics_extra_props, "infinite_mass")

        col = flow.column()
        col.prop(obj.khr_physics_extra_props, "enable_inertia_override")

        col = flow.column()
        col.enabled = obj.khr_physics_extra_props.enable_inertia_override
        col.prop(obj.khr_physics_extra_props, "inertia_major_axis")

        col = flow.column()
        col.enabled = obj.khr_physics_extra_props.enable_inertia_override
        col.prop(obj.khr_physics_extra_props, "inertia_orientation")

        col = flow.column()
        col.prop(obj.khr_physics_extra_props, "enable_com_override")

        col = flow.column()
        col.enabled = obj.khr_physics_extra_props.enable_com_override
        col.prop(obj.khr_physics_extra_props, "center_of_mass")


class KHR_PT_rigid_body_shape(KHR_PT_rigid_body_panel_base):
    bl_label = "Collisions"
    bl_parent_id = "KHR_PT_rigid_body_panel"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "physics"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return KHR_PT_rigid_body_panel_base.rigid_body_selected(context)

    def draw(self, context):
        obj = context.object
        layout = self.layout
        layout.use_property_split = True
        flow = layout.grid_flow(
            row_major=True, columns=0, even_columns=True, even_rows=False, align=True
        )

        col = flow.column()
        col.prop(obj.khr_physics_extra_props, "is_trigger")

        col = flow.column()
        col.prop(obj.rigid_body, "friction")
        col.active = not obj.khr_physics_extra_props.is_trigger

        col = flow.column()
        col.active = not obj.khr_physics_extra_props.is_trigger
        col.prop(obj.rigid_body, "restitution", text="Bounciness")

        col = flow.column()
        col.active = not obj.khr_physics_extra_props.is_trigger
        col.prop(obj.khr_physics_extra_props, "friction_combine")

        col = flow.column()
        col.active = not obj.khr_physics_extra_props.is_trigger
        col.prop(obj.khr_physics_extra_props, "restitution_combine")

        # Some extra shape parameterizations
        if context.object.rigid_body.collision_shape in ("CAPSULE", "CYLINDER", "CONE"):
            row = flow.column()
            row.prop(obj.khr_physics_extra_props, "cone_capsule_override")
            row = flow.column()
            row.active = obj.khr_physics_extra_props.cone_capsule_override
            row.operator(
                "khr_physics_rigid_bodies.calculate_cone_capsule_params",
                text="Calculate from mesh",
            )
            row.prop(obj.khr_physics_extra_props, "cone_capsule_radius_bottom")
            row.prop(obj.khr_physics_extra_props, "cone_capsule_height")
            row.prop(obj.khr_physics_extra_props, "cone_capsule_radius_top")


class KHR_PT_rigid_body_collections(KHR_PT_rigid_body_panel_base):
    """Additional panel to display collision collections for a body, as, by default,
    Blender will not show collections for children of a COMPOUND_PARENT"""

    bl_label = "Collections"
    bl_parent_id = "KHR_PT_rigid_body_panel"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "physics"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return KHR_PT_rigid_body_panel_base.rigid_body_selected(context)

    def draw(self, context):
        obj = context.object
        layout = self.layout
        layout.prop(obj.rigid_body, "collision_collections")


class KHR_PT_rigid_body_other(KHR_PT_rigid_body_panel_base):
    """Panel to display properties for a rigid body which do not fit into other categories"""

    bl_label = "Other"
    bl_parent_id = "KHR_PT_rigid_body_panel"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "physics"

    @classmethod
    def poll(cls, context):
        return KHR_PT_rigid_body_panel_base.rigid_body_selected(context)

    def draw(self, context):
        obj = context.object
        layout = self.layout
        layout.use_property_split = True
        flow = layout.grid_flow(
            row_major=True, columns=0, even_columns=True, even_rows=False, align=True
        )

        col = flow.column()
        col.prop(obj.khr_physics_extra_props, "non_renderable")


class KHR_PT_rigid_body_constraint_panel_base(bpy.types.Panel):
    bl_label = "KHR Physics Constraint Extensions"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "physics"

    @classmethod
    def rigid_body_constraint_selected(cls, context):
        if context.object and context.object.rigid_body:
            return True
        return None


class KHR_PT_rigid_body_constraint_panel(KHR_PT_rigid_body_constraint_panel_base):
    @classmethod
    def poll(cls, context):
        return context.object and context.object.rigid_body_constraint

    def draw(self, context):
        pass


class KHR_PT_rigid_body_constraint_drives(KHR_PT_rigid_body_constraint_panel_base):
    bl_label = "Drives"
    bl_parent_id = "KHR_PT_rigid_body_constraint_panel"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return KHR_PT_rigid_body_constraint_panel.poll(context)

    def draw(self, context):
        pass

    @classmethod
    def _generateDriveSettings(
        cls,
        uilayout: bpy.types.UILayout,
        joint: KHR_rigid_body_constraint_node_properties,
        linear: bool,
    ):
        typeprefix = "lin" if linear else "ang"

        for ax in ["x", "y", "z"]:
            col = uilayout.column()
            drive_field_name = "use_%s_drive_%s" % (typeprefix, ax)
            forcelimited_field_name = "%s_%s_drive_force_limited" % (typeprefix, ax)
            col.prop(joint, drive_field_name)

            sub = col.column(align=True)
            sub.active = getattr(joint, drive_field_name)
            sub.prop(joint, "%s_%s_drive_mode" % (typeprefix, ax))
            sub.prop(joint, "%s_%s_drive_pos_target" % (typeprefix, ax))
            sub.prop(joint, "%s_%s_drive_vel_target" % (typeprefix, ax))
            sub.prop(joint, forcelimited_field_name)
            subsub = sub.column(align=True)
            subsub.active = getattr(joint, forcelimited_field_name)
            subsub.prop(joint, "%s_%s_drive_max_force" % (typeprefix, ax))
            sub.prop(joint, "%s_%s_drive_stiffness" % (typeprefix, ax))
            sub.prop(joint, "%s_%s_drive_damping" % (typeprefix, ax))


class KHR_PT_rigid_body_constraint_drives_angular(
    KHR_PT_rigid_body_constraint_panel_base
):
    bl_label = "Angular"
    bl_parent_id = "KHR_PT_rigid_body_constraint_drives"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return KHR_PT_rigid_body_constraint_panel.poll(context)

    def draw(self, context):
        joint = context.object.khr_physics_extra_constraint_props
        layout = self.layout
        layout.use_property_split = True
        flow = layout.grid_flow(
            row_major=True, columns=0, even_columns=True, even_rows=False, align=True
        )
        KHR_PT_rigid_body_constraint_drives._generateDriveSettings(
            flow, joint, linear=False
        )


class KHR_PT_rigid_body_constraint_drives_linear(
    KHR_PT_rigid_body_constraint_panel_base
):
    bl_label = "Linear"
    bl_parent_id = "KHR_PT_rigid_body_constraint_drives"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return KHR_PT_rigid_body_constraint_panel.poll(context)

    def draw(self, context):
        joint = context.object.khr_physics_extra_constraint_props
        layout = self.layout
        layout.use_property_split = True
        flow = layout.grid_flow(
            row_major=True, columns=0, even_columns=True, even_rows=False, align=True
        )
        KHR_PT_rigid_body_constraint_drives._generateDriveSettings(
            flow, joint, linear=True
        )


registered_classes = [
    KHR_rigid_body_exporter_properties,
    KHR_rigid_body_importer_properties,
    KHR_rigid_body_node_properties,
    KHR_rigid_body_constraint_node_properties,
    KHR_PT_rigid_body_visualizer,
    KHR_PT_rigid_body_panel,
    KHR_PT_rigid_body_motion,
    KHR_PT_rigid_body_shape,
    KHR_PT_rigid_body_collections,
    KHR_PT_rigid_body_mass,
    KHR_PT_rigid_body_other,
    KHR_PT_rigid_body_constraint_panel,
    KHR_PT_rigid_body_constraint_drives,
    KHR_PT_rigid_body_constraint_drives_angular,
    KHR_PT_rigid_body_constraint_drives_linear,
    KHR_OT_DrawVelocityOperator,
    KHR_OT_DrawMassPropsOperator,
]


def register_ui():
    for panel in registered_classes:
        bpy.utils.register_class(panel)

    bpy.types.Scene.khr_physics_exporter_props = bpy.props.PointerProperty(
        type=KHR_rigid_body_exporter_properties
    )
    bpy.types.Scene.khr_physics_importer_props = bpy.props.PointerProperty(
        type=KHR_rigid_body_importer_properties
    )
    bpy.types.Object.khr_physics_extra_props = bpy.props.PointerProperty(
        type=KHR_rigid_body_node_properties
    )
    bpy.types.Object.khr_physics_extra_constraint_props = bpy.props.PointerProperty(
        type=KHR_rigid_body_constraint_node_properties
    )


def unregister_ui():
    for panel in registered_classes:
        bpy.utils.unregister_class(panel)
    del bpy.types.Scene.khr_physics_exporter_props
    del bpy.types.Object.khr_physics_extra_props
