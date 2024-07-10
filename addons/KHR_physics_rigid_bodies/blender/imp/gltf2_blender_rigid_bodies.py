import bpy
from ...io.com.gltf2_io_collision_shapes import *
from ...io.com.gltf2_io_rigid_bodies import *
from typing import cast


class JointFixup:
    """Helper class to store information about how to connect a joint"""

    def __init__(self, joint, connected_idx):
        self.joint = joint
        self.connected_idx = connected_idx


class glTF2ImportUserExtension:
    def __init__(self):
        # We need to wait until we create the gltf2UserExtension to import the gltf2 modules
        # Otherwise, it may fail because the gltf2 may not be loaded yet
        from io_scene_gltf2.io.com.gltf2_io_extensions import Extension
        from io_scene_gltf2.io.com.gltf2_io_extensions import ChildOfRootExtension

        self.Extension = Extension
        self.ChildOfRootExtension = ChildOfRootExtension

        self.properties = bpy.context.scene.khr_physics_exporter_props

        # Additional mapping to hook up joints
        self.vnode_to_blender = {}
        self.joints_to_fixup = []

    def gather_import_gltf_before_hook(self, gltf):
        if not gltf.data.extensions:
            return
        csExt = gltf.data.extensions.get(collisionGeom_Extension_Name)
        if csExt != None:
            self.csExt = CollisionShapesGlTFExtension.from_dict(csExt)
        rbExt = gltf.data.extensions.get(rigidBody_Extension_Name)
        if rbExt != None:
            self.rbExt = RigidBodiesGlTFExtension.from_dict(rbExt)
            try:
                # We need to ensure the scene has a physics world;
                # This is created automatically when we create a rigid body
                # but not when we create a joint. This ensures that if a
                # joint node is seen first, we can still create the joint
                bpy.ops.rigidbody.world_add()
            except RuntimeError:
                # Can trigger if there's already a world in the scene
                pass

    def _find_parent_body(self, blender_node):
        while blender_node:
            if blender_node.rigid_body != None:
                return blender_node
            blender_node = blender_node.parent
        return None

    def gather_import_scene_after_nodes_hook(self, gltf_scene, blender_scene, gltf):
        for fixup in self.joints_to_fixup:
            other_vnode = gltf.vnodes[fixup.connected_idx]
            other = self.vnode_to_blender[other_vnode]
            body_a = self._find_parent_body(fixup.joint)
            body_b = self._find_parent_body(other)

            fixup.joint.rigid_body_constraint.object1 = body_a
            fixup.joint.rigid_body_constraint.object2 = body_b

    def gather_import_node_after_hook(self, vnode, gltf_node, blender_object, gltf):
        try:
            self.gather_import_node_after_hook_2(vnode, gltf_node, blender_object, gltf)
        except:
            import traceback
            print(traceback.format_exc())

    def gather_import_node_after_hook_2(self, vnode, gltf_node, blender_object, gltf):
        if not self.properties.enabled:
            return

        self.vnode_to_blender[vnode] = blender_object

        try:
            ext = gltf_node.extensions[rigidBody_Extension_Name]
        except:
            return

        nodeExt = RigidBodiesNodeExtension.from_dict(ext)

        if (
            nodeExt.collider != None
            or nodeExt.trigger != None
            or nodeExt.motion != None
        ):
            if not blender_object.rigid_body:
                # <todo.eoin This is the only way I've found to add a rigid body to a node
                # There might be a cleaner way.
                prev_active_objects = bpy.context.view_layer.objects.active
                bpy.context.view_layer.objects.active = blender_object
                bpy.ops.rigidbody.object_add()
                bpy.context.view_layer.objects.active = prev_active_objects
            blender_object.rigid_body.enabled = False  # Static by default
            blender_object.rigid_body.collision_shape = "COMPOUND"

            colliderIdx = None
            if nodeExt.trigger != None:
                colliderIdx = nodeExt.trigger.shape
            if nodeExt.collider != None:
                colliderIdx = nodeExt.collider.shape

            if colliderIdx != None:
                shape = self.csExt.shapes[cast(int, colliderIdx)]
                if shape.sphere != None:
                    blender_object.rigid_body.collision_shape = "SPHERE"
                if shape.box != None:
                    blender_object.rigid_body.collision_shape = "BOX"

                # <todo.eoin Might need to undo node transform for these?
                if shape.capsule != None:
                    blender_object.rigid_body.collision_shape = "CAPSULE"
                    ep = blender_object.khr_physics_extra_props
                    ep.cone_capsule_radius_bottom = shape.capsule.radiusBottom
                    ep.cone_capsule_radius_top = shape.capsule.radiusTop
                    ep.cone_capsule_height = shape.capsule.height
                    ep.cone_capsule_override = (
                        shape.capsule.radiusBottom != shape.capsule.radiusTop
                    )
                if shape.cylinder != None:
                    blender_object.rigid_body.collision_shape = "CYLINDER"
                    ep = blender_object.khr_physics_extra_props
                    ep.cone_capsule_radius_bottom = shape.cylinder.radiusBottom
                    ep.cone_capsule_radius_top = shape.cylinder.radiusTop
                    ep.cone_capsule_height = shape.cylinder.height
                    ep.cone_capsule_override = (
                        shape.cylinder.radiusBottom != shape.cylinder.radiusTop
                    )
                    if shape.cylinder.radiusTop == 0:
                        blender_object.rigid_body.collision_shape = "CONE"

                # <todo.eoin Figure out if we can hook in a different mesh/skin/weights
                # other than the one associated with this node
                if shape.mesh != None:
                    blender_object.rigid_body.collision_shape = "MESH"

                    if (
                        shape.extensions
                        and rigidBody_Extension_Name in shape.extensions
                    ):
                        rbShapeExt = RigidBodiesShapeExtension.from_dict(
                            shape.extensions[rigidBody_Extension_Name]
                        )
                        if rbShapeExt.convexHull:
                            blender_object.rigid_body.collision_shape = "CONVEX_HULL"

                # todo.eoin Collision systems

            if nodeExt.collider != None and nodeExt.collider.physics_material != None:
                mat = self.rbExt.materials[cast(int, nodeExt.collider.physics_material)]
                if mat.dynamic_friction != None:
                    blender_object.rigid_body.friction = mat.dynamic_friction
                if mat.restitution != None:
                    blender_object.rigid_body.restitution = mat.restitution
                if mat.friction_combine != None:
                    blender_object.khr_physics_extra_props.friction_combine = (
                        mat.friction_combine
                    )
                if mat.restitution_combine != None:
                    blender_object.khr_physics_extra_props.restitution_combine = (
                        mat.restitution_combine
                    )

        if nodeExt.motion:
            blender_object.rigid_body.enabled = True
            if nodeExt.motion.mass != None:
                blender_object.rigid_body.mass = nodeExt.motion.mass
                if nodeExt.motion.mass == 0:
                    blender_object.khr_physics_extra_props.infinite_mass = True
            if nodeExt.motion.is_kinematic != None:
                blender_object.rigid_body.is_kinematic = nodeExt.motion.is_kinematic
            if nodeExt.motion.center_of_mass != None:
                blender_object.khr_physics_extra_props.center_of_mass = (
                    nodeExt.motion.center_of_mass
                )
                blender_object.khr_physics_extra_props.enable_com_override = True
            if nodeExt.motion.inertia_diagonal != None:
                it = nodeExt.motion.inertia_diagonal
                blender_object.khr_physics_extra_props.inertia_major_axis = it
                blender_object.khr_physics_extra_props.enable_inertia_override = True
            if nodeExt.motion.inertia_orientation != None:
                io = nodeExt.motion.inertia_orientation.to_euler()
                blender_object.khr_physics_extra_props.inertia_orientation = io
                blender_object.khr_physics_extra_props.enable_inertia_override = True
            if nodeExt.motion.linear_velocity != None:
                blender_object.khr_physics_extra_props.linear_velocity = (
                    nodeExt.motion.linear_velocity
                )
            if nodeExt.motion.angular_velocity != None:
                blender_object.khr_physics_extra_props.angular_velocity = (
                    nodeExt.motion.angular_velocity
                )
            if nodeExt.motion.gravity_factor != None:
                blender_object.khr_physics_extra_props.gravity_factor = (
                    nodeExt.motion.gravity_factor
                )

        if nodeExt.joint:
            # <todo.eoin Same as adding rigid body; might be a cleaner way.
            prev_active_objects = bpy.context.view_layer.objects.active
            bpy.context.view_layer.objects.active = blender_object
            bpy.ops.rigidbody.constraint_add()
            bpy.context.view_layer.objects.active = prev_active_objects

            self.joints_to_fixup.append(
                JointFixup(blender_object, nodeExt.joint.connected_node)
            )

            joint = blender_object.rigid_body_constraint
            joint_extra = blender_object.khr_physics_extra_constraint_props
            joint.type = "GENERIC"
            if nodeExt.joint.enable_collision != None:
                blender_object.rigid_body_constraint.disable_collisions = (
                    not nodeExt.joint.enable_collision
                )

            assert nodeExt.joint.joint != None
            jointDesc = self.rbExt.joints[cast(int, nodeExt.joint.joint)]
            if jointDesc.drives:
                for drive in jointDesc.drives:
                    isLinear = drive.type == "linear"
                    isAcceleration = drive.mode == "acceleration"
                    axisname = "xzy"[drive.axis]
                    basisscale = (1.0, 1.0, -1.0)[drive.axis]
                    self._populateDrive(
                        joint_extra,
                        drive,
                        linear=isLinear,
                        axisname=axisname,
                        basisscale=basisscale,
                        acceleration=isAcceleration,
                    )

            for limit in jointDesc.limits:
                use_limit = limit.min_limit != None or limit.max_limit != None
                minLimit = limit.min_limit if limit.min_limit != None else 0
                maxLimit = limit.max_limit if limit.max_limit != None else 0
                (spring, damping) = (limit.spring_stiffness, limit.spring_damping)
                X, Y, Z = (0, 2, 1)
                if limit.linear_axes != None:
                    for axIdx in limit.linear_axes:
                        if axIdx == X:
                            joint.use_limit_lin_x = use_limit
                            joint.limit_lin_x_lower = minLimit
                            joint.limit_lin_x_upper = maxLimit
                            if spring != None or damping:
                                joint.type = "GENERIC_SPRING"
                                joint.use_spring_x = True
                                joint.spring_stiffness_x = spring
                                joint.spring_damping_x = damping
                        if axIdx == Y:
                            joint.use_limit_lin_y = use_limit
                            joint.limit_lin_y_upper = -minLimit
                            joint.limit_lin_y_lower = -maxLimit
                            if spring != None or damping:
                                joint.type = "GENERIC_SPRING"
                                joint.use_spring_y = True
                                joint.spring_stiffness_y = spring
                                joint.spring_damping_y = damping
                        if axIdx == Z:
                            joint.use_limit_lin_z = use_limit
                            joint.limit_lin_z_lower = minLimit
                            joint.limit_lin_z_upper = maxLimit
                            if spring != None or damping:
                                joint.type = "GENERIC_SPRING"
                                joint.use_spring_z = True
                                joint.spring_stiffness_z = spring
                                joint.spring_damping_z = damping
                if limit.angular_axes != None:
                    for axIdx in limit.angular_axes:
                        if axIdx == X:
                            joint.use_limit_ang_x = use_limit
                            joint.limit_ang_x_lower = minLimit
                            joint.limit_ang_x_upper = maxLimit
                            if spring != None or damping:
                                joint.type = "GENERIC_SPRING"
                                joint.use_spring_ang_x = True
                                joint.spring_stiffness_ang_x = spring
                                joint.spring_damping_ang_x = damping
                        if axIdx == Y:
                            joint.use_limit_ang_y = use_limit
                            joint.limit_ang_y_upper = -minLimit
                            joint.limit_ang_y_lower = -maxLimit
                            if spring != None or damping:
                                joint.type = "GENERIC_SPRING"
                                joint.use_spring_ang_y = True
                                joint.spring_stiffness_ang_y = spring
                                joint.spring_damping_ang_y = damping
                        if axIdx == Z:
                            joint.use_limit_ang_z = use_limit
                            joint.limit_ang_z_lower = minLimit
                            joint.limit_ang_z_upper = maxLimit
                            if spring != None or damping:
                                joint.type = "GENERIC_SPRING"
                                joint.use_spring_ang_z = True
                                joint.spring_stiffness_ang_z = spring
                                joint.spring_damping_ang_x = damping

    @staticmethod
    def _populateDrive(
        joint_extra,
        drive: Optional[JointDrive],
        linear: bool,
        acceleration: bool,
        axisname: str,
        basisscale: float,
    ):
        typeprefix = "lin" if linear else "ang"
        setattr(joint_extra, "use_%s_drive_%s" % (typeprefix, axisname), drive != None)

        if drive == None:
            return

        if acceleration:
            setattr(
                joint_extra, "%s_%s_drive_mode" % (typeprefix, axisname), "acceleration"
            )
        else:
            setattr(joint_extra, "%s_%s_drive_mode" % (typeprefix, axisname), "force")

        postarget = drive.position_target
        if postarget:
            postarget *= basisscale
        setattr(
            joint_extra, "%s_%s_drive_pos_target" % (typeprefix, axisname), postarget
        )

        veltarget = drive.velocity_target
        if veltarget:
            veltarget *= basisscale
        setattr(
            joint_extra, "%s_%s_drive_vel_target" % (typeprefix, axisname), veltarget
        )

        setattr(
            joint_extra,
            "%s_%s_drive_force_limited" % (typeprefix, axisname),
            bool(drive.max_force),
        )

        if drive.max_force:
            setattr(
                joint_extra,
                "%s_%s_drive_max_force" % (typeprefix, axisname),
                drive.max_force,
            )
        setattr(
            joint_extra,
            "%s_%s_drive_stiffness" % (typeprefix, axisname),
            drive.stiffness,
        )
        setattr(
            joint_extra, "%s_%s_drive_damping" % (typeprefix, axisname), drive.damping
        )
