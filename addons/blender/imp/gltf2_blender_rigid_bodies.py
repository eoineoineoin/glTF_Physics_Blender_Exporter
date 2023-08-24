import bpy
from ...io.com.gltf2_io_collision_shapes import *
from ...io.com.gltf2_io_rigid_bodies import *


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
        cgExt = gltf.data.extensions.get(collisionGeom_Extension_Name)
        if cgExt != None:
            self.cgExt = CollisionGeomGlTFExtension.from_dict(cgExt)
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
            or nodeExt.rigid_motion != None
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

            colliderIdx = -1
            if nodeExt.trigger != None:
                colliderIdx = nodeExt.trigger.shape
            if nodeExt.collider != None:
                colliderIdx = nodeExt.collider.shape

            if colliderIdx != -1:
                shape = self.cgExt.shapes[colliderIdx]
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

                # <todo.eoin Figure out if we can hook in a different mesh
                # other than the one associated with this node
                if shape.convex != None:
                    blender_object.rigid_body.collision_shape = "CONVEX_HULL"
                if shape.trimesh != None:
                    blender_object.rigid_body.collision_shape = "MESH"

                # todo.eoin Collision systems

            if nodeExt.collider != None and nodeExt.collider.physics_material != None:
                mat = self.rbExt.physics_materials[nodeExt.collider.physics_material]
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

        if nodeExt.rigid_motion:
            blender_object.rigid_body.enabled = True
            if nodeExt.rigid_motion.mass != None:
                blender_object.rigid_body.mass = nodeExt.rigid_motion.mass
                if nodeExt.rigid_motion.mass == 0:
                    blender_object.khr_physics_extra_props.infinite_mass = True
            if nodeExt.rigid_motion.is_kinematic != None:
                blender_object.rigid_body.is_kinematic = (
                    nodeExt.rigid_motion.is_kinematic
                )
            if nodeExt.rigid_motion.center_of_mass != None:
                blender_object.khr_physics_extra_props.center_of_mass = (
                    nodeExt.rigid_motion.center_of_mass
                )
                blender_object.khr_physics_extra_props.enable_com_override = True
            if nodeExt.rigid_motion.inertia_diagonal != None:
                it = nodeExt.rigid_motion.inertia_diagonal
                blender_object.khr_physics_extra_props.inertia_major_axis = it
                blender_object.khr_physics_extra_props.enable_inertia_override = True
            if nodeExt.rigid_motion.inertia_orientation != None:
                io = nodeExt.rigid_motion.inertia_orientation.to_euler()
                blender_object.khr_physics_extra_props.inertia_orientation = io
                blender_object.khr_physics_extra_props.enable_inertia_override = True
            if nodeExt.rigid_motion.linear_velocity != None:
                blender_object.khr_physics_extra_props.linear_velocity = (
                    nodeExt.rigid_motion.linear_velocity
                )
            if nodeExt.rigid_motion.angular_velocity != None:
                blender_object.khr_physics_extra_props.angular_velocity = (
                    nodeExt.rigid_motion.angular_velocity
                )
            if nodeExt.rigid_motion.gravity_factor != None:
                blender_object.khr_physics_extra_props.gravity_factor = (
                    nodeExt.rigid_motion.gravity_factor
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
            joint.type = "GENERIC"
            if nodeExt.joint.enable_collision != None:
                blender_object.rigid_body_constraint.disable_collisions = (
                    not nodeExt.joint.enable_collision
                )

            limitSet = self.rbExt.physics_joint_limits[nodeExt.joint.joint_limits]
            for limit in limitSet.joint_limits:
                minLimit = limit.min_limit if limit.min_limit != None else 0
                maxLimit = limit.max_limit if limit.max_limit != None else 0
                X, Y, Z = (0, 2, 1)
                if limit.linear_axes != None:
                    for axIdx in limit.linear_axes:
                        if axIdx == X:
                            joint.use_limit_lin_x = True
                            joint.limit_lin_x_lower = minLimit
                            joint.limit_lin_x_upper = maxLimit
                        if axIdx == Y:
                            joint.use_limit_lin_y = True
                            joint.limit_lin_y_lower = minLimit
                            joint.limit_lin_y_upper = maxLimit
                        if axIdx == Z:
                            joint.use_limit_lin_z = True
                            joint.limit_lin_z_lower = minLimit
                            joint.limit_lin_z_upper = maxLimit
                if limit.angular_axes != None:
                    for axIdx in limit.angular_axes:
                        if axIdx == X:
                            joint.use_limit_ang_x = True
                            joint.limit_ang_x_lower = minLimit
                            joint.limit_ang_x_upper = maxLimit
                        if axIdx == Y:
                            joint.use_limit_ang_y = True
                            joint.limit_ang_y_lower = minLimit
                            joint.limit_ang_y_upper = maxLimit
                        if axIdx == Z:
                            joint.use_limit_ang_z = True
                            joint.limit_ang_z_lower = minLimit
                            joint.limit_ang_z_upper = maxLimit
