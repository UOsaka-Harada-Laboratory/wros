#!/usr/bin/env python3

import os
import numpy as np
from panda3d.core import Vec3, Mat4
from transforms3d.quaternions import mat2quat

import rospy
import tf2_ros
import rosparam
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

import modeling._ode_cdhelper as mcd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import visualization.panda.world as wd
import grasping.planning.antipodal as gpa

import pyhiro.freesuc as fs
import pyhiro.pandactrl as pc
import pyhiro.pandageom as pg


class GraspPlanner():
    """ Grasp planner class. """

    def __init__(self):

        gripper_name = rospy.get_param('~gripper_name', 'robotiqhe')
        if gripper_name in ['robotiqhe', 'robotiq85', 'robotiq140']:
            self.base = wd.World(cam_pos=[1, 1, 1], lookat_pos=[0, 0, 0])

            self.angle_between_contact_normals = \
                rospy.get_param(
                    '~antipodal_grasp/angle_between_contact_normals', 90)  # noqa
            self.openning_direction = \
                rospy.get_param(
                    '~antipodal_grasp/openning_direction', 'loc_x')  # noqa
            self.max_samples = \
                rospy.get_param(
                    '~antipodal_grasp/max_samples', 4)  # noqa
            self.min_dist_between_sampled_contact_points = \
                rospy.get_param(
                    '~antipodal_grasp/min_dist_between_sampled_contact_points', .016)  # noqa
            self.contact_offset = \
                rospy.get_param(
                    '~antipodal_grasp/contact_offset', .016)  # noqa
        elif gripper_name in ['suction', 'sgb30']:
            self.base = pc.World(camp=[500, 500, 500], lookatp=[0, 0, 0])

            self.torque_resistance = \
                rospy.get_param(
                    '~contact/torque_resistance', 100)  # noqa
            self.min_distance = \
                rospy.get_param(
                    '~contact/min_distance', .1)  # noqa
            self.reduce_radius = \
                rospy.get_param(
                    '~contact/reduce_radius', 100)  # noqa
        else:
            rospy.logerr("The specified gripper is not implemented.")
        self.base.taskMgr.step()

        if gripper_name == 'robotiqhe':
            import robot_sim.end_effectors.gripper.robotiqhe.robotiqhe as gr
            self.gripper = gr.RobotiqHE()
            self.body_stl_path = self.gripper.lft.lnks[0]['mesh_file']
            self.fingers_dict = {
                'gripper.lft.lnks.1': self.gripper.lft.lnks[1],
                'gripper.rgt.lnks.1': self.gripper.rgt.lnks[1]}
        elif gripper_name == 'robotiq85':
            import robot_sim.end_effectors.gripper.robotiq85.robotiq85 as gr
            self.gripper = gr.Robotiq85()
            self.body_stl_path = self.gripper.lft_outer.lnks[0]['mesh_file']
            self.fingers_dict = {
                'gripper.lft_outer.lnks.1': self.gripper.lft_outer.lnks[1],
                'gripper.rgt_outer.lnks.1': self.gripper.rgt_outer.lnks[1],
                'gripper.lft_outer.lnks.2': self.gripper.lft_outer.lnks[2],
                'gripper.rgt_outer.lnks.2': self.gripper.rgt_outer.lnks[2],
                'gripper.lft_outer.lnks.3': self.gripper.lft_outer.lnks[3],
                'gripper.rgt_outer.lnks.3': self.gripper.rgt_outer.lnks[3],
                'gripper.lft_outer.lnks.4': self.gripper.lft_outer.lnks[4],
                'gripper.rgt_outer.lnks.4': self.gripper.rgt_outer.lnks[4],
                'gripper.lft_inner.lnks.1': self.gripper.lft_inner.lnks[1],
                'gripper.rgt_inner.lnks.1': self.gripper.rgt_inner.lnks[1]}
        elif gripper_name == 'robotiq140':
            import robot_sim.end_effectors.gripper.robotiq140.robotiq140 as gr
            self.gripper = gr.Robotiq140()
            self.body_stl_path = self.gripper.lft_outer.lnks[0]['mesh_file']
            self.fingers_dict = {
                'gripper.lft_outer.lnks.1': self.gripper.lft_outer.lnks[1],
                'gripper.rgt_outer.lnks.1': self.gripper.rgt_outer.lnks[1],
                'gripper.lft_outer.lnks.2': self.gripper.lft_outer.lnks[2],
                'gripper.rgt_outer.lnks.2': self.gripper.rgt_outer.lnks[2],
                'gripper.lft_outer.lnks.3': self.gripper.lft_outer.lnks[3],
                'gripper.rgt_outer.lnks.3': self.gripper.rgt_outer.lnks[3],
                'gripper.lft_outer.lnks.4': self.gripper.lft_outer.lnks[4],
                'gripper.rgt_outer.lnks.4': self.gripper.rgt_outer.lnks[4],
                'gripper.lft_inner.lnks.1': self.gripper.lft_inner.lnks[1],
                'gripper.rgt_inner.lnks.1': self.gripper.rgt_inner.lnks[1]}
        elif gripper_name == 'suction':
            import robot_sim.end_effectors.single_contact.suction.sandmmbs.sdmbs as gr  # noqa
            self.gripper = gr
            self.body_stl_path = str(self.gripper.Sdmbs().mbs_stlpath)
        elif gripper_name == 'sgb30':
            import robot_sim.end_effectors.single_contact.suction.sgb30.sgb30 as gr  # noqa
            self.gripper = gr
            self.body_stl_path = str(self.gripper.SGB30().sgb_stlpath)
        else:
            rospy.logerr("The specified gripper is not implemented.")
        gm.gen_frame().attach_to(self.base)
        self.base.taskMgr.step()

        self.object_stl_path = rospy.get_param(
            '~object_mesh_path',
            '/catkin_ws/src/wros_tutorials/meshes/bunnysim.stl')
        self.grasp_target = cm.CollisionModel(self.object_stl_path)
        self.grasp_target.set_rgba([.9, .75, .35, .3])
        self.grasp_target.attach_to(self.base)
        self.base.taskMgr.step()

        self.obstcl_stl_path = rospy.get_param('~obstcl_mesh_path', '')
        if self.obstcl_stl_path != '':
            self.obstacles = []
            self.obstacles.append(cm.CollisionModel(self.object_stl_path))
            self.obstacles.append(cm.CollisionModel(self.obstcl_stl_path))
            for obstacle in self.obstacles:
                obstacle.attach_to(self.base)
                self.base.taskMgr.step()
        self.vis_failures = rospy.get_param('~vis_failures', False)
        self.save_results = rospy.get_param('~save_results', False)
        self.yaml_filename = rospy.get_param('~planner_params', "")

        self.markers = MarkerArray()
        pose = Pose()
        pose.position.x = 0.
        pose.position.y = 0.
        pose.position.z = 0.
        pose.orientation.x = 0.
        pose.orientation.y = 0.
        pose.orientation.z = 0.
        pose.orientation.w = 1.
        scale = [1., 1., 1.]
        if gripper_name in ['robotiqhe', 'robotiq85', 'robotiq140']:
            pass
        elif gripper_name in ['suction', 'sgb30']:
            scale = [0.001, 0.001, 0.001]
        else:
            rospy.logerr("The specified gripper is not implemented.")
        self.markers.markers.append(
            self.gen_marker(
                'base_link',
                'object',
                0,
                pose,
                self.object_stl_path,
                scale=scale,
                color=[1.0, 0.5, 0.5, 0.5]))
        if self.obstcl_stl_path != '':
            self.markers.markers.append(
                self.gen_marker(
                    'base_link',
                    'obstcl',
                    0,
                    pose,
                    self.obstcl_stl_path,
                    scale=scale,
                    color=[1.0, 0.5, 0.5, 0.5]))

        self.pose_dict = {}
        self.br = tf2_ros.StaticTransformBroadcaster()
        if gripper_name in ['robotiqhe', 'robotiq85', 'robotiq140']:
            if self.obstcl_stl_path == '':
                self.planning_service = rospy.Service(
                    'plan_grasp', Empty, self.plan_antipodal_grasps_single_object)
            else:
                self.planning_service = rospy.Service(
                    'plan_grasp', Empty, self.plan_antipodal_grasps)
        elif gripper_name in ['suction', 'sgb30']:
            if self.obstcl_stl_path == '':
                self.planning_service = rospy.Service(
                    'plan_grasp', Empty, self.plan_suction_grasps_single_object)
            else:
                self.planning_service = rospy.Service(
                    'plan_grasp', Empty, self.plan_suction_grasps)
        else:
            rospy.logerr("The specified gripper is not implemented.")
        self.marker_pub = rospy.Publisher(
            'grasp_pub', MarkerArray, queue_size=1)

    def update_tfs(self):
        """ Sends tfs. """

        if self.pose_dict is not {}:
            for name, data in self.pose_dict.items():
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = data['parent']
                t.child_frame_id = name
                t.transform.translation.x = data['pose'].position.x
                t.transform.translation.y = data['pose'].position.y
                t.transform.translation.z = data['pose'].position.z
                t.transform.rotation.x = data['pose'].orientation.x
                t.transform.rotation.y = data['pose'].orientation.y
                t.transform.rotation.z = data['pose'].orientation.z
                t.transform.rotation.w = data['pose'].orientation.w
                self.br.sendTransform(t)

    def gen_marker(
            self,
            frame_name,
            name,
            id_int,
            pose,
            stl_path,
            scale=[1., 1., 1.],
            color=[0., 0., 0., 0.]):
        """ Generates a marker.

            Attributes:
                frame_name (str): Frame name
                name (str): Unique marker name
                id_int (int): Unique id number
                pose (geometry_msgs/Pose): Pose of the marker
                stl_path (str): Stl file path
                scale (list(float)): Object scale to be displayed
                color (list(float)): Object color to be displayed
        """

        marker = Marker()
        marker.header.frame_id = frame_name
        marker.header.stamp = rospy.Time()
        marker.ns = name
        marker.id = id_int
        marker.type = marker.MESH_RESOURCE
        marker.action = marker.ADD
        marker.pose = pose
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])
        marker.color.a = float(color[0])
        marker.color.r = float(color[1])
        marker.color.g = float(color[2])
        marker.color.b = float(color[3])
        marker.mesh_resource = 'file://' + stl_path
        marker.mesh_use_embedded_materials = True

        return marker

    def plan_suction_grasps_single_object(self, req):
        """ Plans suction grasps for a single object. """

        contact_planner = fs.Freesuc(
            self.object_stl_path,
            handpkg=self.gripper,
            torqueresist=self.torque_resistance)
        contact_planner.removeBadSamples(
            mindist=self.min_distance)
        contact_planner.clusterFacetSamplesRNN(
            reduceRadius=self.reduce_radius)
        pg.plotAxisSelf(self.base.render, Vec3(0, 0, 0))
        contact_planner.removeHndcc(self.base)
        objnp = pg.packpandanp(
            contact_planner.objtrimesh.vertices,
            contact_planner.objtrimesh.face_normals,
            contact_planner.objtrimesh.faces,
            name='')
        objnp.setColor(.37, .37, .35, 1)
        objnp.reparentTo(self.base.render)
        rospy.loginfo(
            "Number of generated grasps: %s",
            len(contact_planner.sucrotmats))

        contact_result = []
        parent_frame = 'object'
        for i, hndrot in enumerate(contact_planner.sucrotmats):
            tmphand = self.gripper.newHandNM(hndcolor=[.7, .7, .7, .7])
            centeredrot = Mat4(hndrot)
            tmphand.setMat(centeredrot)
            tmphand.reparentTo(self.base.render)
            tmphand.setColor(.5, .5, .5, .3)
            contact_pos = [tmphand.getPos()[i] / 1000 for i in range(3)]
            mat = tmphand.getMat()
            contact_mat = \
                [list([mat[i][0], mat[i][1], mat[i][2]]) for i in range(3)]

            pose_b = Pose()
            pose_b.position.x = contact_pos[0]
            pose_b.position.y = contact_pos[1]
            pose_b.position.z = contact_pos[2]
            q = mat2quat(np.array(contact_mat).T)
            pose_b.orientation.x = q[1]
            pose_b.orientation.y = q[2]
            pose_b.orientation.z = q[3]
            pose_b.orientation.w = q[0]
            self.markers.markers.append(
                self.gen_marker(
                    parent_frame,
                    'body_'+str(i),
                    0,
                    pose_b,
                    self.body_stl_path,
                    scale=[0.001, 0.001, 0.001],
                    color=[0.2, 0.8, 0.8, 0.8]))
            self.pose_dict['body_'+str(i)] = \
                {'parent': parent_frame, 'pose': pose_b}
            self.update_tfs()

            contact_result_i = np.concatenate([contact_pos, q])
            contact_result.append(contact_result_i)

        if self.save_results:
            result_filename = os.path.join(
                os.path.dirname(self.object_stl_path),
                '../results/',
                os.path.splitext(self.yaml_filename)[0] + '.txt')
            np.savetxt(result_filename, contact_result)

        return EmptyResponse()

    def plan_suction_grasps(self, req):
        """ Plans suction grasps. """

        contact_planner = fs.Freesuc(
            self.object_stl_path,
            handpkg=self.gripper,
            torqueresist=self.torque_resistance)
        contact_planner.removeBadSamples(
            mindist=self.min_distance)
        contact_planner.clusterFacetSamplesRNN(
            reduceRadius=self.reduce_radius)
        pg.plotAxisSelf(self.base.render, Vec3(0, 0, 0))
        contact_planner.removeHndcc(self.base)
        objnp = pg.packpandanp(
            contact_planner.objtrimesh.vertices,
            contact_planner.objtrimesh.face_normals,
            contact_planner.objtrimesh.faces,
            name='')
        objnp.setColor(.37, .37, .35, 1)
        objnp.reparentTo(self.base.render)
        rospy.loginfo(
            "Number of generated grasps: %s",
            len(contact_planner.sucrotmats))

        contact_result = []
        parent_frame = 'object'
        for i, hndrot in enumerate(contact_planner.sucrotmats):
            tmphand = self.gripper.newHandNM(hndcolor=[.7, .7, .7, .7])
            centeredrot = Mat4(hndrot)
            tmphand.setMat(centeredrot)
            tmphand.reparentTo(self.base.render)
            tmphand.setColor(.5, .5, .5, .3)
            contact_pos = [tmphand.getPos()[i] / 1000 for i in range(3)]
            mat = tmphand.getMat()
            contact_mat = \
                [list([mat[i][0], mat[i][1], mat[i][2]]) for i in range(3)]

            pose_b = Pose()
            pose_b.position.x = contact_pos[0]
            pose_b.position.y = contact_pos[1]
            pose_b.position.z = contact_pos[2]
            q = mat2quat(np.array(contact_mat).T)
            pose_b.orientation.x = q[1]
            pose_b.orientation.y = q[2]
            pose_b.orientation.z = q[3]
            pose_b.orientation.w = q[0]
            self.markers.markers.append(
                self.gen_marker(
                    parent_frame,
                    'body_'+str(i),
                    0,
                    pose_b,
                    self.body_stl_path,
                    scale=[0.001, 0.001, 0.001],
                    color=[0.2, 0.8, 0.8, 0.8]))
            self.pose_dict['body_'+str(i)] = \
                {'parent': parent_frame, 'pose': pose_b}
            self.update_tfs()

            contact_result_i = np.concatenate([contact_pos, q])
            contact_result.append(contact_result_i)

        if self.save_results:
            result_filename = os.path.join(
                os.path.dirname(self.object_stl_path),
                '../results/',
                os.path.splitext(self.yaml_filename)[0] + '.txt')
            np.savetxt(result_filename, contact_result)

        return EmptyResponse()

    def plan_antipodal_grasps_single_object(self, req):
        """ Plans antipodal grasps for a single object. """

        grasp_info_list = gpa.plan_grasps(
            self.gripper,
            self.grasp_target,
            angle_between_contact_normals=np.radians(self.angle_between_contact_normals),  # noqa
            openning_direction=self.openning_direction,  # noqa
            max_samples=self.max_samples,  # noqa
            min_dist_between_sampled_contact_points=self.min_dist_between_sampled_contact_points,  # noqa
            contact_offset=self.contact_offset)  # noqa
        rospy.loginfo(
            "Number of generated grasps: %s",
            len(grasp_info_list))

        grasp_result = []
        for i, grasp_info in enumerate(grasp_info_list):
            jaw_width, jaw_pos, jaw_rotmat, hnd_pos, hnd_rotmat = grasp_info
            self.gripper.grip_at_with_jcpose(jaw_pos, jaw_rotmat, jaw_width)
            self.gripper.gen_meshmodel().attach_to(self.base)

            parent_frame = 'object'
            pose_b = Pose()
            pose_b.position.x = hnd_pos[0]
            pose_b.position.y = hnd_pos[1]
            pose_b.position.z = hnd_pos[2]
            q = mat2quat(hnd_rotmat)
            pose_b.orientation.x = q[1]
            pose_b.orientation.y = q[2]
            pose_b.orientation.z = q[3]
            pose_b.orientation.w = q[0]
            self.markers.markers.append(
                self.gen_marker(
                    parent_frame,
                    'body_'+str(i),
                    0,
                    pose_b,
                    self.body_stl_path))
            self.pose_dict['body_'+str(i)] = \
                {'parent': parent_frame, 'pose': pose_b}
            self.update_tfs()

            grasp_result_i = np.concatenate([hnd_pos, q])
            grasp_result.append(grasp_result_i)

            for k, v in self.fingers_dict.items():
                pose = Pose()
                pose.position.x = v['gl_pos'][0]
                pose.position.y = v['gl_pos'][1]
                pose.position.z = v['gl_pos'][2]
                q = mat2quat(v['gl_rotmat'])
                pose.orientation.x = q[1]
                pose.orientation.y = q[2]
                pose.orientation.z = q[3]
                pose.orientation.w = q[0]
                scale = [1., 1., 1.]
                if v['scale'] is not None:
                    scale = v['scale']
                self.markers.markers.append(
                    self.gen_marker(
                        parent_frame,
                        k+'_'+str(i),
                        0,
                        pose,
                        v['mesh_file'],
                        scale))
                self.pose_dict[k+'_'+str(i)] = \
                    {'parent': parent_frame, 'pose': pose}

        if self.save_results:
            result_filename = os.path.join(
                os.path.dirname(self.object_stl_path),
                '../results/',
                os.path.splitext(self.yaml_filename)[0] + '.txt')
            np.savetxt(result_filename, grasp_result)

        return EmptyResponse()

    def plan_antipodal_grasps(self, req):
        """ Plans antipodal grasps. """

        grasp_info_list = gpa.plan_grasps(
            self.gripper,
            self.grasp_target,
            angle_between_contact_normals=np.radians(self.angle_between_contact_normals),  # noqa
            openning_direction=self.openning_direction,  # noqa
            max_samples=self.max_samples,  # noqa
            min_dist_between_sampled_contact_points=self.min_dist_between_sampled_contact_points,  # noqa
            contact_offset=self.contact_offset)  # noqa
        rospy.loginfo(
            "Number of generated grasps: %s",
            len(grasp_info_list))

        grasp_result = []
        for i, grasp_info in enumerate(grasp_info_list):
            jaw_width, jaw_pos, jaw_rotmat, hnd_pos, hnd_rotmat = grasp_info
            self.gripper.grip_at_with_jcpose(jaw_pos, jaw_rotmat, jaw_width)

            # checking collisions
            collisionfree_list = []
            for obstacle in self.obstacles:
                for cdmesh in self.gripper.gen_meshmodel().cm_list:
                    is_collide, cps = mcd.is_collided(
                        cdmesh.cdmesh, obstacle.cdmesh)
                    collisionfree_list.append(not is_collide)

            # if collision-free
            if all(collisionfree_list):
                self.gripper.gen_meshmodel(
                    rgba=[0, 1, 0, .3]).attach_to(self.base)
            else:
                if not self.vis_failures:
                    continue
                self.gripper.gen_meshmodel(
                    rgba=[1, 0, 0, .3]).attach_to(self.base)

            parent_frame = 'object'
            pose_b = Pose()
            pose_b.position.x = hnd_pos[0]
            pose_b.position.y = hnd_pos[1]
            pose_b.position.z = hnd_pos[2]
            q = mat2quat(hnd_rotmat)
            pose_b.orientation.x = q[1]
            pose_b.orientation.y = q[2]
            pose_b.orientation.z = q[3]
            pose_b.orientation.w = q[0]
            self.markers.markers.append(
                self.gen_marker(
                    parent_frame,
                    'body_'+str(i),
                    0,
                    pose_b,
                    self.body_stl_path))
            self.pose_dict['body_'+str(i)] = \
                {'parent': parent_frame, 'pose': pose_b}
            self.update_tfs()

            grasp_result_i = np.concatenate([hnd_pos, q])
            grasp_result.append(grasp_result_i)

            for k, v in self.fingers_dict.items():
                pose = Pose()
                pose.position.x = v['gl_pos'][0]
                pose.position.y = v['gl_pos'][1]
                pose.position.z = v['gl_pos'][2]
                q = mat2quat(v['gl_rotmat'])
                pose.orientation.x = q[1]
                pose.orientation.y = q[2]
                pose.orientation.z = q[3]
                pose.orientation.w = q[0]
                scale = [1., 1., 1.]
                if v['scale'] is not None:
                    scale = v['scale']
                self.markers.markers.append(
                    self.gen_marker(
                        parent_frame,
                        k+'_'+str(i),
                        0,
                        pose,
                        v['mesh_file'],
                        scale))
                self.pose_dict[k+'_'+str(i)] = \
                    {'parent': parent_frame, 'pose': pose}

        if self.save_results:
            result_filename = os.path.join(
                os.path.dirname(self.object_stl_path),
                '../results/',
                os.path.splitext(self.yaml_filename)[0] + '.txt')
            np.savetxt(result_filename, grasp_result)

        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('grasp_planning_server')
    planner = GraspPlanner()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        planner.marker_pub.publish(planner.markers)
        planner.base.taskMgr.step()
        planner.update_tfs()
        rate.sleep()
