#!/usr/bin/env python3

import os
import tf
import time
import math
import rospy
import rospkg
import numpy as np
from geometry_msgs.msg import (Pose,
                               Vector3,
                               Quaternion,
                               PoseStamped)
from std_srvs.srv import Empty, EmptyResponse
from moveit_msgs.msg import PlanningScene, ObjectColor
from visualization_msgs.msg import Marker, MarkerArray

import modeling.geometric_model as gm
import modeling.collision_model as cm
import visualization.panda.world as wd
import grasping.planning.antipodal as gpa
import robot_sim.end_effectors.gripper.robotiqhe.robotiqhe as he


def rotmat2q(rot):
    """ Converts the rotation matrix into quaternion.

        :param quat: the rotation matrix (shape: :math:`[3, 3]`)
        :type quat: numpy.ndarray
        :return: quaternion [x, y, z, w] (shape: :math:`[4,]`)
        :rtype: numpy.ndarray
    """

    R = np.eye(4)
    R[:3, :3] = rot
    return tf.transformations.quaternion_from_matrix(R)


def update_tfs(br, pose_dict):
    """ Sends tfs.

        Attributes:
            br: tf.TransformBroadcaster
            pose_dict: dict{name(str): pose(geometry_msgs/Pose)}
    """

    if pose_dict is not {}:
        for name, pose in pose_dict.items():
            br.sendTransform((pose.position.x,
                            pose.position.y,
                            pose.position.z),
                            (pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w),
                            rospy.Time.now(),
                            name,
                            'object')


def gen_marker(frame_name, name, id_int, pose, stl_path):
    """ Generates a marker.

        Attributes:
            frame_name (str): Frame name
            name (str): Unique marker name
            id_int (int): Unique id number
            pose (geometry_msgs/Pose): Pose of the marker
            stl_path (str): 
    """
    marker = Marker()
    marker.header.frame_id = frame_name
    marker.header.stamp = rospy.Time()
    marker.ns = name
    marker.id = id_int
    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD
    marker.pose = pose
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 0.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.mesh_resource = 'package://wros_tutorials/'+stl_path
    marker.mesh_use_embedded_materials = True

    return marker


def plan_grasps(req):
    """ Plans grasps. """

    grasp_info_list = gpa.plan_grasps(
        gripper,
        object_tube,
        angle_between_contact_normals=math.radians(90),
        openning_direction='loc_x',
        max_samples=4,
        min_dist_between_sampled_contact_points=.016,
        contact_offset=.016)

    for i, grasp_info in enumerate(grasp_info_list):
        jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info
        gripper.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
        gripper.gen_meshmodel().attach_to(base)

        pose = Pose()
        pose.position.x = hnd_pos[0]
        pose.position.y = hnd_pos[1]
        pose.position.z = hnd_pos[2]
        q = rotmat2q(hnd_rotmat)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        markers.markers.append(
            gen_marker('object', 'hande_b', i, pose, gripper_stl_path))
        markers.markers.append(
            gen_marker('object', 'hande_f1', i, pose, finger1_stl_path))
        markers.markers.append(
            gen_marker('object', 'hande_f2', i, pose, finger2_stl_path))

        pose_dict['hande_'+str(i)] = pose
    
    return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('grasp_tf_publisher')
    pkg_path = rospkg.RosPack().get_path('wros_tutorials')

    base = wd.World(cam_pos=[1, 1, 1], lookat_pos=[0, 0, 0])
    base.taskMgr.step()
    gripper_stl_path = ('meshes/hande/base_cvt.stl')
    finger1_stl_path = ('meshes/hande/finger1_cvt.stl')
    finger2_stl_path = ('meshes/hande/finger2_cvt.stl')
    gripper = he.RobotiqHE()
    gm.gen_frame().attach_to(base)
    base.taskMgr.step()
    object_stl_path = os.path.join(
        pkg_path, 'meshes/tubebig.stl')
    object_tube = cm.CollisionModel(object_stl_path)
    object_tube.set_rgba([.9, .75, .35, .3])
    object_tube.attach_to(base)
    base.taskMgr.step()

    markers = MarkerArray()
    pose = Pose()
    pose.position.x = 0.
    pose.position.y = 0.
    pose.position.z = 0.
    pose.orientation.x = 0.
    pose.orientation.y = 0.
    pose.orientation.z = 0.
    pose.orientation.w = 1.
    markers.markers.append(
        gen_marker('base_link', 'object', 0, pose, 'meshes/tubebig.stl'))
    pose_dict = {}
    planning_service = rospy.Service(
        'plan_grasp', Empty, plan_grasps)
    pub = rospy.Publisher('grasp_pub', MarkerArray, queue_size=1)

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(markers)
        base.taskMgr.step()
        update_tfs(br, pose_dict)
        rate.sleep()
