#! /usr/bin/env python

import unittest

import tf_server

import rospy
from geometry_msgs.msg import PoseStamped

class TestBasics(unittest.TestCase):
    def test_transform_pose_1(self):
        target_frame = "/amigo/neck_tilt"

        tf_client = tf_server.TFClient()

        ps = PoseStamped()

        ps.header.frame_id = "/amigo/head_mount"
        ps.header.stamp = rospy.Time()
        # We'll leave the position and orientation at zero
        ps.pose.orientation.w = 1

        tf_client.waitForTransform(target_frame, ps.header.frame_id, rospy.Time(), rospy.Duration(2.0))
        new_pose = tf_client.transformPose(target_frame, ps, timeout=rospy.Time(0.5))

        self.assertNotEqual(new_pose.pose.position.z, ps.pose.position.z)  # These two frames are not at the same Z


if __name__ == "__main__":
    rospy.init_node("test_tf_client")
    unittest.main()