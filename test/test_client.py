#! /usr/bin/env python

import unittest

import tf_server

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped

class TestTfClient(unittest.TestCase):
    def test_transform_pose_1(self):
        target_frame = "/top"

        tf_client = tf_server.TFClient()

        ps = PoseStamped()

        ps.header.frame_id = "/bottom"
        ps.header.stamp = rospy.Time()
        # We'll leave the position and orientation at zero
        ps.pose.orientation.w = 1

        tf_client.waitForTransform(target_frame, ps.header.frame_id, rospy.Time(), rospy.Duration(2.0))
        new_pose = tf_client.transformPose(target_frame, ps, timeout=rospy.Time(1.0))

        self.assertNotEqual(new_pose.pose.position.z, ps.pose.position.z)  # These two frames are not at the same Z

    def test_transform_point(self):
        target_frame = "/top"

        tf_client = tf_server.TFClient()

        ps = PointStamped()

        ps.header.frame_id = "/bottom"
        ps.header.stamp = rospy.Time()

        tf_client.waitForTransform(target_frame, ps.header.frame_id, rospy.Time(), rospy.Duration(2.0))
        new_point = tf_client.transformPoint(target_frame, ps, timeout=rospy.Time(1.0))

        self.assertNotEqual(new_point.point.z, ps.point.z)  # These two frames are not at the same Z


if __name__ == '__main__':
    import rostest
    rostest.rosrun("tf_server", 'test_tf_client', TestTfClient)