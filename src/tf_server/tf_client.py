#! /usr/bin/env python
import roslib; roslib.load_manifest('tf_server')
import rospy

import tf_server.srv

import tf

class TFClient:

    def __init__(self, wait_for_service=True):
        if wait_for_service:
            found = False
            while not found and not rospy.is_shutdown():
                found = True                            
                try:
                    rospy.wait_for_service('/tf/lookup_transform', timeout=1.0)
                    rospy.wait_for_service('/tf/transform_point', timeout=1.0)
                    rospy.wait_for_service('/tf/transform_pose', timeout=1.0)
                    rospy.wait_for_service('/tf/wait_for_transform', timeout=1.0)
                except rospy.ROSException:
                    found = False
                    rospy.loginfo("Waiting for tf server")                    

        self._srv_lookup_transform = rospy.ServiceProxy('/tf/lookup_transform', tf_server.srv.LookupTransform)
        self._srv_transform_point = rospy.ServiceProxy('/tf/transform_point', tf_server.srv.TransformPoint)
        self._srv_transform_pose = rospy.ServiceProxy('/tf/transform_pose', tf_server.srv.TransformPose)
        self._srv_wait_for_transform = rospy.ServiceProxy('/tf/wait_for_transform', tf_server.srv.WaitForTransform)

    def transformPoint(self, target_frame, point, target_time=rospy.Time(0), fixed_frame='', timeout=rospy.Time(10)):
        req = tf_server.srv.TransformPointRequest()
        req.point = point
        req.target_frame = target_frame
        req.target_time = target_time
        req.fixed_frame = fixed_frame if fixed_frame else point.header.frame_id
        req.timeout = timeout

        resp = self._srv_transform_point(req)
        
        if resp.error_msg:
            raise tf.Exception(resp.error_msg)
            rospy.logerr(resp.error_msg)
        else:
            return resp.point

    def transformPose(self, target_frame, pose, target_time=rospy.Time(0), fixed_frame='', timeout=rospy.Time(10)):
        req = tf_server.srv.TransformPoseRequest()
        req.pose = pose
        req.target_frame = target_frame
        req.target_time = target_time
        req.fixed_frame = fixed_frame if fixed_frame else pose.header.frame_id
        req.timeout = timeout

        resp = self._srv_transform_pose(req)
        
        if resp.error_msg:
            raise tf.Exception(resp.error_msg)
            rospy.logerr(resp.error_msg)
        else:
            return resp.pose

    def waitForTransform(self, target_frame, source_frame, time=rospy.Time(0), timeout=rospy.Time(10)):
        req = tf_server.srv.WaitForTransformRequest()
        req.target_frame = target_frame
        req.fixed_frame = source_frame
        req.target_time = time
        req.timeout = timeout

        resp = self._srv_wait_for_transform(req)

        if resp.error_msg:
            rospy.logerr(resp.error_msg)
            return False
        else:
            return True

    def lookupTransform(self, target_frame, source_frame, time=rospy.Time(0), timeout=rospy.Time(10)):
        req = tf_server.srv.LookupTransformRequest()
        req.target_frame = target_frame
        req.source_frame = source_frame
        req.time = time
        req.timeout = timeout

        resp = self._srv_lookup_transform(req)

        if resp.error_msg:
            raise tf.Exception(resp.error_msg)
            rospy.logerr(resp.error_msg)
        else:
            return resp.transform

if __name__ == '__main__':
    rospy.init_node('tf_client')
    tf_client = TFClient()
