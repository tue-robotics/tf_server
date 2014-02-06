#! /usr/bin/env python
import roslib; roslib.load_manifest('tf_server')
import rospy

import tf_server.srv

class TFClient:

    def __init__(self, wait_for_service=True):
        if wait_for_service:
            rospy.wait_for_service('tf/lookup_transform')
        self._srv_lookup_transform = rospy.ServiceProxy('tf/lookup_transform', tf_server.srv.LookupTransform)

    def transformPoint(self, target_frame, point):
        pass
        # return point

    def transformPose(self, target_frame, pose):
        pass
        # return pose

    def waitForTransform(self, target_frame, source_frame, time, max_duration):    
        pass

    def lookupTransform(self, target_frame, source_frame, time=rospy.Time(0)):
        req = tf_server.srv.LookupTransformRequest()
        req.target_frame = target_frame
        req.source_frame = source_frame
        req.time = time

        resp = self._srv_lookup_transform(req)

        if resp.error_msg:
            rospy.logerr(resp.error_msg)
        else:
            return resp.transform.transform.translation, resp.transform.transform.rotation

if __name__ == '__main__':
    rospy.init_node('tf_client')
    tf_client = TFClient()
