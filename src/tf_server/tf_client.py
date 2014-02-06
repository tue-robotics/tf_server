#! /usr/bin/env python
import roslib; roslib.load_manifest('tf_server')
import rospy

import tf_server.srv

if __name__ == '__main__':
    rospy.init_node('tf_client')

    rospy.wait_for_service('tf/lookup_transform')
    srv_lookup_transform = rospy.ServiceProxy('tf/lookup_transform', tf_server.srv.LookupTransform)
    
    req = tf_server.srv.LookupTransformRequest()
    req.target_frame = "/amigo/base_link"
    req.source_frame = "/amigo/wrist_link6_right"

    resp = srv_lookup_transform(req)

    print resp
