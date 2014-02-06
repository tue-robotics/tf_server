#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <tf_server/LookupTransform.h>

#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

tf::TransformListener* tf_listener_;

bool srvLookupTransform(tf_server::LookupTransform::Request& req, tf_server::LookupTransform::Response& res) {
    tf::StampedTransform t;

    try{
        if (req.fixed_frame == "") {
            tf_listener_->lookupTransform(req.target_frame, req.source_frame, req.time, t);
        } else {
            tf_listener_->lookupTransform(req.target_frame, req.target_time, req.source_frame, req.source_time, req.fixed_frame, t);
        }
        tf::transformStampedTFToMsg(t, res.transform);
    } catch (tf::TransformException& ex){
        res.error_msg = ex.what();        
    }

    return true;
}


int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "tf_server");
    ros::NodeHandle nh;

    tf_listener_ = new tf::TransformListener();

    // advertising a service to provide service
    ros::ServiceServer service = nh.advertiseService("/tf/lookup_transform", srvLookupTransform);

    // spin
    ros::spin();

    delete tf_listener_;

    return 0;
}

