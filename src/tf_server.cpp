#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <tf_server/LookupTransform.h>
#include <tf_server/TransformPoint.h>
#include <tf_server/TransformPose.h>
#include <tf_server/WaitForTransform.h>

#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <string>

tf::TransformListener* tf_listener_;

bool srvLookupTransform(tf_server::LookupTransform::Request& req, tf_server::LookupTransform::Response& res) {
    for(int i = 0; i < 2; ++i) {
        res.error_msg = "";
        try{
            std::string* error_code;
            bool result = tf_listener_->waitForTransform(req.target_frame,
                                           req.source_frame,
                                           req.target_time,
                                           req.timeout,
                                           ros::Duration(0.01),
                                           error_code);
            if(error_code != NULL) {
                res.error_msg = *error_code;
                return true;
            }

            if( !result){
                res.error_msg = "Transform not found in time";
                return true;
            }

            tf::StampedTransform t;

            if (req.fixed_frame == "") {
                tf_listener_->lookupTransform(req.target_frame, req.source_frame, req.time, t);
            } else {
                tf_listener_->lookupTransform(req.target_frame, req.target_time, req.source_frame, req.source_time, req.fixed_frame, t);
            }
            tf::transformStampedTFToMsg(t, res.transform);
            return true;
        } catch (tf::TransformException& ex){
            res.error_msg = ex.what();
            if (i == 0)
                ROS_WARN_DELAYED_THROTTLE(10, "Transform error (trying again with zero-time): %s", res.error_msg.c_str());
            else
                ROS_ERROR("Transform error at 2nd attempt");

            // set time to 0-time (= get latest transform) and try again
            req.time = ros::Time();
        }
    }

    return true;
}

bool srvTransformPoint(tf_server::TransformPoint::Request& req, tf_server::TransformPoint::Response& res) {
    tf::Stamped<tf::Point> p_in;
    tf::pointStampedMsgToTF(req.point, p_in);

    for(int i = 0; i < 2; ++i) {
        res.error_msg = "";
        try{
            std::string* error_code;
            bool result = tf_listener_->waitForTransform(req.target_frame,
                                           req.fixed_frame,
                                           req.target_time,
                                           req.timeout,
                                           ros::Duration(0.01),
                                           error_code);
            if(error_code != NULL) {
                res.error_msg = *error_code;
                return true;
            }

            if( !result){
                res.error_msg = "Transform not found in time";
                return true;
            }

            tf::Stamped<tf::Point> p_out;

            if (req.fixed_frame == "") {
                tf_listener_->transformPoint(req.target_frame, p_in, p_out);
            } else {
                tf_listener_->transformPoint(req.target_frame, req.target_time, p_in, req.fixed_frame, p_out);
            }
            tf::pointStampedTFToMsg(p_out, res.point);
            return true;
        } catch (tf::TransformException& ex){
            res.error_msg = ex.what();
            if (i == 0)
                ROS_WARN_DELAYED_THROTTLE(10, "Transform error (trying again with zero-time): %s", res.error_msg.c_str());
            else
                ROS_ERROR("Transform error at 2nd attempt");

            // set time to 0-time (= get latest transform) and try again
            p_in.stamp_ = ros::Time();
        }
    }

    return true;
}

bool srvWaitForTransform(tf_server::WaitForTransform::Request& req, tf_server::WaitForTransform::Response& res) {

    res.error_msg = "";
    bool result = false;
    try{

        std::string* error_code;
        result = tf_listener_->waitForTransform(req.target_frame,
                                       req.fixed_frame,
                                       req.target_time,
                                       req.timeout,
                                       ros::Duration(0.01),
                                       error_code);
        if(error_code != NULL) {
            res.error_msg = *error_code;
        }

        if( !result){
            res.error_msg = "Transform not found in time";
        }
        return true;
    } catch (tf::TransformException& ex){
        res.error_msg = ex.what();
        ROS_WARN_DELAYED_THROTTLE(10, "Transform error: %s", res.error_msg.c_str());
    }

    return true;
}

bool srvTransformPose(tf_server::TransformPose::Request& req, tf_server::TransformPose::Response& res) {
    tf::Stamped<tf::Pose> p_in;
    tf::poseStampedMsgToTF(req.pose, p_in);

    for(int i = 0; i < 2; ++i) {
        res.error_msg = "";
        try{
            std::string* error_code;
            bool result = tf_listener_->waitForTransform(req.target_frame,
                                           req.fixed_frame,
                                           req.target_time,
                                           req.timeout,
                                           ros::Duration(0.01),
                                           error_code);
            if(error_code != NULL) {
                res.error_msg = *error_code;
                return true;
            }

            if( !result){
                res.error_msg = "Transform not found in time";
                return true;
            }

            tf::Stamped<tf::Pose> p_out;

            if (req.fixed_frame == "") {
                tf_listener_->transformPose(req.target_frame, p_in, p_out);
            } else {
                tf_listener_->transformPose(req.target_frame, req.target_time, p_in, req.fixed_frame, p_out);
            }
            tf::poseStampedTFToMsg(p_out, res.pose);
            return true;
        } catch (tf::TransformException& ex){
            res.error_msg = ex.what();
            if (i == 0)
                ROS_WARN_DELAYED_THROTTLE(10, "Transform error (trying again with zero-time): %s", res.error_msg.c_str());
            else
                ROS_ERROR("Transform error at 2nd attempt");

            // set time to 0-time (= get latest transform) and try again
            p_in.stamp_ = ros::Time();
        }
    }

    return true;
}

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "tf_server");
    ros::NodeHandle nh;

    tf_listener_ = new tf::TransformListener();

    // advertising a service to provide service
    ros::ServiceServer srv_lookup_transform = nh.advertiseService("/tf/lookup_transform", srvLookupTransform);
    ros::ServiceServer srv_transform_point = nh.advertiseService("/tf/transform_point", srvTransformPoint);
    ros::ServiceServer srv_transform_pose = nh.advertiseService("/tf/transform_pose", srvTransformPose);
    ros::ServiceServer srv_wait_for_transform = nh.advertiseService("/tf/wait_for_transform", srvWaitForTransform);

    // spin
    ros::spin();

    delete tf_listener_;

    return 0;
}

