#include <string>

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

std::string model_name;
nav_msgs::Path robotPath;
ros::Publisher pubGtPath;

void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    int modelIndex = -1;
    for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == model_name) {
            modelIndex = i;
            break;
        }
    }
    if (modelIndex == -1) {
        ROS_ERROR("Model name \"%s\" does not exist!", model_name.c_str());
        return;
    }

    geometry_msgs::PoseStamped currPose;
    currPose.header.frame_id = "map";
    // Note gazebo_msgs::ModelStates does not have time stamp field
    // Use ros::Time::now() will cause small delay w.r.t. the true value
    currPose.header.stamp = ros::Time::now();
    currPose.pose.position.x = msg->pose[modelIndex].position.x;
    currPose.pose.position.y = msg->pose[modelIndex].position.y;
    currPose.pose.position.z = msg->pose[modelIndex].position.z;
    currPose.pose.orientation.x = msg->pose[modelIndex].orientation.x;
    currPose.pose.orientation.y = msg->pose[modelIndex].orientation.y;
    currPose.pose.orientation.z = msg->pose[modelIndex].orientation.z;
    currPose.pose.orientation.w = msg->pose[modelIndex].orientation.w;

    robotPath.poses.push_back(currPose);
    pubGtPath.publish(robotPath);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "gt_gazebo");
    ros::NodeHandle nh;

    // Get model name
    if (nh.getParam("model_name", model_name)) {
        ROS_INFO("Subscribe pose of %d from gazebo.", model_name.c_str());
    } else {
        ROS_WARN("No model name specified. Set as the default value \"robot\".");
        model_name = "robot";
    }

    robotPath.header.frame_id = "map";
    robotPath.header.stamp = ros::Time::now();

    ros::Subscriber subGazeboState = nh.subscribe("gazebo/model_states", 10, modelStateCallback);
    pubGtPath = nh.advertise<nav_msgs::Path>("/path_gt", 5);

    ros::spin();

    return 0;
}

