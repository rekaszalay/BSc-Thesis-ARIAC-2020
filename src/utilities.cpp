#include "utilities.h"

geometry_msgs::Pose convert_to_frame(geometry_msgs::Pose pose, std::string source_frame, std::string target_frame) {
  geometry_msgs::Pose nextTarget;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PoseStamped pose_target, pose_rel;

  pose_rel.header.frame_id = source_frame;
  pose_rel.pose = pose;

  try {
    std::string error = "";
    while (!tfBuffer.canTransform(target_frame, source_frame, ros::Time(), ros::Duration(4.0), &error))
    {
      ros::Duration(1.0).sleep();
    }
    error = "";
    transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(), ros::Duration(3.0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  tf2::doTransform(pose_rel, pose_target, transformStamped);

  return pose_target.pose;
}