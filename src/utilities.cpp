#include "utilities.h"

geometry_msgs::Pose convert_to_frame(geometry_msgs::Pose pose, std::string source_frame, std::string target_frame) {
  // ROS_INFO("[common_definitions][convert_to_frame] called");
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
      ROS_INFO_STREAM("[transform from " << source_frame << " to " << target_frame << "] transform not available");
    }
    error = "";
    ROS_INFO_STREAM("[transform from " <<source_frame <<" to "<< target_frame<<"] available: "<<tfBuffer.canTransform(target_frame, source_frame, ros::Time(0), &error) << " error: "<<error); 
    // transformStamped = tfBuffer.lookupTransform("world", "kit_tray_1", ros::Time(0), ros::Duration(3.0));
    transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(), ros::Duration(3.0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  // ROS_INFO_STREAM("[arm_control][pickPart] transformStamped =" << transformStamped);
  tf2::doTransform(pose_rel, pose_target, transformStamped);

  return pose_target.pose;
}