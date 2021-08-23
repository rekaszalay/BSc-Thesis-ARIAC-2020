#include "ros/ros.h"
//#include "arm_control.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
   //std::string id = argv[1];

   ros::init(argc, argv, "gantry_control_node");
   ros::NodeHandle node;

   ros::AsyncSpinner spinner(4);
   spinner.start();

  // ArmControl arm(node, id);

   ros::Rate loop_rate(10);
   ROS_INFO("Megy a gantry node :D");

   // while (ros::ok()) {

   //    ros::Duration(5.0).sleep();

   //    loop_rate.sleep();
   // }

   while (!node.hasParam("robot_description")) {ros::Duration(1.0).sleep(); ROS_INFO("waiting...");}
   //ros::spin();  // This executes callbacks on new data until ctrl-c.
   
   /**/robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
   const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
   ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
   moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
   kinematic_state->setToDefaultValues();
   const moveit::core::JointModelGroup* joint_model_group_g = kinematic_model->getJointModelGroup("Gantry");
   const moveit::core::JointModelGroup* joint_model_group_fr = kinematic_model->getJointModelGroup("Full_Robot");
   const moveit::core::JointModelGroup* joint_model_group_ra = kinematic_model->getJointModelGroup("Right_Arm");
   const moveit::core::JointModelGroup* joint_model_group_la = kinematic_model->getJointModelGroup("Left_Arm");
   const moveit::core::JointModelGroup* joint_model_group_ree = kinematic_model->getJointModelGroup("Right_Endeffector");
   const moveit::core::JointModelGroup* joint_model_group_lee = kinematic_model->getJointModelGroup("Left_Endeffector");
   const std::vector<std::string>& joint_names_g = joint_model_group_g->getVariableNames();
   const std::vector<std::string>& joint_names_fr = joint_model_group_fr->getVariableNames();
   const std::vector<std::string>& joint_names_ra = joint_model_group_ra->getVariableNames();
   const std::vector<std::string>& joint_names_la = joint_model_group_la->getVariableNames();
   const std::vector<std::string>& joint_names_ree = joint_model_group_ree->getVariableNames();
   const std::vector<std::string>& joint_names_lee = joint_model_group_lee->getVariableNames();
   // BEGIN_TUTORIAL
   //
   // Setup
   // ^^^^^
   //
   // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
   // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
   // are used interchangably.
   // ARIAC/gantry_moveit_config/config/gantry.srdf
   static const std::string PLANNING_GROUP_GANTRY = "Gantry";
   static const std::string PLANNING_GROUP_FULL_ROBOT = "Full_Robot";
   static const std::string PLANNING_GROUP_RIGHT_ARM = "Right_Arm";
   static const std::string PLANNING_GROUP_RIGHT_EE = "Right_Endeffector";
   static const std::string PLANNING_GROUP_LEFT_ARM = "Left_Arm";
   static const std::string PLANNING_GROUP_LEFT_EE = "Left_Endeffector";
   // The :planning_interface:`MoveGroupInterface` class can be easily
   // setup using just the name of the planning group you would like to control and plan for.
   ROS_INFO("It's still OK here.");
   moveit::planning_interface::MoveGroupInterface move_group_ra(PLANNING_GROUP_RIGHT_ARM);
   moveit::planning_interface::MoveGroupInterface move_group_la(PLANNING_GROUP_LEFT_ARM);
   moveit::planning_interface::MoveGroupInterface move_group_fr(PLANNING_GROUP_FULL_ROBOT);
   moveit::planning_interface::MoveGroupInterface move_group_g(PLANNING_GROUP_GANTRY);
   // We will use the :planning_interface:`PlanningSceneInterface`
   // class to add and remove collision objects in our "virtual world" scene
   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
   // Raw pointers are frequently used to refer to the planning group for improved performance.
   joint_model_group_ra = move_group_ra.getCurrentState()->getJointModelGroup(PLANNING_GROUP_RIGHT_ARM);
   joint_model_group_fr = move_group_fr.getCurrentState()->getJointModelGroup(PLANNING_GROUP_FULL_ROBOT);
   joint_model_group_la = move_group_la.getCurrentState()->getJointModelGroup(PLANNING_GROUP_LEFT_ARM);
   joint_model_group_g = move_group_g.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GANTRY);
   ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
   std::copy(move_group_ra.getJointModelGroupNames().begin(), move_group_ra.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
   geometry_msgs::Pose target_pose1;
   target_pose1.position.x = -0.8;
   target_pose1.position.y = 0.06;
   target_pose1.position.z = 1.42;
   move_group_ra.setPoseTarget(target_pose1);
   move_group_ra.setGoalTolerance(0.01);
   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
   bool success = (move_group_ra.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
   move_group_ra.move();
   spinner.stop();
   ros::shutdown();
   return 0;
}