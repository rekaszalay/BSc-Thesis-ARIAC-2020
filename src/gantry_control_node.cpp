#include <gantry_control_node.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

nist_gear::Model pickPart() {
   controller::GetNextModel srv;
   getNextPart_client.call(srv);
   ROS_INFO(srv.response.nextModel.type.c_str());
   srv.response.nextModel.pose = convert_to_frame(srv.response.nextModel.pose, "world", "torso_main");
   return srv.response.nextModel;
}


int main(int argc, char **argv)
{
   //std::string id = argv[1];

   ros::init(argc, argv, "gantry_control_node");
   ros::NodeHandle node;

   ros::AsyncSpinner spinner(4);
   spinner.start();

   getNextPart_client = node.serviceClient<controller::GetNextModel>("/ariac/next_model");
   getNextPart_client.waitForExistence();

  // ArmControl arm(node, id);

   ros::Rate loop_rate(10);

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
   moveit::planning_interface::MoveGroupInterface move_group_ra(PLANNING_GROUP_RIGHT_ARM);
   moveit::planning_interface::MoveGroupInterface move_group_ree(PLANNING_GROUP_RIGHT_EE);
   moveit::planning_interface::MoveGroupInterface move_group_la(PLANNING_GROUP_LEFT_ARM);
   moveit::planning_interface::MoveGroupInterface move_group_lee(PLANNING_GROUP_LEFT_EE);
   moveit::planning_interface::MoveGroupInterface move_group_fr(PLANNING_GROUP_FULL_ROBOT);
   moveit::planning_interface::MoveGroupInterface move_group_g(PLANNING_GROUP_GANTRY);
   
   trajectory_msgs::JointTrajectory gantry_msg;
   //gantry_msg.points.at(0);

   // We will use the :planning_interface:`PlanningSceneInterface`
   // class to add and remove collision objects in our "virtual world" scene
   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
   // Raw pointers are frequently used to refer to the planning group for improved performance.
   joint_model_group_ra = move_group_ra.getCurrentState()->getJointModelGroup(PLANNING_GROUP_RIGHT_ARM);
   joint_model_group_ree = move_group_ree.getCurrentState()->getJointModelGroup(PLANNING_GROUP_RIGHT_EE);
   joint_model_group_fr = move_group_fr.getCurrentState()->getJointModelGroup(PLANNING_GROUP_FULL_ROBOT);
   joint_model_group_lee = move_group_lee.getCurrentState()->getJointModelGroup(PLANNING_GROUP_LEFT_EE);
   joint_model_group_la = move_group_la.getCurrentState()->getJointModelGroup(PLANNING_GROUP_LEFT_ARM);
   joint_model_group_g = move_group_g.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GANTRY);
   // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
   // std::copy(move_group_fr.getJointModelGroupNames().begin(), move_group_fr.getJointModelGroupNames().end(),
   //          std::ostream_iterator<std::string>(std::cout, ", "));
   // move_group_fr.clearPoseTargets();
   // bool link = move_group_fr.setEndEffectorLink("left_ee_link");
   // bool ee = move_group_fr.setEndEffector("left_ee");
   // ROS_INFO(("link: " + std::to_string(link) + " ee: " + std::to_string(ee)).c_str());
   std::vector<double> jointState = move_group_fr.getCurrentJointValues();
   for (double value : jointState){
      //ROS_INFO(std::to_string(value).c_str());
   }
   // jointState[0] = 0.5;
   // geometry_msgs::PoseStamped currentPose = move_group_fr.getCurrentPose();
   // ROS_INFO(("X: "+ std::to_string(currentPose.pose.position.x) +" y: "+ std::to_string(currentPose.pose.position.y) +" Z: "+ std::to_string(currentPose.pose.position.z)).c_str());
   //move_group_fr.setJointValueTarget(jointState);
   // ROS_INFO(move_group_fr.getEndEffectorLink().c_str());
   // ROS_INFO(move_group_ra.getEndEffectorLink().c_str());
   // ROS_INFO(move_group_la.getEndEffectorLink().c_str());
   geometry_msgs::Pose target_pose1;
   target_pose1.position.x = -0.8;
   target_pose1.position.y = 0.26;
   target_pose1.position.z = 1.42;

   const double EPS3 = 0.1;
   std::vector<double> armsup_left = {PI/2, 0-EPS3, PI/2+EPS3, PI/2, 0, 0};
   std::vector<double> armsup_right = {-PI/2, -PI+EPS3, -PI/2-EPS3, PI/2, 0, 0};
   /// NEW here/
   std::vector<double> armsup_left_shelf = {PI/2, -PI/2, 2.7, PI/2, 0, 0};
   std::vector<double> armsup_right_shelf = {-PI/2, -PI/2, -2.7, PI/2, 0, 0};
   ///
   std::vector<double> armsup_left_hanging = {0, -PI/2, PI/2, PI/2, PI/2, 0};
   std::vector<double> armsup_right_hanging = {0, -PI/2, -PI/2, PI/2, -PI/2, 0};

   const double EPS1 = 0.2;
   std::vector<double> handover_before_left = {PI/2, -EPS1, PI/2+EPS1, PI, -PI/2, PI/2};
   std::vector<double> handover_before_right = {-PI/2, -PI+EPS1, -PI/2-EPS1, 0, PI/2, PI/2};
   const double EPS2 = 0.12;
   std::vector<double> handover_left = {PI/2, -EPS2, PI/2+EPS2, PI, -PI/2, PI/2};
   std::vector<double> handover_right = {-PI/2, -PI+EPS2, -PI/2-EPS2, 0, PI/2, PI/2};
   
   //target_pose1 = convert_to_frame(target_pose1, "world", "torso_base");
   // ROS_INFO((std::to_string(target_pose1.position.x) + " " + std::to_string(target_pose1.position.y) + " " +std::to_string(target_pose1.position.z)).c_str());
   //move_group_ra.setPoseTarget(target_pose1);
   // move_group_fr.setGoalTolerance(0.1);
   //move_group_ra.setGoalTolerance(0.1);
   nist_gear::Model nextModel = pickPart();
   std::vector<double> target(15);
   target[0] = nextModel.pose.position.x;
   target[1] = nextModel.pose.position.y;
   target[2] = 0;
   std::copy(armsup_left_shelf.begin(),armsup_left_shelf.end(),target.begin()+3);
   std::copy(armsup_right_shelf.begin(),armsup_right_shelf.end(),target.begin()+3+6);
   move_group_fr.setJointValueTarget(target);
   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
   bool success = (move_group_fr.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
   // move_group_fr.move();
   move_group_fr.move();
   
   //move_group_ra.execute(my_plan);
   spinner.stop();
   ros::shutdown();
   return 0;
}