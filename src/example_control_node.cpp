// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// %Tag(FULLTEXT)%
// %Tag(INCLUDE_STATEMENTS)%
#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
// %EndTag(INCLUDE_STATEMENTS)%

// %Tag(START_COMP)%
/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) {
   // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
   ros::ServiceClient start_client =
   node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
   // If it's not already ready, wait for it to be ready.
   // Calling the Service using the client before the server is ready would fail.
   if (!start_client.exists()) {
      ROS_INFO("Waiting for the competition to be ready...");
      start_client.waitForExistence();
      ROS_INFO("Competition is now ready.");
   }
   ROS_INFO("Requesting competition start...");
   std_srvs::Trigger srv;  // Combination of the "request" and the "response".
   start_client.call(srv);  // Call the start Service.
   if (!srv.response.success) {  // If not successful, print out why.
      ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
   } else {
      ROS_INFO("Competition started!");
   }
}
// %EndTag(START_COMP)%

/// Example class that can hold state and provide methods that handle incoming data.
class MyCompetitionClass
{
public:
    explicit MyCompetitionClass(ros::NodeHandle & node)
    : current_score_(0), right_arm_has_been_zeroed_(false), left_arm_has_been_zeroed_(false)
    {
       // %Tag(ADV_CMD)%
       right_arm_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
       "/ariac/gantry/right_arm_controller/command", 10);
       
       left_arm_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
       "/ariac/gantry/left_arm_controller/command", 10);
       // %EndTag(ADV_CMD)%
    }
    
    /// Called when a new message is received.
    void current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
       if (msg->data != current_score_)
       {
          ROS_INFO_STREAM("Score: " << msg->data);
       }
       current_score_ = msg->data;
    }
    
    /// Called when a new message is received.
    void competition_state_callback(const std_msgs::String::ConstPtr & msg) {
       if (msg->data == "done" && competition_state_ != "done")
       {
          ROS_INFO("Competition ended.");
       }
       competition_state_ = msg->data;
    }
    
    /// Called when a new Order message is received.
    void order_callback(const nist_gear::Order::ConstPtr & order_msg) {
       //ROS_INFO_STREAM("Received order:\n" << *order_msg);
       received_orders_.push_back(*order_msg);
    }
    
    // %Tag(CB_CLASS)%
    /// Called when a new JointState message is received.
    void right_arm_joint_state_callback(
    const control_msgs::JointTrajectoryControllerState::ConstPtr & joint_state_msg)
    {
       //ROS_INFO_STREAM_THROTTLE(10,
       //                         "Joint States right arm (throttled to 0.1 Hz):\n" << *joint_state_msg);
       //ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
       right_arm_current_joint_states_ = *joint_state_msg;
       if (!right_arm_has_been_zeroed_) {
          right_arm_has_been_zeroed_ = true;
          ROS_INFO("Sending right arm to zero joint positions...");
          //send_arm_to_zero_state(right_arm_joint_trajectory_publisher_, 1);
       }
    }
    
    void left_arm_joint_state_callback(
    const control_msgs::JointTrajectoryControllerState::ConstPtr & joint_state_msg)
    {
       //ROS_INFO_STREAM_THROTTLE(10,
       //                         "Joint States left arm (throttled to 0.1 Hz):\n" << *joint_state_msg);
       //ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
       left_arm_current_joint_states_ = *joint_state_msg;
       if (!left_arm_has_been_zeroed_) {
          left_arm_has_been_zeroed_ = true;
          ROS_INFO("Sending left arm to zero joint positions...");
          //send_arm_to_zero_state(left_arm_joint_trajectory_publisher_, 2);
       }
    }
    
    void gantry_joint_state_callback (
    const sensor_msgs::JointState::ConstPtr & joint_state_msg)
    {
       //ROS_INFO_STREAM_THROTTLE(10,
       //                         "Joint States right arm (throttled to 0.1 Hz):\n" << *joint_state_msg);
       //ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
       gantry_current_joint_states_ = *joint_state_msg;
       if (!gantry_has_been_zeroed_) {
          gantry_has_been_zeroed_ = true;
          ROS_INFO("Sending gantry to zero joint positions...");
          //send_gantry_to_zero_state(right_arm_joint_trajectory_publisher_);
       }
    }
    // %EndTag(CB_CLASS)%
    
    // %Tag(ARM_ZERO)%
    /// Create a JointTrajectory with all positions set to zero, and command the arm.
    void send_arm_to_zero_state(ros::Publisher & joint_trajectory_publisher, int arm_no) {
       // Create a message to send.
       trajectory_msgs::JointTrajectory msg;
       std::string arm = (arm_no == 1) ? "right_" : "left_";
       // Fill the names of the joints to be controlled.
       // Note that the vacuum_gripper_joint is not controllable.
       msg.joint_names.clear();
       msg.joint_names.push_back(arm + "shoulder_pan_joint");
       msg.joint_names.push_back(arm + "shoulder_lift_joint");
       msg.joint_names.push_back(arm + "elbow_joint");
       msg.joint_names.push_back(arm + "wrist_1_joint");
       msg.joint_names.push_back(arm + "wrist_2_joint");
       msg.joint_names.push_back(arm + "wrist_3_joint");
       //msg.joint_names.push_back("linear_arm_actuator_joint");
       // Create one point in the trajectory.
       msg.points.resize(1);
       // Resize the vector to the same length as the joint names.
       // Values are initialized to 0.
       msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
       // How long to take getting to the point (floating point seconds).
       msg.points[0].time_from_start = ros::Duration(0.001);
       ROS_INFO_STREAM("Sending command:\n" << msg);
       //joint_trajectory_publisher.publish(msg);
    }
    // %EndTag(ARM_ZERO)%
    
    /// Called when a new LogicalCameraImage message is received.
    void logical_camera_callback(
    const nist_gear::LogicalCameraImage::ConstPtr & image_msg)
    {
       ROS_INFO_STREAM_THROTTLE(10,
                                "Logical camera: '" << image_msg->models.size() << "' objects.");
    }
    
    /// Called when a new Proximity message is received.
    void break_beam_callback(const nist_gear::Proximity::ConstPtr & msg) {
       if (msg->object_detected) {  // If there is an object in proximity.
          ROS_INFO("Break beam triggered.");
       }
    }

private:
    std::string competition_state_;
    double current_score_;
    ros::Publisher right_arm_joint_trajectory_publisher_;
    ros::Publisher left_arm_joint_trajectory_publisher_;
    std::vector<nist_gear::Order> received_orders_;
    sensor_msgs::JointState gantry_current_joint_states_;
    control_msgs::JointTrajectoryControllerState right_arm_current_joint_states_;
    control_msgs::JointTrajectoryControllerState left_arm_current_joint_states_;
    bool right_arm_has_been_zeroed_;
    bool left_arm_has_been_zeroed_;
    bool gantry_has_been_zeroed_;
};

void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr & msg) {
   if ((msg->max_range - msg->range) > 0.01) {  // If there is an object in proximity.
      ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
   }
}

void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
   size_t number_of_valid_ranges = std::count_if(
   msg->ranges.begin(), msg->ranges.end(), [](const float f) {return std::isfinite(f);});
   if (number_of_valid_ranges > 0) {
      ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
   }
}

// %Tag(MAIN)%
int main(int argc, char ** argv) {
   // Last argument is the default name of the node.
   ros::init(argc, argv, "example_control_node");
   
   ros::NodeHandle node;
   
   // Instance of custom class from above.
   MyCompetitionClass comp_class(node);
   
   // Subscribe to the '/ariac/current_score' topic.
   ros::Subscriber current_score_subscriber = node.subscribe(
   "/ariac/current_score", 10,
   &MyCompetitionClass::current_score_callback, &comp_class);
   
   // Subscribe to the '/ariac/competition_state' topic.
   ros::Subscriber competition_state_subscriber = node.subscribe(
   "/ariac/competition_state", 10,
   &MyCompetitionClass::competition_state_callback, &comp_class);
   
   // %Tag(SUB_CLASS)%
   // Subscribe to the '/ariac/orders' topic.
   ros::Subscriber orders_subscriber = node.subscribe(
   "/ariac/orders", 10,
   &MyCompetitionClass::order_callback, &comp_class);
   // %EndTag(SUB_CLASS)%
   
   ros::Subscriber gantry_joint_state_subscriber = node.subscribe(
   "/ariac/gantry/joint_states", 10,
   &MyCompetitionClass::gantry_joint_state_callback, &comp_class);
   
   // Subscribe to the '/ariac/joint_states' topic.
   ros::Subscriber right_arm_joint_state_subscriber = node.subscribe(
   "/ariac/gantry/right_arm_controller/state", 10,
   &MyCompetitionClass::right_arm_joint_state_callback, &comp_class);
   
   ros::Subscriber left_arm_joint_state_subscriber = node.subscribe(
   "/ariac/gantry/left_arm_controller/state", 10,
   &MyCompetitionClass::left_arm_joint_state_callback, &comp_class);
   
   // %Tag(SUB_FUNC)%
   // Subscribe to the '/ariac/proximity_sensor_1' topic.
   ros::Subscriber proximity_sensor_subscriber = node.subscribe(
   "/ariac/proximity_sensor_1", 10, proximity_sensor_callback);
   // %EndTag(SUB_FUNC)%
   
   // Subscribe to the '/ariac/break_beam_1_change' topic.
   ros::Subscriber break_beam_subscriber = node.subscribe(
   "/ariac/break_beam_1_change", 10,
   &MyCompetitionClass::break_beam_callback, &comp_class);
   
   // Subscribe to the '/ariac/logical_camera_1' topic.
   ros::Subscriber logical_camera_subscriber = node.subscribe(
   "/ariac/logical_camera_1", 10,
   &MyCompetitionClass::logical_camera_callback, &comp_class);
   
   // Subscribe to the '/ariac/laser_profiler_1' topic.
   ros::Subscriber laser_profiler_subscriber = node.subscribe(
   "/ariac/laser_profiler_1", 10, laser_profiler_callback);
   
   ROS_INFO("Setup complete.");
   start_competition(node);
   ros::AsyncSpinner spinner(1);
   spinner.start();
//    ROS_INFO_NAMED("The robot description ", node.hasParam("ariac/gantry/robot_description") ? "exists." : "doesn't exist");
//    //ros::spin();  // This executes callbacks on new data until ctrl-c.
//    ROS_INFO(node.getNamespace().c_str());
//    ros::V_string names;
//    node.getParamNames(names);
//    for(std::string name : names) ROS_INFO(name.c_str());

//    /**/robot_model_loader::RobotModelLoader robot_model_loader("ariac/gantry/robot_description");
//    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
//    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
   
//    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
//    kinematic_state->setToDefaultValues();
//    const moveit::core::JointModelGroup* joint_model_group_g = kinematic_model->getJointModelGroup("Gantry");
//    const moveit::core::JointModelGroup* joint_model_group_fr = kinematic_model->getJointModelGroup("Full_Robot");
//    const moveit::core::JointModelGroup* joint_model_group_ra = kinematic_model->getJointModelGroup("Right_Arm");
//    const moveit::core::JointModelGroup* joint_model_group_la = kinematic_model->getJointModelGroup("Left_Arm");
//    const moveit::core::JointModelGroup* joint_model_group_ree = kinematic_model->getJointModelGroup("Right_Endeffector");
//    const moveit::core::JointModelGroup* joint_model_group_lee = kinematic_model->getJointModelGroup("Left_Endeffector");
   
//    const std::vector<std::string>& joint_names_g = joint_model_group_g->getVariableNames();
//    const std::vector<std::string>& joint_names_fr = joint_model_group_fr->getVariableNames();
//    const std::vector<std::string>& joint_names_ra = joint_model_group_ra->getVariableNames();
//    const std::vector<std::string>& joint_names_la = joint_model_group_la->getVariableNames();
//    const std::vector<std::string>& joint_names_ree = joint_model_group_ree->getVariableNames();
//    const std::vector<std::string>& joint_names_lee = joint_model_group_lee->getVariableNames();
   
   
//    // BEGIN_TUTORIAL
//    //
//    // Setup
//    // ^^^^^
//    //
//    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
//    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
//    // are used interchangably.
//    // ARIAC/gantry_moveit_config/config/gantry.srdf
//    static const std::string PLANNING_GROUP_GANTRY = "Gantry";
//    static const std::string PLANNING_GROUP_FULL_ROBOT = "Full_Robot";
//    static const std::string PLANNING_GROUP_RIGHT_ARM = "Right_Arm";
//    static const std::string PLANNING_GROUP_RIGHT_EE = "Right_Endeffector";
//    static const std::string PLANNING_GROUP_LEFT_ARM = "Left_Arm";
//    static const std::string PLANNING_GROUP_LEFT_EE = "Left_Endeffector";
   
   
//    // The :planning_interface:`MoveGroupInterface` class can be easily
//    // setup using just the name of the planning group you would like to control and plan for.
//    ROS_INFO("It's still OK here.");
//    moveit::planning_interface::MoveGroupInterface move_group_ra(PLANNING_GROUP_RIGHT_ARM);
// //   moveit::planning_interface::MoveGroupInterface move_group_la(PLANNING_GROUP_LEFT_ARM);
// //   moveit::planning_interface::MoveGroupInterface move_group_fr(PLANNING_GROUP_FULL_ROBOT);
// //   moveit::planning_interface::MoveGroupInterface move_group_g(PLANNING_GROUP_GANTRY);
   
//    // We will use the :planning_interface:`PlanningSceneInterface`
//    // class to add and remove collision objects in our "virtual world" scene
//    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
   
//    // Raw pointers are frequently used to refer to the planning group for improved performance.
//    joint_model_group_ra = move_group_ra.getCurrentState()->getJointModelGroup(PLANNING_GROUP_RIGHT_ARM);
// //   joint_model_group_fr = move_group_fr.getCurrentState()->getJointModelGroup(PLANNING_GROUP_FULL_ROBOT);
// //   joint_model_group_la = move_group_la.getCurrentState()->getJointModelGroup(PLANNING_GROUP_LEFT_ARM);
// //   joint_model_group_g = move_group_g.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GANTRY);
   
//    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
//    std::copy(move_group_ra.getJointModelGroupNames().begin(), move_group_ra.getJointModelGroupNames().end(),
//              std::ostream_iterator<std::string>(std::cout, ", "));
   
//    geometry_msgs::Pose target_pose1;
//    target_pose1.orientation.w = 1.0;
//    target_pose1.position.x = 0.28;
//    target_pose1.position.y = -0.2;
//    target_pose1.position.z = 0.5;
//    move_group_ra.setPoseTarget(target_pose1);
   
//    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
   
//    bool success = (move_group_ra.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   
//    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
   
//    move_group_ra.move();
   
   return 0;
}
// %EndTag(MAIN)%
// %EndTag(FULLTEXT)%
