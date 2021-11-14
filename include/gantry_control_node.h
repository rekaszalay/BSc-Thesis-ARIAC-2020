#include "utilities.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <nist_gear/VacuumGripperControl.h>
#include <tf2/LinearMath/Quaternion.h>

ros::ServiceClient getNextPart_client;
ros::ServiceClient left_gripper_control_client, right_gripper_control_client;

ros::Publisher planning_scene_diff_publisher;
ros::ServiceClient planning_scene_diff_client;
ros::ServiceClient planning_scene_get_client;
moveit_msgs::PlanningScene planning_scene_;

static const std::string PLANNING_GROUP_GANTRY = "Gantry";
static const std::string PLANNING_GROUP_FULL_ROBOT = "Full_Robot";
static const std::string PLANNING_GROUP_RIGHT_ARM = "Right_Arm";
static const std::string PLANNING_GROUP_RIGHT_EE = "Right_Endeffector";
static const std::string PLANNING_GROUP_LEFT_ARM = "Left_Arm";
static const std::string PLANNING_GROUP_LEFT_EE = "Left_Endeffector";