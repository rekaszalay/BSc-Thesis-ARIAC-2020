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
ros::ServiceClient getNextPlacePosition_client;

ros::Publisher planning_scene_diff_publisher;
ros::ServiceClient planning_scene_diff_client;
ros::ServiceClient planning_scene_get_client;
moveit_msgs::PlanningScene planning_scene_;

nist_gear::VacuumGripperState rightVGS, leftVGS;

static const std::string PLANNING_GROUP_GANTRY = "Gantry";
static const std::string PLANNING_GROUP_FULL_ROBOT = "Full_Robot";
static const std::string PLANNING_GROUP_RIGHT_ARM = "Right_Arm";
static const std::string PLANNING_GROUP_RIGHT_EE = "Right_Endeffector";
static const std::string PLANNING_GROUP_LEFT_ARM = "Left_Arm";
static const std::string PLANNING_GROUP_LEFT_EE = "Left_Endeffector";

const double EPS3 = 0.1;
std::vector<double> armsup_left = {PI / 2, 0 - EPS3, PI / 2 + EPS3, PI / 2, 0, 0};
std::vector<double> armsup_right = {-PI / 2, -PI + EPS3, -PI / 2 - EPS3, PI / 2, 0, 0};

std::vector<double> gantry_bin = {2.9, 0.7, 0};
std::vector<double> gantry_shelf_right = {3.6, 2.4, 0};
std::vector<double> gantry_shelf_left = {0, 0, PI};

std::vector<double> armsup_left_shelf = {PI / 2, -PI / 2, 2.7, PI / 2, 0, 0};
std::vector<double> armsup_right_shelf = {-PI / 2, -PI / 2, -2.7, PI / 2, 0, 0};

std::vector<double> armsup_left_hanging = {0, -PI / 2, PI / 2, PI / 2, PI / 2, 0};
std::vector<double> armsup_right_hanging = {0, -PI / 2, -PI / 2, PI / 2, -PI / 2, 0};

const double EPS1 = 0.2;
std::vector<double> handover_before_left = {PI / 2, -EPS1, PI / 2 + EPS1, PI, -PI / 2, PI / 2};
std::vector<double> handover_before_right = {-PI / 2, -PI + EPS1, -PI / 2 - EPS1, 0, PI / 2, PI / 2};
const double EPS2 = 0.12;
std::vector<double> handover_left = {PI / 2, -EPS2, PI / 2 + EPS2, PI, -PI / 2, PI / 2};
std::vector<double> handover_right = {-PI / 2, -PI + EPS2, -PI / 2 - EPS2, 0, PI / 2, PI / 2};

