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

static const std::string PLANNING_GROUP_GANTRY = "Gantry";
static const std::string PLANNING_GROUP_FULL_ROBOT = "Full_Robot";
static const std::string PLANNING_GROUP_RIGHT_ARM = "Right_Arm";
static const std::string PLANNING_GROUP_LEFT_ARM = "Left_Arm";

const double EPS = 0.1;

std::vector<double> left_arm_moving = {PI / 2, 0 - EPS, PI / 2 + EPS, PI / 2, 0, 0};
std::vector<double> right_arm_moving = {-PI / 2, -PI + EPS, -PI / 2 - EPS, PI / 2, 0, 0};

std::vector<double> gantry_shelf_around_1 = {0.6, 2.0, PI/2};
std::vector<double> gantry_shelf_around_2 = {0.6, 5.0, PI};
std::vector<double> gantry_shelf_around_3 = {3.6, 5.0, PI};

std::vector<double> gantry_shelf_right = {3.6, 2.4, 0};
std::vector<double> gantry_shelf_left = {3.6, 4.8, PI};

std::vector<double> left_arm_shelf = {PI / 2, -PI / 2, 2.7, PI / 2, 0, 0};
std::vector<double> right_arm_shelf = {-PI / 2, -PI / 2, -2.7, PI / 2, 0, 0};