#include <gantry_control_node.h>

class GantryControl
{
public:
      GantryControl(ros::NodeHandle &node)
      {
            left_gripper_control_client = node.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/left_arm/gripper/control");
            left_gripper_control_client.waitForExistence();

            right_gripper_control_client = node.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/right_arm/gripper/control");
            right_gripper_control_client.waitForExistence();

            getNextPart_client = node.serviceClient<controller::GetNextModel>("/ariac/next_model");
            getNextPart_client.waitForExistence();

            getNextPlacePosition_client = node.serviceClient<controller::GetPlacePosition>("/ariac/next_place_position");
            getNextPlacePosition_client.waitForExistence();

            right_vacuum_gripper_state_subscriber = node.subscribe("/ariac/gantry/right_arm/gripper/state", 10, &GantryControl::right_arm_gripper_state_callback, this);
            left_vacuum_gripper_state_subscriber = node.subscribe("/ariac/gantry/left_arm/gripper/state", 10, &GantryControl::left_arm_gripper_state_callback, this);

            gripperDown.x=0.0;
            gripperDown.y=0.707;
            gripperDown.z=0.0;
            gripperDown.w=0.707;
      }


      nist_gear::Model pickPart()
      {
            controller::GetNextModel srv;
            getNextPart_client.call(srv);
            return srv.response.nextModel;
      }

      geometry_msgs::Pose getPlacePosition(std::string type)
      {
            controller::GetPlacePosition srv;
            srv.request.type = type;
            getNextPlacePosition_client.call(srv);
            return srv.response.position;
      }

      void gripperControl(std::string arm, bool onOff)
      {
            nist_gear::VacuumGripperControl srv;
            srv.request.enable = onOff;

            if (arm == "left")
            {
                  left_gripper_control_client.call(srv);
            }
            else
            {
                  right_gripper_control_client.call(srv);
            }
            ROS_INFO_STREAM("[GantryControl][activateGripper " << onOff << "] DEBUG: srv.response =" << srv.response);
      }

      void grabPart(std::string arm, geometry_msgs::Point targetPosition) {
            moveit::planning_interface::MoveGroupInterface *moveGroup;
            if (arm == "left") moveGroup = &move_group_la;
            else if (arm == "right") moveGroup = &move_group_ra;
            else {ROS_INFO_STREAM("[grabPart] Invalid arm ID!"); return;}
            std::vector<geometry_msgs::Pose> waypoints;
            geometry_msgs::Pose waypoint;
            waypoint.position = targetPosition;
            waypoint.position.z += 0.3;
            waypoint.orientation = gripperDown;
            waypoints.push_back(waypoint);
            // ROS_INFO_STREAM("[grabPart] waypoint " << waypoints.size() <<": " << waypoint);
            waypoint.position.z -= 0.105;
            waypoints.push_back(waypoint);
            // ROS_INFO_STREAM("[grabPart] waypoint " << waypoints.size() <<": " << waypoint);
            waypoint.position.z -= 0.1;
            waypoints.push_back(waypoint);
            // waypoint.position.z += 0.15;
            // waypoints.push_back(waypoint);
            // ROS_INFO_STREAM("[grabPart] waypoint " << waypoints.size() <<": " << waypoint);

            moveGroup->execute(trajectoryFromWaypoints(*moveGroup, waypoints));
      }

      moveit_msgs::RobotTrajectory trajectoryFromWaypoints(moveit::planning_interface::MoveGroupInterface &moveGroup, std::vector<geometry_msgs::Pose> waypoints){
            moveit_msgs::Constraints cons;
            moveit_msgs::OrientationConstraint ocons;
            ocons.header = moveGroup.getCurrentPose().header;
            ocons.link_name = moveGroup.getEndEffectorLink();
            ocons.orientation = moveGroup.getCurrentPose().pose.orientation;
            ocons.absolute_x_axis_tolerance = 0.5;
            ocons.absolute_y_axis_tolerance = 0.5;
            ocons.absolute_z_axis_tolerance = 0.5;
            ocons.weight = 1;
            cons.orientation_constraints = {ocons};
            moveGroup.setPathConstraints(cons);
            const double jump_threshold = 0.0;
            const double eef_step = 0.15;
            moveit_msgs::RobotTrajectory trajectory;
            double fraction = moveGroup.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, 1);
            ROS_INFO_STREAM("[Cartesian path][trajectoryFromWaypoint] fraction = " << fraction);
            int attempts = 0;
            while (std::abs(fraction - 1.0f) > 0.01 && attempts < 5)
            {
                  ROS_INFO("[trajectoryFromWaypoint] Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
                  fraction = moveGroup.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, 1);
                  attempts++;
            }
            trajectory.joint_trajectory.header.stamp = ros::Time::now();
            trajectory.multi_dof_joint_trajectory.header.stamp = ros::Time::now();
            return trajectory;
      }


      void withdrawArm(std::string armID, bool afterGrab = true) {
            nist_gear::VacuumGripperState *gripperState;
            moveit::planning_interface::MoveGroupInterface *moveGroup;
            if (armID == "left") {
                  moveGroup = &move_group_la;
                  gripperState = &leftVGS;
            }
            else if (armID == "right")
            {
                  moveGroup = &move_group_ra;
                  gripperState = &rightVGS;
            }
            else
            {
                  ROS_INFO_STREAM("[withdrawArm] Invalid arm ID!");
                  return;
            }
            geometry_msgs::Pose waypoint(moveGroup->getCurrentPose().pose);
            waypoint.position.z +=0.2;
            if (afterGrab) {
                  while (!gripperState->attached) {
                        ROS_INFO_STREAM("Apple not attached. Trying again...");
                        std::vector<geometry_msgs::Pose> waypoints;
                        waypoints.push_back(waypoint);
                        waypoint.position.z -=0.2;
                        waypoints.push_back(waypoint);
                        moveGroup->execute(trajectoryFromWaypoints(*moveGroup, waypoints));
                        waypoint.position.z +=0.2;
                        ros::Duration(1).sleep();
                  }
            }
            moveGroup->execute(trajectoryFromWaypoints(*moveGroup, {waypoint}));
      }

      void right_arm_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr &state)
      {
            rightVGS = *state;
      }

      void left_arm_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr &state)
      {
            leftVGS = *state;
      }

      void move_full_robot(std::vector<double> gantry_state, std::vector<double> right_arm_state, std::vector<double> left_arm_state)
      {
            std::vector<double> target(15), currentJointValues(move_group_fr.getCurrentJointValues());
            std::copy(currentJointValues.begin(), currentJointValues.end(), target.begin());
            if (!gantry_state.empty())
            {
                  gantry_state[1] *= -1;
                  std::copy(gantry_state.begin(), gantry_state.end(), target.begin());
                  ROS_INFO_STREAM("Setting Gantry joint targets");
            }
            if (!left_arm_state.empty())
            {
                  std::copy(left_arm_state.begin(), left_arm_state.end(), target.begin() + 3);
                  ROS_INFO_STREAM("Setting Left arm joint targets");
            }
            if (!right_arm_state.empty())
            {
                  std::copy(right_arm_state.begin(), right_arm_state.end(), target.begin() + 3 + 6);
                  ROS_INFO_STREAM("Setting Right arm joint targets");
            }
            move_group_fr.setJointValueTarget(target);
            move_group_fr.move();
            ROS_INFO_STREAM("Full Robot moved");
      }

      moveit::planning_interface::MoveGroupInterface move_group_ra = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_RIGHT_ARM);
      moveit::planning_interface::MoveGroupInterface move_group_la = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_LEFT_ARM); 
      moveit::planning_interface::MoveGroupInterface move_group_fr = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_FULL_ROBOT);
      moveit::planning_interface::MoveGroupInterface move_group_g = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GANTRY);
private:
      ros::ServiceClient left_gripper_control_client, right_gripper_control_client;
      ros::ServiceClient getNextPart_client;
      ros::ServiceClient getNextPlacePosition_client;

      nist_gear::VacuumGripperState rightVGS, leftVGS;
      geometry_msgs::Quaternion gripperDown;

      ros::Subscriber right_vacuum_gripper_state_subscriber;
      ros::Subscriber left_vacuum_gripper_state_subscriber;
};

int main(int argc, char **argv)
{
      ros::init(argc, argv, "gantry_control_node");
      ros::NodeHandle node;

      ros::AsyncSpinner spinner(4);
      GantryControl gantry(node);
      spinner.start();

      ros::Rate loop_rate(10);

      while (!node.hasParam("robot_description"))
      {
            ros::Duration(1.0).sleep();
            ROS_INFO("waiting...");
      }

      planning_scene_diff_client =
          node.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
      planning_scene_diff_client.waitForExistence();
      planning_scene_get_client =
          node.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
      planning_scene_get_client.waitForExistence();

      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      moveit_msgs::ApplyPlanningScene aps;
      aps.request.scene.is_diff = true;
      planning_scene_interface.applyPlanningScene(aps.request.scene);
      planning_scene_diff_client.call(aps);

      //move robot to bin
      while (ros::ok())
      {
            nist_gear::Model nextModel = gantry.pickPart();
            if (nextModel.type == "null") {ros::Duration(1).sleep(); continue;}
            bool test = true;
            if (test)
            {      
                  gantry.move_full_robot(gantry_shelf_around_1, armsup_right,armsup_left_shelf);
                  gantry.move_full_robot(gantry_shelf_around_2,{},{});
                  gantry.move_full_robot(gantry_shelf_left,armsup_right_shelf,{});
                  gantry.move_full_robot({3.6, 4.8, PI/2},{},{});
                  // move_group_ra.setGoalTolerance(0.1);
                  // move_group_ra.setGoalOrientationTolerance(0.1);
                  // ROS_INFO_STREAM("starting pose: " << gantry.move_group_fr.getCurrentPose());
                  // //gantry.move_full_robot(gantry_shelf_right, armsup_right_shelf,armsup_left_shelf);
                  // ROS_INFO_STREAM("shelf pose: " << gantry.move_group_fr.getCurrentPose());
                  // geometry_msgs::PoseStamped target;
                  // target = gantry.move_group_fr.getCurrentPose();
                  // target.pose.position.x = gantry_shelf_right[0];
                  // target.pose.position.y = gantry_shelf_right[1];
                  // target.header.stamp = ros::Time::now();
                  // gantry.move_group_fr.setPoseTarget(target);
                  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  // bool planning_success = false;

                  // int attempts = 0;
                  // while (!planning_success && attempts < 3)
                  // {
                  //       attempts++;
                  //       planning_success = (gantry.move_group_fr.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                  //       ROS_INFO_STREAM("[ArmControl][reachTarget] planning_success = " << std::to_string(planning_success) << ", attempts: " << attempts << std::endl);
                  // }
                  // ROS_INFO_STREAM("[test] gantry target " << target<< "move success: " << gantry.move_group_fr.execute(my_plan));
                  gantry.grabPart("right", gantry.getPlacePosition("apple_green").position);

            }
            else
                  gantry.move_full_robot({nextModel.pose.position.x + 0.4, nextModel.pose.position.y - 0.4, 0.0}, armsup_right_shelf, armsup_left_shelf);

            //grab apple
            gantry.gripperControl("right", true);
            gantry.grabPart("right", nextModel.pose.position);
            gantry.withdrawArm("right");

            moveit_msgs::AttachedCollisionObject aco;
            aco.object.id = "attachedApple";
            aco.object.header.frame_id = gantry.move_group_ra.getEndEffectorLink();
            aco.link_name = gantry.move_group_ra.getEndEffectorLink();
            geometry_msgs::Pose acoP;
            acoP.orientation.w = 1.0;
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.2;
            primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.11;

            aco.object.primitives.push_back(primitive);
            aco.object.primitive_poses.push_back(acoP);

            aco.object.operation = aco.object.ADD;
            aps.request.scene.is_diff = true;
            planning_scene_interface.applyPlanningScene(aps.request.scene);

            // // lift arm after grabbing apple
            // double fraction;
            // moveit_msgs::RobotTrajectory trajectory;
            // std::vector<geometry_msgs::Pose> waypoints;
            geometry_msgs::Pose tP;
            // tP.position = nextModel.pose.position;
            // tP.position.z +=  0.05;
            // const double jump_threshold = 0.0;
            // const double eef_step = 0.15;
            // int attempts = 0;
            // gantry.move_group_ra.clearPathConstraints();
            // gantry.move_group_ra.setPlanningTime(10.0);
            // tP.position.z += 0.1;
            // waypoints.push_back(tP);
            // tP.position.z += 0.1;
            // tP.position.x += 0.1;
            // waypoints.push_back(tP);
            // fraction = gantry.move_group_ra.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, 1);
            // ROS_INFO_STREAM("[Cartesian path][withdraw arm] fraction = " << fraction);
            // attempts = 0;
            // while (std::abs(fraction - 1.0f) > 0.01 && attempts < 5)
            // {
            //       ROS_INFO("[Lift apple] Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
            //       fraction = gantry.move_group_ra.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, 1);
            //       attempts++;
            // }
            // gantry.move_group_ra.execute(trajectory);

            tP.position = gantry.getPlacePosition(nextModel.type).position;
            if (tP.position.y < 3.6)
            { //másik polc közepe y = -3.6
                  tP.position.z += 0.02;
                  gantry.move_full_robot(gantry_shelf_right, armsup_right_shelf, {});
            } else {
                  
            }
            gantry.grabPart("right", tP.position);
            gantry.gripperControl("right", false);
            // ROS_INFO_STREAM("placePart complete");
            gantry.withdrawArm("right", false);

            gantry.move_full_robot({}, armsup_right_shelf, {});
      }

      spinner.stop();
      ros::shutdown();
      return 0;
}