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

            gripperDown.x = 0.0;
            gripperDown.y = 0.707;
            gripperDown.z = 0.0;
            gripperDown.w = 0.707;
      }

      nist_gear::Model pickPart()
      {
            controller::GetNextModel srv;
            getNextPart_client.call(srv);
            ROS_INFO_STREAM("[pickPart] next model to pick is " << srv.response.nextModel.type);
            return srv.response.nextModel;
      }

      geometry_msgs::Pose getPlacePosition(std::string type)
      {
            controller::GetPlacePosition srv;
            srv.request.type = type;
            getNextPlacePosition_client.call(srv);
            ROS_INFO_STREAM("[getPlacePosition] place position for " << type);
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

      void grabPart(std::string arm, geometry_msgs::Point targetPosition)
      {
            moveit::planning_interface::MoveGroupInterface *moveGroup;
            if (arm == "left")
            {
                  ROS_INFO_STREAM("[grabPart] Moving left arm");
                  moveGroup = &move_group_la;
            }
            else if (arm == "right")
            {
                  ROS_INFO_STREAM("[grabPart] Moving right arm");
                  moveGroup = &move_group_ra;
            }
            else
            {
                  ROS_INFO_STREAM("[grabPart] Invalid arm ID!");
                  return;
            }
            
            std::vector<geometry_msgs::Pose> waypoints;
            geometry_msgs::Pose waypoint;
            waypoint.position = targetPosition;
            waypoint.position.z += 0.3;
            waypoint.orientation = gripperDown;
            waypoints.push_back(waypoint);
            waypoint.position.z -= 0.105;
            waypoints.push_back(waypoint);
            waypoint.position.z -= 0.1;
            waypoints.push_back(waypoint);
            ROS_INFO_STREAM("[grabPart] Executing trajectory");

            moveGroup->execute(trajectoryFromWaypoints(*moveGroup, waypoints));
      }

      moveit_msgs::RobotTrajectory trajectoryFromWaypoints(moveit::planning_interface::MoveGroupInterface &moveGroup, std::vector<geometry_msgs::Pose> waypoints)
      {
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
                  if (std::abs(fraction - 1.0f) > 0.01)
                  {
                        std::vector<double> gantry_state = move_group_g.getCurrentJointValues();
                        gantry_state[0] += 0.05;
                        gantry_state[1] = -gantry_state[1];
                        for (double joint : gantry_state )
                        {
                              ROS_INFO_STREAM("[trajectoryFromWaypoint] fraction < 1, moving full robot to " << joint);
                        }
                        move_full_robot(gantry_state, {}, {});
                  }
            }
            trajectory.joint_trajectory.header.stamp = ros::Time::now();
            trajectory.multi_dof_joint_trajectory.header.stamp = ros::Time::now();
            return trajectory;
      }

      void withdrawArm(std::string armID, bool afterGrab = true)
      {
            nist_gear::VacuumGripperState *gripperState;
            moveit::planning_interface::MoveGroupInterface *moveGroup;
            if (armID == "left")
            {
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
            waypoint.position.z += 0.2;
            if (afterGrab)
            {
                  ros::Duration(0.5).sleep();
                  while (!gripperState->attached)
                  {
                        ROS_INFO_STREAM("[withdrawArm] Apple not attached. Trying again...");
                        std::vector<geometry_msgs::Pose> waypoints;
                        waypoints.push_back(waypoint);
                        waypoint.position.z -= 0.2;
                        waypoints.push_back(waypoint);
                        moveGroup->execute(trajectoryFromWaypoints(*moveGroup, waypoints));
                        waypoint.position.z += 0.2;
                        ros::Duration(1).sleep();
                  }
                  moveit_msgs::AttachedCollisionObject aco;
                  aco.object.id = "attachedApple";
                  aco.object.header.frame_id = moveGroup->getEndEffectorLink();
                  aco.link_name = moveGroup->getEndEffectorLink();
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

            } else {
                  moveit_msgs::AttachedCollisionObject aco;
                  aco.object.id = "attachedApple";
                  aco.object.header.frame_id = moveGroup->getEndEffectorLink();
                  aco.link_name = moveGroup->getEndEffectorLink();
                  aco.object.operation = aco.object.REMOVE;
                  aps.request.scene.is_diff = true;
                  planning_scene_interface.applyPlanningScene(aps.request.scene);
            }
            moveGroup->execute(trajectoryFromWaypoints(*moveGroup, {waypoint}));
      }

      void goAroundShelf(bool back = false)
      {
            if (back)
            {
                  move_full_robot(gantry_shelf_around_3, right_arm_moving, left_arm_moving);
                  move_full_robot(gantry_shelf_around_2, {}, {});
                  move_full_robot(gantry_shelf_around_1, {}, {});
            }
            else
            {
                  move_full_robot(gantry_shelf_around_1, right_arm_moving, left_arm_moving);
                  move_full_robot(gantry_shelf_around_2, {}, {});
                  move_full_robot(gantry_shelf_around_3, {}, {});
                  move_full_robot(gantry_shelf_left, {}, {});
            }
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
                  ROS_INFO_STREAM("[move_full_robot] Setting Gantry joint targets");
            }
            if (!left_arm_state.empty())
            {
                  std::copy(left_arm_state.begin(), left_arm_state.end(), target.begin() + 3);
                  ROS_INFO_STREAM("[move_full_robot] Setting Left arm joint targets");
            }
            if (!right_arm_state.empty())
            {
                  std::copy(right_arm_state.begin(), right_arm_state.end(), target.begin() + 3 + 6);
                  ROS_INFO_STREAM("[move_full_robot] Setting Right arm joint targets");
            }

            move_group_fr.setJointValueTarget(target);
            move_group_fr.move();
            ROS_INFO_STREAM("[move_full_robot] Full Robot moved");
      }

      moveit::planning_interface::MoveGroupInterface move_group_ra = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_RIGHT_ARM);
      moveit::planning_interface::MoveGroupInterface move_group_la = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_LEFT_ARM);
      moveit::planning_interface::MoveGroupInterface move_group_fr = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_FULL_ROBOT);
      moveit::planning_interface::MoveGroupInterface move_group_g = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GANTRY);
      std::string arm_to_use = "";
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      moveit_msgs::ApplyPlanningScene aps;

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
      
      gantry.move_group_fr.setMaxAccelerationScalingFactor(1.0);
      gantry.move_group_fr.setMaxVelocityScalingFactor(1.0);

      //move robot to bin
      while (ros::ok())
      {
            nist_gear::Model nextModel = gantry.pickPart();
            while (nextModel.type == "null")
            {
                  ROS_INFO_STREAM("No next model. Waiting...");
                  ros::Duration(1).sleep();
                  nextModel = gantry.pickPart();
            }
            if (nextModel.type.find("green") != std::string::npos)
            {
                  gantry.arm_to_use = "left";
            }
            else if (nextModel.type.find("red") != std::string::npos)
            {
                  gantry.arm_to_use = "right";
            }
            
            bool test = false;
            if (test)
            {
                  gantry.goAroundShelf();
            }

            if (-gantry.move_group_fr.getCurrentJointValues()[1] > 3.6)
            {
                  gantry.goAroundShelf(1);
            }
            if (gantry.arm_to_use == "right") {
                  ROS_INFO_STREAM("moving robot to right arm grab position");
                  gantry.move_full_robot({nextModel.pose.position.x + 0.4, nextModel.pose.position.y - 0.4, 0.0}, right_arm_shelf, left_arm_shelf);
            } else {
                  ROS_INFO_STREAM("moving robot to left arm grab position");
                  gantry.move_full_robot({nextModel.pose.position.x + 0.8, nextModel.pose.position.y + 0.4, PI}, right_arm_shelf, left_arm_shelf);
            }

            //grab apple
            gantry.gripperControl(gantry.arm_to_use, true);
            gantry.grabPart(gantry.arm_to_use, nextModel.pose.position);
            gantry.withdrawArm(gantry.arm_to_use);

            geometry_msgs::Pose tP;
            tP.position = gantry.getPlacePosition(nextModel.type).position;
            if (tP.position.y < 3.6)
            {
                  gantry.move_full_robot(gantry_shelf_right, right_arm_shelf, left_arm_shelf);
            }
            else
            {
                  gantry.goAroundShelf();
            }

            //put down apple
            gantry.grabPart(gantry.arm_to_use, tP.position);
            gantry.gripperControl(gantry.arm_to_use, false);
            gantry.withdrawArm(gantry.arm_to_use, false);

            //gantry.move_full_robot({}, right_arm_shelf, left_arm_shelf);
      }

      spinner.stop();
      ros::shutdown();
      return 0;
}