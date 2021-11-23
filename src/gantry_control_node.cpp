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

            gripperDown.x=0.0;
            gripperDown.y=0.707;
            gripperDown.z=0.0;
            gripperDown.w=0.707;

            // options(PLANNING_GROUP_GANTRY, ROBOT_DESCRIPTION, node),
            // move_group_g = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GANTRY);
            // move_group_ra = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_RIGHT_ARM);
            // move_group_la = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_LEFT_ARM);
      }

      nist_gear::Model pickPart()
      {
            controller::GetNextModel srv;
            getNextPart_client.call(srv);
            //ROS_INFO(srv.response.nextModel.type.c_str());
            //srv.response.nextModel.pose = convert_to_frame(srv.response.nextModel.pose, "world", "torso_main");
            return srv.response.nextModel;
      }

      geometry_msgs::Pose getPlacePosition(std::string type)
      {
            controller::GetPlacePosition srv;
            srv.request.type = type;
            getNextPlacePosition_client.call(srv);
            //ROS_INFO(srv.response.nextModel.type.c_str());
            //srv.response.nextModel.pose = convert_to_frame(srv.response.nextModel.pose, "world", "torso_main");
            ROS_INFO_STREAM("[GetPlacePosition] srv response =" << srv.response);
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
            moveit_msgs::RobotTrajectory trajectory;
            std::vector<geometry_msgs::Pose> waypoints;
            geometry_msgs::Pose waypoint;
            waypoint.position = targetPosition;
            waypoint.position.z += 0.3;
            waypoint.orientation = gripperDown;
            waypoints.push_back(waypoint);
            //ROS_INFO_STREAM("[grabPart] waypoint " << waypoints.size() <<": " << waypoint);
            waypoint.position.z -= 0.105;
            waypoints.push_back(waypoint);
            //ROS_INFO_STREAM("[grabPart] waypoint " << waypoints.size() <<": " << waypoint);
            waypoint.position.z -= 0.1;
            waypoints.push_back(waypoint);
            //ROS_INFO_STREAM("[grabPart] waypoint " << waypoints.size() <<": " << waypoint);

            moveit_msgs::Constraints cons;
            moveit_msgs::OrientationConstraint ocons;
            ocons.header = moveGroup->getCurrentPose().header;
            ocons.link_name = moveGroup->getEndEffectorLink();
            ocons.orientation = moveGroup->getCurrentPose().pose.orientation;
            ocons.absolute_x_axis_tolerance = 0.5;
            ocons.absolute_y_axis_tolerance = 0.5;
            ocons.absolute_z_axis_tolerance = 0.5;
            ocons.weight = 1;
            cons.orientation_constraints = {ocons};
            moveGroup->setPathConstraints(cons);
            const double jump_threshold = 0.0;
            const double eef_step = 0.15;
            double fraction = moveGroup->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, 1);
            ROS_INFO_STREAM("[Cartesian path][grabPart] fraction = " << fraction);
            int attempts = 0;
            while (std::abs(fraction - 1.0f) > 0.01 && attempts < 5)
            {
                  ROS_INFO("[grabPart] Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
                  fraction = moveGroup->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, 1);
                  attempts++;
            }
            trajectory.joint_trajectory.header.stamp = ros::Time::now();
            trajectory.multi_dof_joint_trajectory.header.stamp = ros::Time::now();
            moveGroup->execute(trajectory);
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
            // for (double joint : currentJointValues) {
            //       ROS_INFO_STREAM("movefullrobot currentJoints" << joint);
            // }
            std::copy(currentJointValues.begin(), currentJointValues.end(), target.begin());
            if (!gantry_state.empty())
            {
                  gantry_state[1] *= -1;
                  // move_group_g.setJointValueTarget(gantry_state);
                  // move_group_g.move();
                  std::copy(gantry_state.begin(), gantry_state.end(), target.begin());
                  ROS_INFO_STREAM("Gantry moved");
            }
            if (!left_arm_state.empty())
            {
                  // move_group_la.setJointValueTarget(left_arm_state);
                  // move_group_la.move();
                  std::copy(left_arm_state.begin(), left_arm_state.end(), target.begin() + 3);
                  ROS_INFO_STREAM("Left arm moved");
            }
            if (!right_arm_state.empty())
            {
                  // move_group_ra.setJointValueTarget(right_arm_state);
                  // move_group_ra.move();
                  std::copy(right_arm_state.begin(), right_arm_state.end(), target.begin() + 3 + 6);
                  ROS_INFO_STREAM("Right arm moved");
            }
            // std::vector<double> target(15);
            // target[0] = nextModel.pose.position.x + 0.4;
            // target[1] = -nextModel.pose.position.y + 0.4;
            // // target[0] = move_group_fr.getCurrentPose().pose.position.x;
            // // target[1] = move_group_fr.getCurrentPose().pose.position.y;
            // target[2] = 0;
            // std::copy(armsup_left_shelf.begin(), armsup_left_shelf.end(), target.begin() + 3);
            // std::copy(armsup_right_shelf.begin(), armsup_right_shelf.end(), target.begin() + 3 + 6);
            // // target_pose1.position.x=target[0];
            // // target_pose1.position.y=target[1];
            // // target_pose1.position.z=1.6f;
            // for (double joint : target) {
            //       ROS_INFO_STREAM("movefullrobot targetJoints " << joint);
            // }
            move_group_fr.setJointValueTarget(target);
            move_group_fr.move();
            ROS_INFO_STREAM("Full Robot moved");
      }

      moveit::planning_interface::MoveGroupInterface move_group_ra = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_RIGHT_ARM);
      moveit::planning_interface::MoveGroupInterface move_group_la = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_LEFT_ARM); 
      moveit::planning_interface::MoveGroupInterface move_group_g = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GANTRY);
      moveit::planning_interface::MoveGroupInterface move_group_fr = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_FULL_ROBOT);
      // moveit::planning_interface::MoveGroupInterface move_group_ree(PLANNING_GROUP_RIGHT_EE);
      // moveit::planning_interface::MoveGroupInterface move_group_lee(PLANNING_GROUP_LEFT_EE);
private:
      ros::ServiceClient left_gripper_control_client, right_gripper_control_client;
      geometry_msgs::Quaternion gripperDown;
};

int main(int argc, char **argv)
{
      //std::string id = argv[1];

      ros::init(argc, argv, "gantry_control_node");
      ros::NodeHandle node;

      ros::AsyncSpinner spinner(4);
      GantryControl gantry(node);
      spinner.start();

      // ArmControl arm(node, id);

      ros::Rate loop_rate(10);

      // while (ros::ok()) {

      //    ros::Duration(5.0).sleep();

      //    loop_rate.sleep();
      // }

      while (!node.hasParam("robot_description"))
      {
            ros::Duration(1.0).sleep();
            ROS_INFO("waiting...");
      }
      //ros::spin();  // This executes callbacks on new data until ctrl-c.

      // ros::Subscriber right_arm_gripper_state_subscriber = node.subscribe( "/ariac/gantry/right_arm/gripper/state", 10,
      // right_arm_gripper_state_callback);
      // ros::Subscriber left_arm_gripper_state_subscriber = node.subscribe( "/ariac/gantry/left_arm/gripper/state", 10,
      // left_arm_gripper_state_callback);

      planning_scene_diff_client =
          node.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
      planning_scene_diff_client.waitForExistence();
      planning_scene_get_client =
          node.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
      planning_scene_get_client.waitForExistence();

      /**/ robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
      const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
      //ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
      moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
      kinematic_state->setToDefaultValues();
      //const moveit::core::JointModelGroup *joint_model_group_fr = kinematic_model->getJointModelGroup("Full_Robot");
      const moveit::core::JointModelGroup *joint_model_group_ra = kinematic_model->getJointModelGroup("Right_Arm");
      const moveit::core::JointModelGroup *joint_model_group_la = kinematic_model->getJointModelGroup("Left_Arm");
      const moveit::core::JointModelGroup *joint_model_group_g = kinematic_model->getJointModelGroup("Gantry");
      // const moveit::core::JointModelGroup *joint_model_group_ree = kinematic_model->getJointModelGroup("Right_Endeffector");
      // const moveit::core::JointModelGroup *joint_model_group_lee = kinematic_model->getJointModelGroup("Left_Endeffector");
      //const std::vector<std::string> &joint_names_fr = joint_model_group_fr->getVariableNames();
      const std::vector<std::string> &joint_names_ra = joint_model_group_ra->getVariableNames();
      const std::vector<std::string> &joint_names_la = joint_model_group_la->getVariableNames();
      const std::vector<std::string> &joint_names_g = joint_model_group_g->getVariableNames();
      // const std::vector<std::string> &joint_names_ree = joint_model_group_ree->getVariableNames();
      // const std::vector<std::string> &joint_names_lee = joint_model_group_lee->getVariableNames();
      // BEGIN_TUTORIAL
      //
      // Setup
      // ^^^^^
      //
      // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
      // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
      // are used interchangably.
      // ARIAC/gantry_moveit_config/config/gantry.srdf
      // The :planning_interface:`MoveGroupInterface` class can be easily
      // setup using just the name of the planning group you would like to control and plan for.
      // moveit::planning_interface::MoveGroupInterface move_group_g = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GANTRY);
      // moveit::planning_interface::MoveGroupInterface move_group_ra(PLANNING_GROUP_RIGHT_ARM);
      // move_group_la = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_LEFT_ARM);
      // trajectory_msgs::JointTrajectory gantry_msg;
      //gantry_msg.points.at(0);

      // We will use the :planning_interface:`PlanningSceneInterface`
      // class to add and remove collision objects in our "virtual world" scene
      // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      // Raw pointers are frequently used to refer to the planning group for improved performance.
      // joint_model_group_ra = move_group_ra.getCurrentState()->getJointModelGroup(PLANNING_GROUP_RIGHT_ARM);
      // joint_model_group_la = move_group_la.getCurrentState()->getJointModelGroup(PLANNING_GROUP_LEFT_ARM);
      // joint_model_group_g = move_group_g.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GANTRY);
      //joint_model_group_fr = move_group_fr.getCurrentState()->getJointModelGroup(PLANNING_GROUP_FULL_ROBOT);
      // joint_model_group_lee = move_group_lee.getCurrentState()->getJointModelGroup(PLANNING_GROUP_LEFT_EE);
      // joint_model_group_ree = move_group_ree.getCurrentState()->getJointModelGroup(PLANNING_GROUP_RIGHT_EE);
      // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
      // std::copy(move_group_fr.getJointModelGroupNames().begin(), move_group_fr.getJointModelGroupNames().end(),
      //          std::ostream_iterator<std::string>(std::cout, ", "));
      // move_group_fr.clearPoseTargets();
      // bool link = move_group_fr.setEndEffectorLink("left_ee_link");
      // bool ee = move_group_fr.setEndEffector("left_ee");
      // ROS_INFO(("link: " + std::to_string(link) + " ee: " + std::to_string(ee)).c_str());
      // std::vector<double> jointState = move_group_fr.getCurrentJointValues();
      // for (double value : jointState)
      // {
      //ROS_INFO(std::to_string(value).c_str());
      // }
      // jointState[0] = 0.5;
      // geometry_msgs::PoseStamped currentPose = move_group_fr.getCurrentPose();
      // ROS_INFO(("X: "+ std::to_string(currentPose.pose.position.x) +" y: "+ std::to_string(currentPose.pose.position.y) +" Z: "+ std::to_string(currentPose.pose.position.z)).c_str());
      //move_group_fr.setJointValueTarget(jointState);
      // ROS_INFO(move_group_fr.getEndEffectorLink().c_str());
      // ROS_INFO(move_group_ra.getEndEffectorLink().c_str());
      // ROS_INFO(move_group_la.getEndEffectorLink().c_str());
      // ROS_INFO_STREAM("Starting RA " << move_group_ra.getCurrentPose());
      geometry_msgs::Pose orient;
      orient.orientation.x = 0.0;
      orient.orientation.y = 0.707;
      orient.orientation.z = 0.0;
      orient.orientation.w = 0.707;

      //       orientation:
      //     x: 0.0451722
      //     y: 0.795099
      //     z: 0.0816435
      //     w: 0.599259

      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//       moveit_msgs::CollisionObject collision_object;
//       collision_object.header.frame_id = "world";

//       collision_object.id = "testCollObj";

//       shape_msgs::SolidPrimitive primitive;
//       primitive.type = primitive.BOX;
//       primitive.dimensions.resize(3);
//       primitive.dimensions[primitive.BOX_X] = 0.1;
//       primitive.dimensions[primitive.BOX_Y] = 1.5;
//       primitive.dimensions[primitive.BOX_Z] = 0.5;

//       geometry_msgs::Pose box_pose;
//       box_pose.orientation.w = 1.0;
//       box_pose.position.x = 2.0;
//       box_pose.position.y = 1.22;
//       box_pose.position.z = 1.1;
// //3.375796 y= 1.224079 z= 0.924265
//       collision_object.primitives.push_back(primitive);
//       collision_object.primitive_poses.push_back(box_pose);
//       collision_object.operation = collision_object.ADD;

//       std::vector<moveit_msgs::CollisionObject> collision_objects;
//       collision_objects.push_back(collision_object);

      // tf2::Quaternion q;
      // q.setRPY( 0, 0, -1 );
      // //ROS_INFO_STREAM("x: "<<q.getX()<<" y: "<<q.getY()<<" z: " <<q.getZ() << " w: "<<q.getW());

      // q.normalize();

      // tf2::Quaternion quat_tf;
      // geometry_msgs::Quaternion quat_msg;

      //tf2::convert(quat_msg , quat_tf);

      // ROS_INFO_NAMED("tutorial", "Add an object into the world");
      //planning_scene_interface.applyCollisionObjects(collision_objects);
      moveit_msgs::ApplyPlanningScene aps;
      aps.request.scene.is_diff = true;
      planning_scene_interface.applyPlanningScene(aps.request.scene);
      planning_scene_diff_client.call(aps);

      // geometry_msgs::Pose target_pose1;
      // target_pose1.position.x = -0.8;
      // target_pose1.position.y = 0.26;
      // target_pose1.position.z = 1.42;

      //target_pose1 = convert_to_frame(target_pose1, "world", "torso_main");
      // ROS_INFO((std::to_string(target_pose1.position.x) + " " + std::to_string(target_pose1.position.y) + " " +std::to_string(target_pose1.position.z)).c_str());
      //move_group_ra.setPoseTarget(target_pose1);
      // move_group_ra.setGoalTolerance(0.1);
      // move_group_ra.setGoalOrientationTolerance(0.1);
      // move_group_g.setGoalTolerance(0.1);
      moveit_msgs::Constraints cons;
      moveit_msgs::OrientationConstraint ocons;

      ocons.header = gantry.move_group_ra.getCurrentPose().header;
      ocons.link_name = gantry.move_group_ra.getEndEffectorLink();
      // q.setRPY( PI, PI, PI );
      // ROS_INFO_STREAM("x: "<<q.getX()<<" y: "<<q.getY()<<" z: " <<q.getZ() << " w: "<<q.getW());

      // q.normalize();

      // // tf2::Quaternion quat_tf;
      // // geometry_msgs::Quaternion quat_msg;

      // tf2::convert(quat_msg , quat_tf);
      nist_gear::Model nextModel = gantry.pickPart();
      //gantry.getPlacePosition(nextModel.type);
      //ROS_INFO_STREAM("after pickpart " << nextModel.pose);


      //gantry.move_full_robot(gantry_bin, armsup_right_shelf, armsup_left_shelf);
      gantry.move_full_robot({nextModel.pose.position.x + 0.4, nextModel.pose.position.y - 0.4, 0.0}, armsup_right_shelf, armsup_left_shelf);


      //move_group_g.setPoseTarget(target_pose1);
      // ROS_INFO(("x: " + std::to_string(target[0]) + " y: " + std::to_string(target[1])).c_str());
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      // bool planning_success = (move_group_fr.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      // int attempts = 0;
      // while (!planning_success && attempts < 5)
      // {
      //    attempts++;
      //    planning_success = (move_group_fr.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      //    ROS_INFO_STREAM("[ArmControl][reachTarget] planning_success = " << std::to_string(planning_success) << ", attempts: " << attempts << std::endl);
      // }
      // // move_group_fr.execute(my_plan);
      // move_group_fr.execute(my_plan);
      // move_group_fr.setMaxVelocityScalingFactor(1);
      //move_group_ra.setMaxVelocityScalingFactor(1);
      // ROS_INFO_STREAM("Current Right_Arm alap "<<move_group_ra.getCurrentPose().pose);

      // geometry_msgs::PoseStamped cPose = move_group_g.getCurrentPose();
      //for (std::string str : move_group_g.getLinkNames()) ROS_INFO(("gantry link: " + str).c_str());
      // ROS_INFO(("endeffector: " + move_group_g.getEndEffectorLink()).c_str());
      //ROS_INFO(("Current gantry pose: x= " + std::to_string(cPose.pose.position.x) + " y= " + std::to_string(cPose.pose.position.y) + " z= " + std::to_string(cPose.pose.position.z)).c_str());
      // move_group_fr.move();
      // cPose = move_group_ra.getCurrentPose();
      // ROS_INFO(("Current RA pose: x= " + std::to_string(cPose.pose.position.x) + " y= " + std::to_string(cPose.pose.position.y) + " z= " + std::to_string(cPose.pose.position.z)).c_str());

      // nextModel.pose = convert_to_frame(nextModel.pose, "world", "right_base");
      // ROS_INFO_STREAM("right_base " << nextModel);
      geometry_msgs::Pose tP;
      tP.position=nextModel.pose.position;
      // tP = convert_to_frame(tP, "world", "right_ee_link");
      // tP = move_group_ra.getRandomPose();
      //ROS_INFO("before tP");
      //ROS_INFO_STREAM(nextModel.pose);
      tP.position.x = nextModel.pose.position.x;
      tP.position.y = nextModel.pose.position.y;
      tP.position.z = nextModel.pose.position.z + 0.3;
      // ROS_INFO_STREAM(tP.position);
      // tP.position.x=2.6;
      // tP.position.y=1.4;
      // tP.position.z=1.4;
      // tP.orientation = cPose.pose.orientation;

      // move_group_ra.setPoseTarget(tP);
      // ocons.orientation = move_group_ra.getCurrentPose().pose.orientation;
      // ocons.absolute_x_axis_tolerance = 2;
      // ocons.absolute_y_axis_tolerance = 2;
      // ocons.absolute_z_axis_tolerance = 2;
      // ocons.weight = 1;
      // cons.orientation_constraints = {ocons};
      // move_group_ra.setPathConstraints(cons);

      // move_group_ra.setPositionTarget(tP.position.x, tP.position.y, tP.position.z);
      // ROS_INFO(("tP pose: x= " + std::to_string(tP.position.x) + " y= " + std::to_string(tP.position.y) + " z= " + std::to_string(tP.position.z)).c_str());
      // planning_success = (move_group_ra.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      // attempts = 0;
      // while (!planning_success && attempts < 15)
      // {
      //       attempts++;
      //       planning_success = (move_group_ra.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      //       ROS_INFO_STREAM("[ArmControl][reachTarget] planning_success = " << std::to_string(planning_success) << ", attempts: " << attempts << std::endl);
      // }
      // // move_group_ra.execute(my_plan);
      // move_group_ra.execute(my_plan);
      // ROS_INFO_STREAM("Current Right_Arm "<<move_group_ra.getCurrentPose().pose);

      // ocons.orientation = gantry.move_group_ra.getCurrentPose().pose.orientation;
      // ocons.absolute_x_axis_tolerance = 0.5;
      // ocons.absolute_y_axis_tolerance = 0.5;
      // ocons.absolute_z_axis_tolerance = 0.5;
      // ocons.weight = 1;
      // cons.orientation_constraints = {ocons};
      // gantry.move_group_ra.setPathConstraints(cons);
      // // // ROS_INFO_STREAM("Current Right_Arm starting "<<move_group_ra.getCurrentPose().pose);
      double fraction;
      moveit_msgs::RobotTrajectory trajectory;
      std::vector<geometry_msgs::Pose> waypoints;
      // geometry_msgs::Pose tmpPose;
      // //waypoints.push_back(tP);
      // tP.orientation = orient.orientation;
      // waypoints.push_back(tP);
      // tP.position.z -= 0.105;
      // waypoints.push_back(tP);
      // //tP.position.y-=0.1;
      // tP.position.z -= 0.1;
      // waypoints.push_back(tP);
      // // tP.position.z += 0.15;
      // // waypoints.push_back(tP);
      const double jump_threshold = 0.0;
      const double eef_step = 0.15;
      // // double fraction = gantry.move_group_ra.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, 1);
      // // ROS_INFO_STREAM("[Cartesian path][Reach for apple] fraction = " << fraction);
      int attempts = 0;
      // while (std::abs(fraction - 1.0f) > 0.01 && attempts < 5)
      // {
      //       // tmpPose = move_group_ra.getCurrentPose().pose;
      //       // tmpPose.position.x -= 0.1;
      //       // move_group_ra.setPoseTarget(tmpPose);
      //       // move_group_ra.move();
      //       ROS_INFO("[Reach for apple] Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
      //       fraction = gantry.move_group_ra.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, 1);
      //       attempts++;
      // }
      // trajectory.joint_trajectory.header.stamp = ros::Time::now();
      // trajectory.multi_dof_joint_trajectory.header.stamp = ros::Time::now();
      gantry.gripperControl("right", true);
      // ROS_INFO_STREAM("Current Right_Arm "<<move_group_ra.getCurrentPose().pose);
      

      ROS_INFO_STREAM("nextModel.pose.position: " << nextModel.pose.position);
      ROS_INFO_STREAM("tP.position: " << tP.position);
      gantry.grabPart("right", nextModel.pose.position);
      //gantry.move_group_ra.execute(trajectory);
      // ROS_INFO_STREAM("Current Right_Arm "<<move_group_ra.getCurrentPose().pose);
      // ROS_INFO_STREAM("Current Right_Arm "<<move_group_ra.getCurrentPose().pose.orientation);
      ros::Duration(1.0).sleep();
      // attempts = 0;
      // while (!rightVGS.attached && attempts < 2)
      // {
      //       waypoints.clear();
      //       tP.position.z += 0.15;
      //       waypoints.push_back(tP);
      //       tP.position.z -= 0.15;
      //       waypoints.push_back(tP);
      //       fraction = gantry.move_group_ra.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, 1);
      //       ROS_INFO("[isAttahed] attempts %i", attempts);
      //       attempts++;
      //       ROS_INFO_STREAM("[Cartesian path][grab apple again] fraction = " << fraction);
      //       gantry.move_group_ra.execute(trajectory);
      // }

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
      //0.727725
      //planning_scene_interface.applyAttachedCollisionObject(aco);

      aps.request.scene.is_diff = true;
      planning_scene_interface.applyPlanningScene(aps.request.scene);
      //planning_scene_diff_client.call(aps);

      gantry.move_group_ra.clearPathConstraints();
      //waypoints.clear();
      gantry.move_group_ra.setPlanningTime(10.0);
      tP.position.z += 0.1;
      waypoints.push_back(tP);
      tP.position.z += 0.1;
      tP.position.x += 0.1;
      waypoints.push_back(tP);
      fraction = gantry.move_group_ra.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, 1);
      ROS_INFO_STREAM("[Cartesian path][withdraw arm] fraction = " << fraction);
      attempts = 0;
      // ROS_INFO_STREAM("[RA EE link] " << gantry.move_group_ra.getEndEffectorLink());
      while (std::abs(fraction - 1.0f) > 0.01 && attempts < 5)
      {
            ROS_INFO("[Lift apple] Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
            fraction = gantry.move_group_ra.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, 1);
            attempts++;
      }
      gantry.move_group_ra.execute(trajectory);

      // move_group_fr.setJointValueTarget(move_full_robot(move_group_fr.getCurrentPose().pose.position.x,-move_group_fr.getCurrentPose().pose.position.y,armsup_right_shelf,armsup_left_shelf));

      // move_group_fr.move();

      tP.position = gantry.getPlacePosition(nextModel.type).position;
      ROS_INFO_STREAM("[placePart] gantry moves to " << tP);
      tP.position.z += 0.02;
      gantry.move_full_robot(gantry_shelf_right, armsup_right_shelf, {});
      // gantry.move_full_robot({tP.position.x + 0.6, tP.position.y - 0.9, 0}, armsup_right_shelf, {});

      gantry.grabPart("right", tP.position);

      // waypoints.clear();
      // tP.position.y -= 0.3;
      // tP.position.x -= 0.3;
      // tP.position.z += 0.3;
      // waypoints.push_back(tP);
      // tP.orientation = orient.orientation;
      // tP.position.z -= 0.1;
      // waypoints.push_back(tP);
      // tP.position.z -= 0.1;
      // waypoints.push_back(tP);

      // fraction = gantry.move_group_ra.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, 1);
      // ROS_INFO_STREAM("[Cartesian path][place apple again] fraction = " << fraction);
      // attempts = 0;
      // ROS_INFO_STREAM("[RA EE link] " << gantry.move_group_ra.getEndEffectorLink());
      // while (std::abs(fraction - 1.0f) > 0.01 && attempts < 5)
      // {
      //       ROS_INFO("[Place apple] Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
      //       fraction = gantry.move_group_ra.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, 1);
      //       attempts++;
      // }
      // gantry.move_group_ra.execute(trajectory);

      gantry.gripperControl("right", false);
      gantry.move_full_robot({}, armsup_right_shelf, {});

      spinner.stop();
      ros::shutdown();
      return 0;
}