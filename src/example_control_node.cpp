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
#include "utilities.h"

// %EndTag(INCLUDE_STATEMENTS)%

// %Tag(START_COMP)%
/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle &node)
{
   // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
   ros::ServiceClient start_client =
       node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
   // If it's not already ready, wait for it to be ready.
   // Calling the Service using the client before the server is ready would fail.
   if (!start_client.exists())
   {
      ROS_INFO("Waiting for the competition to be ready...");
      start_client.waitForExistence();
      ROS_INFO("Competition is now ready.");
   }
   ROS_INFO("Requesting competition start...");
   std_srvs::Trigger srv;  // Combination of the "request" and the "response".
   start_client.call(srv); // Call the start Service.
   if (!srv.response.success)
   { // If not successful, print out why.
      ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
   }
   else
   {
      ROS_INFO("Competition started!");
   }
}
// %EndTag(START_COMP)%

/// Example class that can hold state and provide methods that handle incoming data.
class MyCompetitionClass
{
public:
   sensor_msgs::PointCloud2 rgbd_camera_1_img, rgbd_cam_screenshot;
   nist_gear::LogicalCameraImage logical_camera_1_img;
   nist_gear::LogicalCameraImage logical_camera_2_img;

   explicit MyCompetitionClass(ros::NodeHandle &node)
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
   void current_score_callback(const std_msgs::Float32::ConstPtr &msg)
   {
      if (msg->data != current_score_)
      {
         // ROS_INFO_STREAM("Score: " << msg->data);
      }
      current_score_ = msg->data;
   }

   /// Called when a new message is received.
   void competition_state_callback(const std_msgs::String::ConstPtr &msg)
   {
      if (msg->data == "done" && competition_state_ != "done")
      {
         ROS_INFO("Competition ended.");
      }
      competition_state_ = msg->data;
   }

   /// Called when a new Order message is received.
   void order_callback(const nist_gear::Order::ConstPtr &order_msg)
   {
      //ROS_INFO_STREAM("Received order:\n" << *order_msg);
      received_orders_.push_back(*order_msg);
   }

   // %Tag(CB_CLASS)%
   /// Called when a new JointState message is received.
   void right_arm_joint_state_callback(
       const control_msgs::JointTrajectoryControllerState::ConstPtr &joint_state_msg)
   {
      //ROS_INFO_STREAM_THROTTLE(10,
      //                         "Joint States right arm (throttled to 0.1 Hz):\n" << *joint_state_msg);
      //ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
      right_arm_current_joint_states_ = *joint_state_msg;
      if (!right_arm_has_been_zeroed_)
      {
         right_arm_has_been_zeroed_ = true;
         // ROS_INFO("Sending right arm to zero joint positions...");
         //send_arm_to_zero_state(right_arm_joint_trajectory_publisher_, 1);
      }
   }

   void left_arm_joint_state_callback(
       const control_msgs::JointTrajectoryControllerState::ConstPtr &joint_state_msg)
   {
      //ROS_INFO_STREAM_THROTTLE(10,
      //                         "Joint States left arm (throttled to 0.1 Hz):\n" << *joint_state_msg);
      //ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
      left_arm_current_joint_states_ = *joint_state_msg;
      if (!left_arm_has_been_zeroed_)
      {
         left_arm_has_been_zeroed_ = true;
         // ROS_INFO("Sending left arm to zero joint positions...");
         //send_arm_to_zero_state(left_arm_joint_trajectory_publisher_, 2);
      }
   }

   void gantry_joint_state_callback(
       const sensor_msgs::JointState::ConstPtr &joint_state_msg)
   {
      //ROS_INFO_STREAM_THROTTLE(10,
      //                         "Joint States right arm (throttled to 0.1 Hz):\n" << *joint_state_msg);
      //ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
      gantry_current_joint_states_ = *joint_state_msg;
      if (!gantry_has_been_zeroed_)
      {
         gantry_has_been_zeroed_ = true;
         // ROS_INFO("Sending gantry to zero joint positions...");
         //send_gantry_to_zero_state(right_arm_joint_trajectory_publisher_);
      }
   }
   // %EndTag(CB_CLASS)%

   // %Tag(ARM_ZERO)%
   /// Create a JointTrajectory with all positions set to zero, and command the arm.
   void send_arm_to_zero_state(ros::Publisher &joint_trajectory_publisher, int arm_no)
   {
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
      // ROS_INFO_STREAM("Sending command:\n"
      //                 << msg);
      //joint_trajectory_publisher.publish(msg);
   }
   // %EndTag(ARM_ZERO)%

   /// Called when a new LogicalCameraImage message is received.
   void logical_camera_callback(
       const nist_gear::LogicalCameraImage::ConstPtr &image_msg)
   {
      ROS_INFO_STREAM_THROTTLE(10,
                               "Logical camera: '" << image_msg->models.size() << "' objects.");
      ///TODO:megcsinálni mindkettőre
      logical_camera_1_img = *image_msg;
   }

   void rgbd_camera_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
   {
      // ROS_INFO((std::to_string(msg.data[0]) + " " +std::to_string(msg.data[1]) + " " +std::to_string(msg.data[2])+ " " +std::to_string(msg.data[3])).c_str());
      // for (sensor_msgs::PointField field : msg.fields) {ROS_INFO(("name: " + field.name +" type: "+ std::to_string(field.datatype) +" count: "+ std::to_string(field.count) +" offest: "+ std::to_string(field.offset)).c_str());}
      // for (int i = 0; i < 96; i++) {
      //    ROS_INFO(std::to_string(msg.data[i]).c_str());
      // }
      rgbd_camera_1_img = *msg;
      // for (sensor_msgs::PointField field : msg->fields) ROS_INFO_STREAM("RGBD point field: " << field);
   }

   pcl::PointXYZ getColorOfClosestPoint(pcl::PointXYZRGB point)
   {
      pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(rgbd_cam_screenshot, *pc);
      // ROS_INFO((std::to_string(point.x) + " " + std::to_string(point.y) + " " + std::to_string(point.z)).c_str());
      // ROS_INFO((std::to_string(pc->points.at(0).r) + std::to_string(pc->points.at(0).g) + std::to_string(pc->points.at(0).b)).c_str());
      kdtree.setInputCloud(pc);
      //std::string frame = pc->header.frame_id;
      int howManyPointsDoWeNeedBack = 1;
      std::vector<int> resultPtIdx(howManyPointsDoWeNeedBack);
      std::vector<float> ezCsakKell(howManyPointsDoWeNeedBack);
      pcl::PointXYZ result(0.0f, 0.0f, 0.0f);
      ///TODO: transform pointcloud
      if (kdtree.nearestKSearch(point, howManyPointsDoWeNeedBack, resultPtIdx, ezCsakKell) > 0)
      {
         for (int point : resultPtIdx)
         {
            ROS_INFO_STREAM("found point all details **************************************\n" << pc->points.at(point));
            result.x += pc->points.at(point).b; //hülye a szenzor
            result.y += pc->points.at(point).g;
            result.z += pc->points.at(point).r;
            geometry_msgs::Pose p;
            p.position.x = pc->points.at(point).x;
            p.position.y = pc->points.at(point).y;
            p.position.z = pc->points.at(point).z;
            //ROS_INFO_STREAM("[getColorOfClosestPoint] closest found point rgbd : " << p);
            p = convert_to_frame(p, rgbd_camera_1_img.header.frame_id, "world");
            //ROS_INFO_STREAM("[getColorOfClosestPoint] closest found point world : " << p);
            // ROS_INFO((std::to_string(pc->points.at(point).x) + " " + std::to_string(pc->points.at(point).y) + " " + std::to_string(pc->points.at(point).z)).c_str());
         }
      }
      result.x;// /= 4.0f;
      result.y;// /= 4.0f;
      result.z;// /= 4.0f;
      //ROS_INFO(("Result color r:" + std::to_string(result.x) + " g: " + std::to_string(result.y) + " b: " + std::to_string(result.z)).c_str());
      // ROS_INFO_STREAM("[getColorOfClosestPoint] result: "<<result);
      return result;
      //int pixelCount = rgbd_camera_1_img.height * rgbd_camera_1_img.width;
      //ROS_INFO(std::to_string(pixelCount).c_str());
      //ROS_INFO(std::to_string( msg.data.size()).c_str());
   }

   nist_gear::Model getNextItemToMove()
   {
      if (logical_camera_1_img.models.empty())
      {
         ROS_INFO("Logical camera sees nothing.");
         throw "Logical camera sees nothing.";
      }
      else
      {  
         rgbd_cam_screenshot = rgbd_camera_1_img;
         nist_gear::Model nextModel = logical_camera_1_img.models.at(0);
         // ROS_INFO_STREAM(nextModel.pose);
         // ROS_INFO_STREAM(rgbd_camera_1_img.header.frame_id);
         nextModel.pose = convert_to_frame(nextModel.pose, "logical_camera_1_frame", "world");
         //ROS_INFO_STREAM("[GetNextItemToMove] nextModel.pose world 1 : " << nextModel);
         nextModel.pose.position.z = nextModel.pose.position.z;
         nextModel.pose = convert_to_frame(nextModel.pose, "world", rgbd_camera_1_img.header.frame_id);
         //ROS_INFO_STREAM("[GetNextItemToMove] nextModel.pose rgbd frame 2 : " << nextModel);
         pcl::PointXYZRGB point;
         point.x = nextModel.pose.position.x;
         point.y = nextModel.pose.position.y;
         point.z = nextModel.pose.position.z;
         //ROS_INFO_STREAM("[GetNextItemToMove] PointXYZRGB rgbd 3 : " << point);

         pcl::PointXYZ color = getColorOfClosestPoint(point);
         if (nextModel.type.find("apple") != std::string::npos) {
            if (color.x > color.y)
               nextModel.type = "red_apple";
            else
               nextModel.type = "green_apple";
         }
         nextModel.pose = convert_to_frame(nextModel.pose, rgbd_camera_1_img.header.frame_id, "world");
         //ROS_INFO_STREAM("[GetNextItemToMove] nextModel.pose world 4 : " << nextModel.pose);
         return nextModel;
      }
   }

   bool testCameras() {
      ROS_INFO("Testing cameras***************************************************************");
      ROS_INFO_STREAM(rgbd_camera_1_img.header.frame_id);
      std::vector<nist_gear::Model> apples;
      std::vector<pcl::PointXYZ> colors;

      if (logical_camera_1_img.models.empty())return true;
      else for (nist_gear::Model model : logical_camera_1_img.models) {
         rgbd_cam_screenshot = rgbd_camera_1_img;
         model.pose = convert_to_frame(model.pose, "logical_camera_1_frame", rgbd_camera_1_img.header.frame_id);
         pcl::PointXYZRGB point;
         point.x = -model.pose.position.x;
         point.y = model.pose.position.y;
         point.z = model.pose.position.z;
         pcl::PointXYZ color = getColorOfClosestPoint(point);
         if (model.type.find("apple") != std::string::npos) {
            if (color.x > color.y)
               model.type = "red_apple";
            else
               model.type = "green_apple";
            apples.push_back(model);
            colors.push_back(color);
         }
      }
      ROS_INFO_STREAM("test results: ");
      for (int i = 0; i < apples.size(); i++) ROS_INFO_STREAM(apples.at(i) << " color: " << colors.at(i));
      return false;
   }

   bool getNextItemToMove(controller::GetNextModel::Request &req, controller::GetNextModel::Response &res)
   {
      ROS_INFO("GetNextItemToMove is called...");
      res.nextModel = getNextItemToMove();
      return true;
   }

   /// Called when a new Proximity message is received.
   void break_beam_callback(const nist_gear::Proximity::ConstPtr &msg)
   {
      if (msg->object_detected)
      { // If there is an object in proximity.
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

void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr &msg)
{
   if ((msg->max_range - msg->range) > 0.01)
   { // If there is an object in proximity.
      ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
   }
}

void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   size_t number_of_valid_ranges = std::count_if(
       msg->ranges.begin(), msg->ranges.end(), [](const float f)
       { return std::isfinite(f); });
   if (number_of_valid_ranges > 0)
   {
      ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
   }
}

// %Tag(MAIN)%
int main(int argc, char **argv)
{
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

      ros::ServiceServer next_model_service =
          node.advertiseService("/ariac/next_model", &MyCompetitionClass::getNextItemToMove, &comp_class);

   // %Tag(SUB_FUNC)%
   // Subscribe to the '/ariac/proximity_sensor_1' topic.
   ros::Subscriber proximity_sensor_subscriber = node.subscribe(
       "/ariac/proximity_sensor_1", 10, proximity_sensor_callback);
   // %EndTag(SUB_FUNC)%

   // Subscribe to the '/ariac/break_beam_1_change' topic.
   ros::Subscriber break_beam_subscriber = node.subscribe(
       "/ariac/break_beam_1_change", 10,
       &MyCompetitionClass::break_beam_callback, &comp_class);

   ros::Subscriber rgbd_camera_subscriber = node.subscribe(
       "/ariac/rgbd_camera_1/points", 10,
       &MyCompetitionClass::rgbd_camera_callback, &comp_class);

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
   bool first = true;

   while (ros::ok())
   {
      ros::Duration(0.5).sleep();
      if (first) {
         first = false;// comp_class.testCameras();
         geometry_msgs::Pose p1,p2,p3;
         p1 = comp_class.getNextItemToMove().pose;
         p2 = convert_to_frame(p1, "world", "rgbd_camera_1_frame");
         ROS_INFO_STREAM("*******-------------------------------------------*****************\nrgbd_camera_1_frame " << p2 << "\nworld frame " << p1 << "\n*******-------------------------------------------*****************");
      }
      //nist_gear::Model model = comp_class.getNextItemToMove();
      // ROS_INFO(("Type: " + model.type + " Pose: " + std::to_string(model.pose.position.x) + ";"
      //          + std::to_string(model.pose.position.y) + ";" + std::to_string(model.pose.position.z)).c_str());
   }
   return 0;
}
// %EndTag(MAIN)%
// %EndTag(FULLTEXT)%
