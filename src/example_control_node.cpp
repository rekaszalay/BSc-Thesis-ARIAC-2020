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

#include "utilities.h"


void start_competition(ros::NodeHandle &node)
{
   ros::ServiceClient start_client =
       node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
   if (!start_client.exists())
   {
      ROS_INFO("Waiting for the competition to be ready...");
      start_client.waitForExistence();
      ROS_INFO("Competition is now ready.");
   }
   ROS_INFO("Requesting competition start...");
   std_srvs::Trigger srv;
   start_client.call(srv);
   if (!srv.response.success)
   { 
      ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
   }
   else
   {
      ROS_INFO("Competition started!");
   }
}


class MyCompetitionClass
{
public:
   sensor_msgs::PointCloud2 rgbd_camera_1_img, rgbd_cam_screenshot;
   nist_gear::LogicalCameraImage logical_camera_1_img;
   nist_gear::LogicalCameraImage logical_camera_2_img;

   explicit MyCompetitionClass(ros::NodeHandle &node)
       : current_score_(0), right_arm_has_been_zeroed_(false), left_arm_has_been_zeroed_(false)
   {

   }

   void logical_camera_1_callback(const nist_gear::LogicalCameraImage::ConstPtr &image_msg) {
      ROS_INFO_STREAM_THROTTLE(10,
         "Logical camera 1: '" << image_msg->models.size() << "' objects.");
      logical_camera_1_img = *image_msg;
   }

   void logical_camera_2_callback(const nist_gear::LogicalCameraImage::ConstPtr &image_msg) {
      ROS_INFO_STREAM_THROTTLE(10,
         "Logical camera 2: '" << image_msg->models.size() << "' objects.");
      logical_camera_2_img = *image_msg;
   }

   void rgbd_camera_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
   {
      rgbd_camera_1_img = *msg;
   }

   pcl::PointXYZ getColorOfClosestPoint(pcl::PointXYZRGB point)
   {
      pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(rgbd_cam_screenshot, *pc);
      kdtree.setInputCloud(pc);
      int numberOfNearestPts = 4;
      std::vector<int> resultPtIdx(numberOfNearestPts);
      std::vector<float> pointSquareDist(numberOfNearestPts);
      pcl::PointXYZ result(0.0f, 0.0f, 0.0f);
      if (kdtree.nearestKSearch(point, numberOfNearestPts, resultPtIdx, pointSquareDist) > 0)
      {
         for (int point : resultPtIdx)
         {
            result.x += pc->points.at(point).b;
            result.y += pc->points.at(point).g;
            result.z += pc->points.at(point).r;
         }
      }
      result.x /= 4;
      result.y /= 4;
      result.z /= 4;
      return result;
   }

   nist_gear::Model getNextItemToMove()
   {
      nist_gear::Model nextModel;
      nextModel.type="null";
      std::vector<nist_gear::Model> logical_camera_models(logical_camera_1_img.models);
      if (logical_camera_1_img.models.empty())
      {
         ROS_INFO("Logical camera sees nothing.");
         return nextModel;
      }
      else
      {  
         rgbd_cam_screenshot = rgbd_camera_1_img;
         for (nist_gear::Model model : logical_camera_models) {
               //if (model.type.find("green") != std::string::npos)
               {nextModel = model;
               break;}
         }
         if (nextModel.type == "null")
         {
            ROS_INFO("Logical camera sees no apple.");
            return nextModel;
         }
         nextModel.pose = convert_to_frame(nextModel.pose, "logical_camera_1_frame", "world");
         nextModel.pose.position.z += 0.05;
         nextModel.pose = convert_to_frame(nextModel.pose, "world", rgbd_camera_1_img.header.frame_id);
         pcl::PointXYZRGB point;
         point.x = nextModel.pose.position.x;
         point.y = nextModel.pose.position.y;
         point.z = nextModel.pose.position.z;

         pcl::PointXYZ color = getColorOfClosestPoint(point);
         if (nextModel.type.find("apple") != std::string::npos) {
            if (color.x > color.y)
               nextModel.type = "apple_red";
            else
               nextModel.type = "apple_green";
         }
         nextModel.pose = convert_to_frame(nextModel.pose, rgbd_camera_1_img.header.frame_id, "world");
         nextModel.pose.position.z -= 0.05;
         return nextModel;
      }
   }

   geometry_msgs::Pose getNextPlacePosition(std::string type) {
      geometry_msgs::Pose poseRed, poseGreen;
      poseRed.position.x= 2.5;
      poseRed.position.y= 3.1;
      poseRed.position.z= 1.5;
      poseGreen.position.x= 2.5;
      poseGreen.position.y= 3.95;
      poseGreen.position.z= 1.5;
      if (logical_camera_2_img.models.empty()) {
         if (type == "apple_red") {
            ROS_INFO_STREAM("[getNextPlacePosition] no models, return red"); 
            return poseRed;
         }
         if (type == "apple_green") {
            ROS_INFO_STREAM("[getNextPlacePosition] no models, return green"); 
            return poseGreen;
         };
      }
      nist_gear::Model lastModel;
      lastModel.type = "null";

      std::vector<nist_gear::Model> LCmodels;
      for(nist_gear::Model model : logical_camera_2_img.models) {
         model.pose = convert_to_frame(model.pose, "logical_camera_2_frame", "world");
         LCmodels.push_back(model);
      }
      for(nist_gear::Model model : LCmodels) {
         if (model.type == type) {
            if (lastModel.type == "null") lastModel = model;
            if (abs(lastModel.pose.position.y - model.pose.position.y) < 0.05) {
               if (model.pose.position.x > lastModel.pose.position.x) lastModel = model;
            } else if (model.pose.position.y > lastModel.pose.position.y) lastModel = model;
         }
      }
      if (lastModel.type == "null") {
         ROS_INFO_STREAM("[getNextPlacePosition] camera sees no " << type); 
         if (type.find("red") != std::string::npos) return poseRed;
         else return poseGreen;
      } else if (lastModel.pose.position.x + 0.2 > 3.8) {
         ROS_INFO_STREAM("[getNextPlacePosition] found last model, new row " << lastModel.pose.position); 
         lastModel.pose.position.x = 2.5;
         lastModel.pose.position.y += 0.2;
         lastModel.pose.position.z += 0.1;
         return lastModel.pose;
      } else {
         ROS_INFO_STREAM("[getNextPlacePosition] found last model, same row " << lastModel.pose.position); 
         lastModel.pose.position.x += 0.2;
         lastModel.pose.position.z += 0.1;
         return lastModel.pose;
      }
      ROS_INFO_STREAM("[[getNextPlacePosition] HOW DID I GET HERE?? " << lastModel);
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
               model.type = "apple_red";
            else
               model.type = "apple_green";
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

   bool getNextPlacePosition(controller::GetPlacePosition::Request &req, controller::GetPlacePosition::Response &res)
   {
      ROS_INFO("GetNextPlacePosition is called...");
      res.position = getNextPlacePosition(req.type);
      return true;
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


int main(int argc, char **argv)
{
   ros::init(argc, argv, "example_control_node");

   ros::NodeHandle node;

   MyCompetitionClass comp_class(node);

   ros::ServiceServer next_model_service =
         node.advertiseService("/ariac/next_model", &MyCompetitionClass::getNextItemToMove, &comp_class);

   ros::ServiceServer next_place_position_service =
         node.advertiseService("/ariac/next_place_position", &MyCompetitionClass::getNextPlacePosition, &comp_class);

   ros::Subscriber rgbd_camera_subscriber = node.subscribe(
       "/ariac/rgbd_camera_1/points", 10,
       &MyCompetitionClass::rgbd_camera_callback, &comp_class);

   ros::Subscriber logical_camera_1_subscriber = node.subscribe(
       "/ariac/logical_camera_1", 10,
       &MyCompetitionClass::logical_camera_1_callback, &comp_class);

   ros::Subscriber logical_camera_2_subscriber = node.subscribe(
       "/ariac/logical_camera_2", 10,
       &MyCompetitionClass::logical_camera_2_callback, &comp_class);

   ROS_INFO("Setup complete.");
   start_competition(node);
   ros::AsyncSpinner spinner(1);
   spinner.start();
   bool first = false;

   while (ros::ok())
   {
      ros::Duration(0.5).sleep();
      if (first) {
         first = false;// comp_class.testCameras();
         geometry_msgs::Pose p1,p2,p3;
         p1 = comp_class.getNextItemToMove().pose;
         p2 = convert_to_frame(p1, "world", "rgbd_camera_1_frame");
      }
   }
   return 0;
}
