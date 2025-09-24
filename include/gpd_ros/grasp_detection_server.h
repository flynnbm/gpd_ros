/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Andreas ten Pas
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef GRASP_DETECTION_SERVER_H_
#define GRASP_DETECTION_SERVER_H_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.h>
#include <visualization_msgs/msg/marker_array.h>
#include <std_msgs/msg/header.hpp>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// GPD
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>

// this project (services)
#include <gpd_ros/srv/detect_grasps.hpp>

// this project (messages)
#include <gpd_ros/msg/grasp_config.hpp>
#include <gpd_ros/msg/grasp_config_list.hpp>

// this project (headers)
#include <gpd_ros/grasp_messages.h>
#include <gpd_ros/grasp_plotter.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;

class GraspDetectionServer
{
public:

  /**
   * \brief Constructor.
   * \param node the ROS node
  */
  GraspDetectionServer(rclcpp::Node::SharedPtr& node);

  /**
   * \brief Destructor.
  */
  ~GraspDetectionServer()
  {
    delete cloud_camera_;
    delete grasp_detector_;
    delete rviz_plotter_;
  }

  /**
   * \brief Service callback for detecting grasps.
   * \param req the service request
   * \param res the service response
   */
  bool detectGrasps(gpd_ros::srv::DetectGrasps::Request& req, gpd_ros::srv::DetectGrasps::Response& res);


private:

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<gpd_ros::msg::GraspConfigList>::SharedPtr grasps_pub_;

  std_msgs::msg::Header cloud_camera_header_; ///< stores header of the point cloud
  std::string frame_; ///< point cloud frame

  gpd::GraspDetector* grasp_detector_; ///< used to run the grasp pose detection
  gpd::util::Cloud* cloud_camera_; ///< stores point cloud with (optional) camera information and surface normals
  GraspPlotter* rviz_plotter_; ///< used to plot detected grasps in rviz

  bool use_rviz_; ///< if rviz is used for visualization instead of PCL
  std::vector<double> workspace_; ///< workspace limits
  Eigen::Vector3d view_point_; ///< (input) view point of the camera onto the point cloud
};

#endif /* GRASP_DETECTION_SERVER_H_ */