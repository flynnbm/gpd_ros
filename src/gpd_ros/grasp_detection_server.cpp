#include <gpd_ros/grasp_detection_server.h>

GraspDetectionServer::GraspDetectionServer(rclcpp::Node::SharedPtr& node) :
  node_(node),
  grasp_detector_(nullptr),
  cloud_camera_(nullptr),
  rviz_plotter_(nullptr),
  use_rviz_(false)
{
  // set camera viewpoint to default origin
  const auto camera_position =
      node_->declare_parameter<std::vector<double>>("camera_position", {0.0, 0.0, 0.0});
  if (camera_position.size() >= 3) {
    view_point_ << camera_position[0], camera_position[1], camera_position[2];
  } else {
    // if there aren't enough elements for whatever reason, set position to (0,0,0)
    RCLCPP_WARN(node_->get_logger(), "Parameter 'camera_position' must have at least 3 elements; using (0,0,0).");
    view_point_.setZero();
  }

  // find config file using filepath param "config_file"
  // create grasp_detector based on params in config file
  const std::string cfg_file =
      node_->declare_parameter<std::string>("config_file", "");
  grasp_detector_ = new gpd::GraspDetector(cfg_file);

  // rviz plotter (enabled only if a topic is provided)
  const std::string rviz_topic =
      node_->declare_parameter<std::string>("rviz_topic", "grasp_markers");
  if (!rviz_topic.empty()) {
    rviz_plotter_ = new GraspPlotter(
        node_, grasp_detector_->getHandSearchParameters().hand_geometry_);
    use_rviz_ = true;
  } else {
    use_rviz_ = false;
  }

  // publish detected grasps on "grasps_topic" param
  const std::string grasps_topic =
      node_->declare_parameter<std::string>("grasps_topic", "clustered_grasps");
  grasps_pub_ = node_->create_publisher<gpd_ros::msg::GraspConfigList>(
      grasps_topic, rclcpp::QoS(10));

  // set workspace limits based on "workspace" param
  workspace_ = node_->declare_parameter<std::vector<double>>(
                "workspace", std::vector<double>{});
}

bool GraspDetectionServer::detectGrasps(gpd_ros::srv::DetectGrasps::Request& req, gpd_ros::srv::DetectGrasps::Response& res)
{
  RCLCPP_INFO(node_->get_logger(), "Received service request ...");

  // 1. Initialize cloud camera.
  cloud_camera_ = nullptr;
  const gpd_ros::msg::CloudSources& cloud_sources = req.cloud_indexed.cloud_sources;

  // Set view points.
  Eigen::Matrix3Xd view_points(3, cloud_sources.view_points.size());
  for (std::size_t i = 0; i < cloud_sources.view_points.size(); i++)
  {
    view_points.col(i) << cloud_sources.view_points[i].x, cloud_sources.view_points[i].y,
      cloud_sources.view_points[i].z;
  }

  // Set point cloud.
  if (cloud_sources.cloud.fields.size() == 6 && cloud_sources.cloud.fields[3].name == "normal_x"
    && cloud_sources.cloud.fields[4].name == "normal_y" && cloud_sources.cloud.fields[5].name == "normal_z")
  {
    PointCloudPointNormal::Ptr cloud(new PointCloudPointNormal);
    pcl::fromROSMsg(cloud_sources.cloud, *cloud);

    // TODO: multiple cameras can see the same point
    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), cloud->size());
    for (std::size_t i = 0; i < cloud_sources.camera_source.size(); i++)
    {
      camera_source(cloud_sources.camera_source[i].data, i) = 1;
    }

    cloud_camera_ = new gpd::util::Cloud(cloud, camera_source, view_points);
  }
  else
  {
    PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
    pcl::fromROSMsg(cloud_sources.cloud, *cloud);

    // TODO: multiple cameras can see the same point
    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), cloud->size());
    for (std::size_t i = 0; i < cloud_sources.camera_source.size(); i++)
    {
      camera_source(cloud_sources.camera_source[i].data, i) = 1;
    }

    cloud_camera_ = new gpd::util::Cloud(cloud, camera_source, view_points);
    std::cout << "view_points:\n" << view_points << "\n";
  }

  // Set the indices at which to sample grasp candidates.
  std::vector<int> indices(req.cloud_indexed.indices.size());
  for (std::size_t i=0; i < indices.size(); i++)
  {
    indices[i] = req.cloud_indexed.indices[i].data;
  }
  cloud_camera_->setSampleIndices(indices);

  frame_ = req.cloud_indexed.cloud_sources.cloud.header.frame_id;

  RCLCPP_INFO_STREAM(node_->get_logger(), "Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points, and "
    << req.cloud_indexed.indices.size() << " samples");

  // 2. Preprocess the point cloud.
  grasp_detector_->preprocessPointCloud(*cloud_camera_);

  // 3. Detect grasps in the point cloud.
  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = grasp_detector_->detectGrasps(*cloud_camera_);

  if (grasps.size() > 0)
  {
    // Visualize the detected grasps in rviz.
    if (use_rviz_)
    {
      rviz_plotter_->drawGrasps(grasps, frame_);
    }

    // Publish the detected grasps.
    gpd_ros::msg::GraspConfigList selected_grasps_msg = GraspMessages::createGraspListMsg(grasps, cloud_camera_header_);
    res.grasp_configs = selected_grasps_msg;
    RCLCPP_INFO_STREAM(node_->get_logger(), "Detected " << selected_grasps_msg.grasps.size() << " highest-scoring grasps.");
    return true;
  }

  RCLCPP_WARN(node_->get_logger(), "No grasps detected!");
  return false;
}

int main(int argc, char** argv)
{
  // seed the random number generator
  std::srand(static_cast<unsigned int>(std::time(nullptr)));

  // initialize ROS
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("detect_grasps_server");

  GraspDetectionServer server(node);

  const std::string service_name =
      node->declare_parameter<std::string>("service_name", "detect_grasps");

  auto srv = node->create_service<gpd_ros::srv::DetectGrasps>(service_name,
      [&server, node](const std::shared_ptr<gpd_ros::srv::DetectGrasps::Request> req,
                      std::shared_ptr<gpd_ros::srv::DetectGrasps::Response> res)
      {
        const bool ok = server.detectGrasps(*req, *res);
        if (!ok) {
          RCLCPP_WARN(node->get_logger(), "detectGrasps() returned false");
        }
      });

  RCLCPP_INFO(node->get_logger(), "Service '%s' ready.", service_name.c_str());

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}