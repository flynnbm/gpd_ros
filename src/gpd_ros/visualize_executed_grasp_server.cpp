#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>

#include <gpd_ros/srv/visualize_executed_grasp.hpp>

using VisualizeExecutedGrasp = gpd_ros::srv::VisualizeExecutedGrasp;

class VisualizeExecutedGraspServer : public rclcpp::Node
{
public:
  VisualizeExecutedGraspServer()
  : rclcpp::Node("gpd_visualize_executed_grasp_server"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    source_frame_ = this->declare_parameter<std::string>("source_frame", "camera_link");
    marker_outer_diameter_ = this->declare_parameter<double>("marker_outer_diameter", 0.105);
    marker_hand_depth_ = this->declare_parameter<double>("marker_hand_depth", 0.06);
    marker_finger_width_ = this->declare_parameter<double>("marker_finger_width", 0.01);
    marker_hand_height_ = this->declare_parameter<double>("marker_hand_height", 0.02);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/executed_grasp_markers", rclcpp::QoS(1).reliable().transient_local());

    service_ = this->create_service<VisualizeExecutedGrasp>(
      "/visualize_executed_grasp",
      std::bind(&VisualizeExecutedGraspServer::handle_request, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

private:
  visualization_msgs::msg::Marker makeCube_(
      const std::string& frame_id, const rclcpp::Time& stamp,
      const std::string& ns, int id, const Eigen::Vector3d& center,
      const Eigen::Matrix3d& orientation, const Eigen::Vector3d& scale,
      float red, float green, float blue, float alpha = 0.55f) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = center.x();
    marker.pose.position.y = center.y();
    marker.pose.position.z = center.z();
    const Eigen::Quaterniond q(orientation);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = scale.x();
    marker.scale.y = scale.y();
    marker.scale.z = scale.z();
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    return marker;
  }

  void appendFork_(visualization_msgs::msg::MarkerArray& markers,
      const std::string& frame_id, const rclcpp::Time& stamp,
      const std::string& ns, const Eigen::Vector3d& origin,
      const Eigen::Vector3d& forward, const Eigen::Vector3d& closing,
      const Eigen::Vector3d& vertical, float red, float green, float blue) const
  {
    Eigen::Matrix3d orientation;
    orientation.col(0) = forward.normalized();
    orientation.col(1) = closing.normalized();
    orientation.col(2) = vertical.normalized();
    if (orientation.determinant() < 0.0) {
      orientation.col(2) *= -1.0;
    }

    const double half_opening = 0.5 * marker_outer_diameter_ - 0.5 * marker_finger_width_;
    const Eigen::Vector3d left_bottom = origin - half_opening * orientation.col(1);
    const Eigen::Vector3d right_bottom = origin + half_opening * orientation.col(1);
    const Eigen::Vector3d left_center = left_bottom + 0.5 * marker_hand_depth_ * orientation.col(0);
    const Eigen::Vector3d right_center = right_bottom + 0.5 * marker_hand_depth_ * orientation.col(0);
    const Eigen::Vector3d base_center = origin - 0.01 * orientation.col(0);
    const Eigen::Vector3d approach_center = base_center - 0.04 * orientation.col(0);

    markers.markers.push_back(makeCube_(frame_id, stamp, ns, 0, left_center, orientation,
      Eigen::Vector3d(marker_hand_depth_, marker_finger_width_, marker_hand_height_), red, green, blue));
    markers.markers.push_back(makeCube_(frame_id, stamp, ns, 1, right_center, orientation,
      Eigen::Vector3d(marker_hand_depth_, marker_finger_width_, marker_hand_height_), red, green, blue));
    markers.markers.push_back(makeCube_(frame_id, stamp, ns, 2, approach_center, orientation,
      Eigen::Vector3d(0.08, marker_finger_width_, marker_hand_height_), red, green, blue));
    markers.markers.push_back(makeCube_(frame_id, stamp, ns, 3, base_center, orientation,
      Eigen::Vector3d(0.02, marker_outer_diameter_ - marker_finger_width_, marker_hand_height_),
      red, green, blue));
  }

  void handle_request(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<VisualizeExecutedGrasp::Request> req,
      std::shared_ptr<VisualizeExecutedGrasp::Response> res)
  {
    if (req->selected_index >= req->grasp_configs.grasps.size()) {
      res->success = false;
      res->message = "selected_index is outside grasp_configs";
      return;
    }
    if (req->transformed_target.header.frame_id.empty()) {
      res->success = false;
      res->message = "transformed_target frame is empty";
      return;
    }

    const std::string grasp_source_frame = req->grasp_configs.header.frame_id.empty()
      ? source_frame_
      : req->grasp_configs.header.frame_id;
    if (req->grasp_configs.header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(),
        "Visualization request has an empty grasp header; falling back to configured source_frame '%s'",
        source_frame_.c_str());
    }
    if (grasp_source_frame.empty()) {
      res->success = false;
      res->message = "grasp header and configured source_frame are both empty";
      return;
    }

    geometry_msgs::msg::TransformStamped target_from_gpd_msg;
    try {
      target_from_gpd_msg = tf_buffer_.lookupTransform(
        req->transformed_target.header.frame_id, grasp_source_frame,
        tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      res->success = false;
      res->message = std::string("TF lookup failed: ") + ex.what();
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
      return;
    }

    const auto& grasp = req->grasp_configs.grasps[req->selected_index];
    Eigen::Matrix3d gpd_orientation;
    gpd_orientation.col(0) = Eigen::Vector3d(
      grasp.approach.x, grasp.approach.y, grasp.approach.z).normalized();
    gpd_orientation.col(1) = Eigen::Vector3d(
      grasp.binormal.x, grasp.binormal.y, grasp.binormal.z).normalized();
    gpd_orientation.col(2) = Eigen::Vector3d(
      grasp.axis.x, grasp.axis.y, grasp.axis.z).normalized();
    if (gpd_orientation.determinant() < 0.0) {
      gpd_orientation.col(2) *= -1.0;
    }

    Eigen::Isometry3d source_from_gpd = Eigen::Isometry3d::Identity();
    source_from_gpd.linear() = gpd_orientation;
    source_from_gpd.translation() = Eigen::Vector3d(
      grasp.position.x, grasp.position.y, grasp.position.z);
    const Eigen::Isometry3d target_from_gpd =
      tf2::transformToEigen(target_from_gpd_msg.transform) * source_from_gpd;
    Eigen::Isometry3d target_from_expected = Eigen::Isometry3d::Identity();
    tf2::fromMsg(req->transformed_target.pose, target_from_expected);

    visualization_msgs::msg::MarkerArray markers;
    const std::string& frame_id = req->transformed_target.header.frame_id;
    const rclcpp::Time stamp = this->now();

    // Clear markers from the previous request, including markers published by
    // older versions of this service that also displayed the actual TCP pose.
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    appendFork_(markers, frame_id, stamp, "gpd_grasp", target_from_gpd.translation(),
      target_from_gpd.linear().col(0), target_from_gpd.linear().col(1),
      target_from_gpd.linear().col(2), 0.1f, 0.25f, 1.0f);
    // This robot's gripper closes along TCP +X and approaches along TCP +Z.
    appendFork_(markers, frame_id, stamp, "planned_tcp", target_from_expected.translation(),
      target_from_expected.linear().col(2), target_from_expected.linear().col(0),
      target_from_expected.linear().col(2).cross(target_from_expected.linear().col(0)),
      0.1f, 1.0f, 0.2f);

    marker_pub_->publish(markers);
    res->success = true;
    res->message = "Published selected GPD grasp and planned TCP target";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  double marker_outer_diameter_{0.105}, marker_hand_depth_{0.06};
  double marker_finger_width_{0.01}, marker_hand_height_{0.02};
  std::string source_frame_{"camera_link"};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Service<VisualizeExecutedGrasp>::SharedPtr service_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisualizeExecutedGraspServer>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
