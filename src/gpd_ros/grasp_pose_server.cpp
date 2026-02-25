#include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gpd_ros/msg/grasp_config_list.hpp>
#include <gpd_ros/msg/grasp_config.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>

#include <gpd_ros/srv/compute_grasp_poses.hpp>

using ComputeGraspPoses = gpd_ros::srv::ComputeGraspPoses;

class GraspPoseServer : public rclcpp::Node
{
public:
  GraspPoseServer()
  : rclcpp::Node("gpd_grasp_pose_server"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Parameter defaults
    this->gripper_offset_ = this->declare_parameter<double>("gripper_offset", 0.00);
    this->approach_dist_  = this->declare_parameter<double>("approach_dist",  0.10);
    this->retreat_dist_   = this->declare_parameter<double>("retreat_dist",   0.10);

    this->grasp_rot_x_ = this->declare_parameter<double>("grasp_rot_x", 0.0);
    this->grasp_rot_y_ = this->declare_parameter<double>("grasp_rot_y", 0.0);
    this->grasp_rot_z_ = this->declare_parameter<double>("grasp_rot_z", 0.0);
    this->grasp_rot_w_ = this->declare_parameter<double>("grasp_rot_w", 1.0);

    this->target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
    this->source_frame_ = this->declare_parameter<std::string>("source_frame", "camera_link");

    service_ = this->create_service<ComputeGraspPoses>(
      "/compute_grasp_poses",
      std::bind(&GraspPoseServer::handle_request, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

private:
  // Helper: make a PoseStamped from an Isometry, with consistent header
  geometry_msgs::msg::PoseStamped stampPose_(
      const Eigen::Isometry3d& T,
      const std::string& frame_id,
      const rclcpp::Time& stamp) const
  {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = stamp;

    const Eigen::Quaterniond q(T.linear());
    pose_stamped.pose.position.x = T.translation().x();
    pose_stamped.pose.position.y = T.translation().y();
    pose_stamped.pose.position.z = T.translation().z();
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();
    return pose_stamped;
  }

  void handle_request(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                      const std::shared_ptr<ComputeGraspPoses::Request> req,
                      std::shared_ptr<ComputeGraspPoses::Response> res)
  {
    RCLCPP_INFO(this->get_logger(), "Received /compute_grasp_poses service call");

    // lookup transform to target_frame from source_frame
    geometry_msgs::msg::TransformStamped T_target_source_msg;
    try {
      T_target_source_msg = tf_buffer_.lookupTransform(
        target_frame_, source_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "TF lookup failed (%s <- %s): %s",
                   target_frame_.c_str(), source_frame_.c_str(), ex.what());
      return;
    }
    const Eigen::Isometry3d T_target_source = tf2::transformToEigen(T_target_source_msg.transform);

    // generate output time stamp
    rclcpp::Time out_stamp = T_target_source_msg.header.stamp;
    if (out_stamp.nanoseconds() == 0) {
      out_stamp = this->now();
    }

    // generate initial offset transform from parameters
    const Eigen::Quaterniond q_offset(grasp_rot_w_, grasp_rot_x_, grasp_rot_y_, grasp_rot_z_);
    Eigen::Isometry3d T_offset = Eigen::Isometry3d::Identity();
    T_offset.linear() = q_offset.toRotationMatrix();
    T_offset.translation() = Eigen::Vector3d(0, 0, gripper_offset_);

    res->target_poses.clear();
    res->approach_poses.clear();
    res->retreat_poses.clear();
    res->target_poses.reserve(req->grasps.grasps.size());
    res->approach_poses.reserve(req->grasps.grasps.size());
    res->retreat_poses.reserve(req->grasps.grasps.size());

    for (const auto &g : req->grasps.grasps)
    {
      // Build grasp pose in SOURCE frame from GPD axes
      Eigen::Matrix3d R_grasp_source;
      R_grasp_source.col(0) = Eigen::Vector3d(-g.axis.x,    -g.axis.y,    -g.axis.z);
      R_grasp_source.col(1) = Eigen::Vector3d( g.binormal.x, g.binormal.y, g.binormal.z);
      R_grasp_source.col(2) = Eigen::Vector3d( g.approach.x, g.approach.y, g.approach.z);
      for (int c = 0; c < 3; ++c) {
        double n = R_grasp_source.col(c).norm();
        if (n > 1e-9) R_grasp_source.col(c) /= n;
      }

      Eigen::Isometry3d T_grasp_source = Eigen::Isometry3d::Identity();
      T_grasp_source.linear() = R_grasp_source;
      T_grasp_source.translation() = Eigen::Vector3d(g.position.x, g.position.y, g.position.z);

      // grasp pose in target's frame
      const Eigen::Isometry3d T_target_grasp = T_target_source * T_grasp_source * T_offset;

      // Target pose
      res->target_poses.push_back( stampPose_(T_target_grasp, target_frame_, out_stamp) );

      // Approach pose: back along tool Z axis
      const Eigen::Isometry3d T_target_approach =
          T_target_grasp * Eigen::Translation3d(0.0, 0.0, -approach_dist_);
      res->approach_poses.push_back( stampPose_(T_target_approach, target_frame_, out_stamp) );

      // Retreat pose: straight up along robot frame Z axis
      Eigen::Isometry3d T_target_retreat = T_target_grasp;
      T_target_retreat.translate(Eigen::Vector3d(0.0, 0.0, retreat_dist_));
      res->retreat_poses.push_back( stampPose_(T_target_retreat, target_frame_, out_stamp) );
    }
  }

  // class members
  double gripper_offset_{0.0}, approach_dist_{0.0}, retreat_dist_{0.0};
  double grasp_rot_x_{0.0}, grasp_rot_y_{0.0}, grasp_rot_z_{0.0}, grasp_rot_w_{1.0};
  std::string target_frame_{"base_link"}, source_frame_{"camera_link"};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Service<ComputeGraspPoses>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GraspPoseServer>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}