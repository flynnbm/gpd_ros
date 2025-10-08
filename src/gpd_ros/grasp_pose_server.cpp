#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
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
    // Parameters
    double gripper_offset_ = this->declare_parameter<double>("gripper_offset", -0.02);
    double approach_dist_ = this->declare_parameter<double>("approach_dist", 0.10);

    double grasp_rot_x_ = this->declare_parameter<double>("grasp_rot_x", 0.0);
    double grasp_rot_y_ = this->declare_parameter<double>("grasp_rot_y", 0.39269908169);
    double grasp_rot_z_ = this->declare_parameter<double>("grasp_rot_z", -0.981747704);
    double grasp_rot_w_ = this->declare_parameter<double>("grasp_rot_w", 1.0);

    std::string target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
    std::string source_frame_  = this->declare_parameter<std::string>("source_frame",  "camera_link");

    // Service (3-arg callback: header, req, res)
    service_ = this->create_service<ComputeGraspPoses>(
      "/compute_grasp_poses",
      std::bind(&GraspPoseServer::handle_request, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

private:
  void handle_request(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                      const std::shared_ptr<ComputeGraspPoses::Request> req,
                      std::shared_ptr<ComputeGraspPoses::Response> res)
  {
    RCLCPP_INFO(this->get_logger(), "Received /compute_grasp_poses service call");

    // Lookup transform: target <- source
    geometry_msgs::msg::TransformStamped T_target_source_msg;
    try {
      T_target_source_msg = tf_buffer_.lookupTransform(
        target_frame_, source_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "TF lookup failed (%s <- %s): %s",
                   target_frame_.c_str(), source_frame_.c_str(), ex.what());
      // leave res->poses empty; client should treat empty as failure
      return;
    }
    const Eigen::Isometry3d T_target_source = tf2::transformToEigen(T_target_source_msg.transform);

    // TCP & orientation offset
    const Eigen::Quaterniond q_offset(grasp_rot_w_, grasp_rot_x_, grasp_rot_y_, grasp_rot_z_);
    Eigen::Isometry3d T_offset = Eigen::Isometry3d::Identity();
    T_offset.linear() = q_offset.toRotationMatrix();
    T_offset.translation() = Eigen::Vector3d(0, 0, -gripper_offset_);

    res->target_poses.clear();
    res->approach_poses.clear();
    res->target_poses.reserve(req->grasps.grasps.size());
    res->approach_poses.reserve(req->grasps.grasps.size());

    for (const auto &g : req->grasps.grasps)
    {
        // Rotation matrix with columns = [-axis, binormal, approach]
        Eigen::Matrix3d R_grasp_source;
        R_grasp_source.col(0) = Eigen::Vector3d(-g.axis.x,    -g.axis.y,    -g.axis.z);
        R_grasp_source.col(1) = Eigen::Vector3d( g.binormal.x, g.binormal.y, g.binormal.z);
        R_grasp_source.col(2) = Eigen::Vector3d( g.approach.x, g.approach.y, g.approach.z);

        // Normalize
        for (int c = 0; c < 3; ++c) {
            double n = R_grasp_source.col(c).norm();
            if (n > 1e-9) R_grasp_source.col(c) /= n;
        }

        Eigen::Isometry3d T_grasp_source = Eigen::Isometry3d::Identity();
        T_grasp_source.linear() = R_grasp_source;
        T_grasp_source.translation() = Eigen::Vector3d(g.position.x, g.position.y, g.position.z);

        // Compose final pose in target frame
        const Eigen::Isometry3d T_target_grasp = T_target_source * T_grasp_source * T_offset;

        geometry_msgs::msg::Pose target_pose_msg;
        const Eigen::Quaterniond q(T_target_grasp.linear());
        target_pose_msg.position.x = T_target_grasp.translation().x();
        target_pose_msg.position.y = T_target_grasp.translation().y();
        target_pose_msg.position.z = T_target_grasp.translation().z();
        target_pose_msg.orientation.x = q.x();
        target_pose_msg.orientation.y = q.y();
        target_pose_msg.orientation.z = q.z();
        target_pose_msg.orientation.w = q.w();

        res->target_poses.push_back(target_pose_msg);

        // generate approach pose by moving back along z axis from target pose
        Eigen::Isometry3d T_target_approach =
        T_target_grasp * Eigen::Translation3d(0.0, 0.0, -approach_dist_);

        geometry_msgs::msg::Pose approach_msg;
        {
            const Eigen::Quaterniond qa(T_target_approach.linear());
            approach_msg.position.x = T_target_approach.translation().x();
            approach_msg.position.y = T_target_approach.translation().y();
            approach_msg.position.z = T_target_approach.translation().z();
            approach_msg.orientation.x = qa.x();
            approach_msg.orientation.y = qa.y();
            approach_msg.orientation.z = qa.z();
            approach_msg.orientation.w = qa.w();
        }
        res->approach_poses.push_back(approach_msg);
    }
  }

  // class members
  double gripper_offset_, approach_dist_;
  double grasp_rot_x_, grasp_rot_y_, grasp_rot_z_, grasp_rot_w_;
  std::string target_frame_, source_frame_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Service
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