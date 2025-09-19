#include <gpd_ros/grasp_messages.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

gpd_ros::msg::GraspConfigList GraspMessages::createGraspListMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, 
                                                          const std_msgs::msg::Header& header)
{
  gpd_ros::msg::GraspConfigList msg;

  for (int i = 0; i < hands.size(); i++) {
    msg.grasps.push_back(convertToGraspMsg(*hands[i]));
  }

  msg.header = header;

  return msg;
}

gpd_ros::msg::GraspConfig GraspMessages::convertToGraspMsg(const gpd::candidate::Hand& hand)
{
  gpd_ros::msg::GraspConfig msg;

  msg.position = tf2::toMsg(hand.getPosition());
  msg.approach = tf2::toMsg2(hand.getApproach());
  msg.binormal = tf2::toMsg2(hand.getBinormal());
  msg.axis = tf2::toMsg2(hand.getAxis());
  msg.width.data = hand.getGraspWidth();
  msg.score.data = hand.getScore();
  msg.sample = tf2::toMsg(hand.getSample());

  return msg;
}