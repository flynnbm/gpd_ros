#include <gpd_ros/grasp_messages.h>
#include <tf2_eigen/tf2_eigen.hpp>

gpd_ros::GraspConfigList GraspMessages::createGraspListMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, 
                                                          const std_msgs::Header& header)
{
  gpd_ros::GraspConfigList msg;

  for (int i = 0; i < hands.size(); i++) {
    msg.grasps.push_back(convertToGraspMsg(*hands[i]));
  }

  msg.header = header;

  return msg;
}

gpd_ros::GraspConfig GraspMessages::convertToGraspMsg(const gpd::candidate::Hand& hand)
{
  gpd_ros::GraspConfig msg;

  msg.position = tf2::toMsg(hand.getPosition());
  msg.approach = tf2::toMsg(hand.getApproach());
  msg.binormal = tf2::toMsg(hand.getBinormal());
  msg.axis = tf2::toMsg(hand.getAxis());
  msg.width.data = hand.getGraspWidth();
  msg.score.data = hand.getScore();
  msg.sample = tf2::toMsg(hand.getSample());

  return msg;
}