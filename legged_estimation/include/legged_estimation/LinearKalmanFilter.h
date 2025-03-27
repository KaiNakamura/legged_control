//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include "legged_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <realtime_tools/realtime_buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include "std_msgs/Float64.h"
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>
#include "std_msgs/Float64MultiArray.h"

namespace legged {
using namespace ocs2;
// Based on https://roboticsproceedings.org/rss08/p03.pdf
class KalmanFilterEstimate : public StateEstimateBase {
 public:
  KalmanFilterEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);
  void getMap(const grid_map_msgs::GridMap& msg);

  vector_t update(const ros::Time& time, const ros::Duration& period) override;

  void loadSettings(const std::string& taskFile, bool verbose);

 protected:
  void updateFromTopic();

  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  nav_msgs::Odometry getOdomMsg();

  vector_t feetHeights_;

  // Config
  scalar_t footRadius_ = 0.02;
  scalar_t imuProcessNoisePosition_ = 0.02;
  scalar_t imuProcessNoiseVelocity_ = 0.02;
  scalar_t footProcessNoisePosition_ = 0.002;
  scalar_t footSensorNoisePosition_ = 0.005;
  scalar_t footSensorNoiseVelocity_ = 0.1;
  scalar_t footHeightSensorNoise_ = 0.01;

 private:
  size_t numContacts_, dimContacts_, numState_, numObserve_;

  matrix_t a_, b_, c_, q_, p_, r_;

  // Xhat previous = {r_3x3; v_3x3; ps_12x3}
  // ps = ee locations
  // Xhat now = {r_3x3;v_3x3; q_4x3, ps_12x3}
  // q is the quaternion
  vector_t xHat_, ps_, vs_;

  // Topic
  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2::Transform world2odom_;
  std::string frameOdom_, frameGuess_;
  bool topicUpdated_;

  grid_map::GridMap map;
  ros::Subscriber map_sub;

  int printCount = 0;

  ros::Publisher yz1;
  ros::Publisher ymodelz1;
  ros::Publisher yz2;
  ros::Publisher ymodelz2;
  ros::Publisher eyz1;
  ros::Publisher eh;

  ros::Publisher pz1;
  ros::Publisher rz;
  ros::Publisher joints;
  ros::Publisher joint_vels;

  double footCompression = 0.04;
  ros::Time contactTime[4]{};
  contact_flag_t prevContactFlag_{};
  double settleTime = 0.00;

  bool firstContactDetected[4] = {false, false, false, false};
  double swingLegBias[4] = {0.0, 0.0, 0.0, 0.0};
};

}  // namespace legged