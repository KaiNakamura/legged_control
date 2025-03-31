//
// Created by qiayuan on 2021/11/15.
//
#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>

#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged {

using namespace ocs2;
using namespace legged_robot;

class StateEstimateBase {
 public:
  StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);
  virtual void updateJointStates(const vector_t& jointPos, const vector_t& jointVel);
  virtual void updateContact(contact_flag_t contactFlag) { 

    if(contactRead){
      for(int i = 0; i < contactFlag.size(); i++){
        contactChanged_[i] = abs(contactFlag_[i] - contactFlag[i]);
      }
    }

    contactFlag_ = contactFlag; 
    contactRead = true;
  }
  virtual void updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal, const vector3_t& linearAccelLocal,
                         const matrix3_t& orientationCovariance, const matrix3_t& angularVelCovariance,
                         const matrix3_t& linearAccelCovariance);

  virtual vector_t update(const ros::Time& time, const ros::Duration& period) = 0;

  size_t getMode() { return stanceLeg2ModeNumber(contactFlag_); }

 protected:
  void updateAngular(const vector3_t& zyx, const vector_t& angularVel);
  void updateLinear(const vector_t& pos, const vector_t& linearVel);
  void publishMsgs(const nav_msgs::Odometry& odom);

  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;

  vector3_t zyxOffset_ = vector3_t::Zero();
  vector_t rbdState_;
  contact_flag_t contactFlag_{};
  contact_flag_t contactChanged_{};
  bool contactRead = false;

  Eigen::Quaternion<scalar_t> quat_;
  vector3_t angularVelLocal_, linearAccelLocal_;
  matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;

  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odomPub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>> posePub_;
  ros::Time lastPub_;
};

template <typename T>
T square(T a) {
  return a * a;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q) {
  Eigen::Matrix<SCALAR_T, 3, 1> zyx;

  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}

template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> zyxToQuat(const Eigen::Matrix<SCALAR_T, 3, 1>& zyx) {
  SCALAR_T cy = std::cos(zyx(0) * SCALAR_T(0.5));  // Yaw cosine
  SCALAR_T sy = std::sin(zyx(0) * SCALAR_T(0.5));  // Yaw sine
  SCALAR_T cp = std::cos(zyx(1) * SCALAR_T(0.5));  // Pitch cosine
  SCALAR_T sp = std::sin(zyx(1) * SCALAR_T(0.5));  // Pitch sine
  SCALAR_T cr = std::cos(zyx(2) * SCALAR_T(0.5));  // Roll cosine
  SCALAR_T sr = std::sin(zyx(2) * SCALAR_T(0.5));  // Roll sine

  Eigen::Quaternion<SCALAR_T> q;
  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;

  return q.normalized();  // Ensure it's a unit quaternion
}

}  // namespace legged
