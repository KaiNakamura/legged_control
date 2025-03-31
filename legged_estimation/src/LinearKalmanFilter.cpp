//
// Created by qiayuan on 2022/7/24.
//

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_estimation/LinearKalmanFilter.h"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace legged {
// 
KalmanFilterEstimate::KalmanFilterEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                           const PinocchioEndEffectorKinematics& eeKinematics)
    : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics),
      numContacts_(info_.numThreeDofContacts + info_.numSixDofContacts),
      dimContacts_(3 * numContacts_),
      numState_(6 + dimContacts_),
      numObserve_(2 * dimContacts_),
      tfListener_(tfBuffer_),
      topicUpdated_(false) {
  xHat_.setZero(numState_);
  ps_.setZero(dimContacts_);
  vs_.setZero(dimContacts_);
  a_.setIdentity(numState_, numState_);
  b_.setZero(numState_, 3);
  matrix_t c1(3, 6), c2(3, 6);
  c1 << matrix3_t::Identity(), matrix3_t::Zero();
  c2 << matrix3_t::Zero(), matrix3_t::Identity();
  c_.setZero(numObserve_, numState_);
  for (ssize_t i = 0; i < numContacts_; ++i) {
    c_.block(3 * i, 0, 3, 6) = c1;
    c_.block(3 * (numContacts_ + i), 0, 3, 6) = c2;
  }
  c_.block(0, 6, dimContacts_, dimContacts_) = -matrix_t::Identity(dimContacts_, dimContacts_);

  q_.setIdentity(numState_, numState_);
  p_ = 100. * q_;
  r_.setIdentity(numObserve_, numObserve_);
  feetHeights_.setZero(numContacts_);

  eeKinematics_->setPinocchioInterface(pinocchioInterface_);

  world2odom_.setRotation(tf2::Quaternion::getIdentity());
  sub_ = ros::NodeHandle().subscribe<nav_msgs::Odometry>("/laser_odometry", 10, &KalmanFilterEstimate::callback, this);

  map_sub = ros::NodeHandle().subscribe("elevation_mapping/elevation_map", 1, &KalmanFilterEstimate::getMap, this);
  joints = ros::NodeHandle().advertise<std_msgs::Float64MultiArray>("state_estimation/joints", 10);
}

vector_t KalmanFilterEstimate::update(const ros::Time& time, const ros::Duration& period) {
  scalar_t dt = period.toSec();
  a_.block(0, 3, 3, 3) = dt * matrix3_t::Identity();
  b_.block(0, 0, 3, 3) = 0.5 * dt * dt * matrix3_t::Identity();
  b_.block(3, 0, 3, 3) = dt * matrix3_t::Identity();
  q_.block(0, 0, 3, 3) = (dt / 20.f) * matrix3_t::Identity();
  q_.block(3, 3, 3, 3) = (dt * 9.81f) * matrix3_t::Identity();
  q_.block(6, 6, dimContacts_, dimContacts_) = dt * matrix_t::Identity(dimContacts_, dimContacts_);

  // for (ssize_t i = 0; i < numContacts_; ++i) {
  //   c_.block(3 * i, 3, 3, 3) = dt * matrix3_t::Identity();
  // }

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  size_t actuatedDofNum = info_.actuatedDofNum;

  vector_t qPino(info_.generalizedCoordinatesNum);
  vector_t vPino(info_.generalizedCoordinatesNum);
  qPino.setZero();
  qPino.segment<3>(3) = rbdState_.head<3>();  // Only set orientation, let position in origin.
  qPino.tail(actuatedDofNum) = rbdState_.segment(6, actuatedDofNum);

  std_msgs::Float64MultiArray joint_msg;
  double joint_values[actuatedDofNum];
  joint_msg.data.resize(actuatedDofNum);
  std::copy(rbdState_.segment(6, actuatedDofNum).data(), rbdState_.segment(6, actuatedDofNum).data() + actuatedDofNum, joint_msg.data.begin());
  joints.publish(joint_msg);

  vPino.setZero();
  vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qPino.segment<3>(3),
      rbdState_.segment<3>(info_.generalizedCoordinatesNum));  // Only set angular velocity, let linear velocity be zero
  vPino.tail(actuatedDofNum) = rbdState_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);

  pinocchio::forwardKinematics(model, data, qPino, vPino);
  pinocchio::updateFramePlacements(model, data);

  const auto eePos = eeKinematics_->getPosition(vector_t());
  const auto eeVel = eeKinematics_->getVelocity(vector_t(), vector_t());

  matrix_t q = matrix_t::Identity(numState_, numState_);
  q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * imuProcessNoisePosition_;
  q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * imuProcessNoiseVelocity_;
  q.block(6, 6, dimContacts_, dimContacts_) = q_.block(6, 6, dimContacts_, dimContacts_) * footProcessNoisePosition_;

  matrix_t r = matrix_t::Identity(numObserve_, numObserve_);
  r.block(0, 0, dimContacts_, dimContacts_) = r_.block(0, 0, dimContacts_, dimContacts_) * footSensorNoisePosition_;
  r.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) =
      r_.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) * footSensorNoiseVelocity_;

  for (int i = 0; i < numContacts_; i++) {
    int i1 = 3 * i;

    int qIndex = 6 + i1;
    int rIndex1 = i1;
    int rIndex2 = dimContacts_ + i1;
    int rIndex3 = 2 * dimContacts_ + i;
    bool isContact = contactFlag_[i];

    if(contactChanged_[i]){
      contactTime[i] = time;
    }

    ros::Duration elapsedTime = (time - contactTime[i]);
    double timeSinceContact = elapsedTime.toSec();
    if(timeSinceContact < settleTime){
      isContact = !isContact;
    }

    scalar_t high_suspect_number(1000000);
    scalar_t low_suspect_number(0.0001);
    q.block(qIndex, qIndex, 3, 3) = (isContact ? 1. : high_suspect_number) * q.block(qIndex, qIndex, 3, 3);
    r.block(rIndex1, rIndex1, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex1, rIndex1, 3, 3);
    r.block(rIndex2, rIndex2, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex2, rIndex2, 3, 3);

    ps_.segment(3 * i, 3) = -eePos[i];
    ps_.segment(3 * i, 3)[2] += footRadius_;
    vs_.segment(3 * i, 3) = -eeVel[i];

  }

  vector3_t g(0, 0, -9.81);
  vector3_t accel = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)) * linearAccelLocal_ + g;

  vector_t y(numObserve_);
  // z
  y << ps_, vs_;

  // x_k|k-1 = f(x_k-1|k-1, u _k-1)
  xHat_ = a_ * xHat_ + b_ * accel;
  matrix_t at = a_.transpose();

  // P = FPF^T + Q
  matrix_t pm = a_ * p_ * at + q;
  matrix_t cT = c_.transpose(); 
  vector_t yModel = c_ * xHat_;

  // y = z - h(x_k|k-1)
  vector_t ey = y - yModel;

  // S = HPH^T + R
  matrix_t s = c_ * pm * cT + r;

  // Combined equivalent to x_k = x_k-1 + Ky = x_k-1 + PH^TS^-iy
  vector_t sEy = s.lu().solve(ey);
  vector_t Ky = pm * cT * s.completeOrthogonalDecomposition().pseudoInverse() * ey;
  xHat_ += Ky;

  // Combined equivalent to P_k = (I-KH)P_k-1
  matrix_t sC = s.lu().solve(c_);
  p_ = (matrix_t::Identity(numState_, numState_) - pm * cT * s.completeOrthogonalDecomposition().pseudoInverse() * c_) * pm;

  matrix_t pt = p_.transpose();
  p_ = (p_ + pt) / 2.0;

  if (topicUpdated_) {
    updateFromTopic();
    topicUpdated_ = false;
  }

  updateLinear(xHat_.segment<3>(0), xHat_.segment<3>(3));

  auto odom = getOdomMsg();
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base";
  publishMsgs(odom);

  return rbdState_;
}

void KalmanFilterEstimate::updateFromTopic() {
  // Attempt to read an Odometry message
  auto* msg = buffer_.readFromRT();
  if (!msg) return;  // No new message available

  //------------------------------------------------------------------------------
  // 1) Build the world->sensor TF from the Odometry message
  //------------------------------------------------------------------------------
  tf2::Transform world2sensor;
  world2sensor.setOrigin(tf2::Vector3(msg->pose.pose.position.x,
                                      msg->pose.pose.position.y,
                                      msg->pose.pose.position.z));
  world2sensor.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x,
                                           msg->pose.pose.orientation.y,
                                           msg->pose.pose.orientation.z,
                                           msg->pose.pose.orientation.w));

  // If this is our first callback and we haven't computed world2odom_ yet:
  if (world2odom_.getRotation() == tf2::Quaternion::getIdentity()) {
    tf2::Transform odom2sensor;
    try {
      geometry_msgs::TransformStamped tf_msg = tfBuffer_.lookupTransform("odom", "lidar_imu_frame", ros::Time(0));
      tf2::fromMsg(tf_msg.transform, odom2sensor);
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      return;
    }
    world2odom_ = world2sensor * odom2sensor.inverse();
  }

  //------------------------------------------------------------------------------
  // 2) We get odom->base by combining transforms
  //    This is effectively the measured base pose in the "odom" frame.
  //------------------------------------------------------------------------------
  tf2::Transform base2sensor;
  try {
    // This is the frame from the URDF, not the one that fast-lio is publishing.
    geometry_msgs::TransformStamped tf_msg = tfBuffer_.lookupTransform("base", "lidar_imu_frame", ros::Time(0));
    tf2::fromMsg(tf_msg.transform, base2sensor);
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  tf2::Transform odom2base = world2odom_.inverse() * world2sensor * base2sensor.inverse();

  //------------------------------------------------------------------------------
  // 3) POSITION UPDATE (same idea as before)
  //    We'll treat (x, y, z) from odometry as a measurement for the first 3 entries of xHat_.
  //------------------------------------------------------------------------------
  // Extract measured base position in 'odom' frame
  Eigen::Vector3d posOdom(odom2base.getOrigin().x(),
                          odom2base.getOrigin().y(),
                          odom2base.getOrigin().z());

  // The measurement model H only updates the position portion of xHat_.
  //  xHat_ = [ px, py, pz, vx, vy, vz, footPositions(12) ]
  // So:
  Eigen::Matrix<double, 3, 18> H;
  H.setZero();
  H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();

  // Covariance of the odometry position. 
  // We'll read it from the top-left 3x3 of msg->pose.covariance:
  // The row-major indexing for a 6x6 is [i*6 + j], i=0..5, j=0..5
  // where i=0..2, j=0..2 covers position. 
  Eigen::Matrix3d Rpos;
  Rpos << msg->pose.covariance[0], msg->pose.covariance[1], msg->pose.covariance[2],
          msg->pose.covariance[6], msg->pose.covariance[7], msg->pose.covariance[8],
          msg->pose.covariance[12],msg->pose.covariance[13],msg->pose.covariance[14];

  // Fast-LIO has higher covariance, so let's make Rpos a bit bigger
  // Innovation (residual)
  Eigen::Vector3d y = posOdom - H * xHat_;

  // Kalman gain
  Eigen::Matrix3d S = H * p_ * H.transpose() + Rpos;
  Eigen::Matrix<double, 18, 3> K = p_ * H.transpose() * S.inverse();

  // Correction
  xHat_ += K * y;
  p_ = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * p_;

  // Force p_ to remain symmetric
  p_ = 0.5 * (p_ + p_.transpose());

  //------------------------------------------------------------------------------
  // 4) ORIENTATION UPDATE
  //
  // Because our filter state 'xHat_' does NOT store orientation, we do
  // a *separate* small-angle (or Euler) correction for the quaternion 'quat_'.
  // We'll treat 'quat_' as a 3D state in Euler angles (or small rotation vector)
  // and do a standard linear update with the odometry's orientation as a measurement.
  //------------------------------------------------------------------------------

  // (a) Convert the filter's orientation (IMU-based) to Euler angles
  //     Let's assume ZYX order or whichever you typically use:
  Eigen::Vector3d eulFilter = quatToZyx(quat_);  // You may have a utility for this

  // (b) Convert odometry orientation to Euler
  tf2::Quaternion qOdom(odom2base.getRotation());
  // Make sure it is normalized:
  if (fabs(qOdom.length() - 1.0) > 1e-6) {
    qOdom.normalize();
  }
  // Convert to ZYX euler
  double roll, pitch, yaw;
  tf2::Matrix3x3(qOdom).getRPY(roll, pitch, yaw);
  Eigen::Vector3d eulOdom(yaw, pitch, roll);  // Be consistent with your ZYX usage!

  // (c) Orientation covariance from the bottom-right 3x3 of the 6x6 pose covariance
  // i=3..5, j=3..5 => row-major indices [ (3)*6+3, (3)*6+4, ... etc. ] 
  // That is [21, 22, 23, 27, 28, 29, 33, 34, 35].
  // We'll store it in Rori:
  Eigen::Matrix3d Rori;
  Rori << msg->pose.covariance[21], msg->pose.covariance[22], msg->pose.covariance[23],
          msg->pose.covariance[27], msg->pose.covariance[28], msg->pose.covariance[29],
          msg->pose.covariance[33], msg->pose.covariance[34], msg->pose.covariance[35];

  // (d) We keep orientationCovariance_ as a 3x3 capturing our filter's uncertainty
  // about orientation. The measurement model here is simply identity: eul_meas = eul_filter
  Eigen::Matrix3d Hori = Eigen::Matrix3d::Identity();

  // (e) Innovation
  // We want eulOdom - eulFilter, but remember angles can wrap, so you might want
  // to do a small wrap-around step. For simplicity, just do a naive difference here:
  Eigen::Vector3d yori = eulOdom - eulFilter;
  // Optionally wrap each component into [-pi, pi], if you expect big differences.

  // (f) Kalman gain for orientation
  Eigen::Matrix3d Sori = Hori * orientationCovariance_ * Hori.transpose() + Rori;
  Eigen::Matrix3d Kori = orientationCovariance_ * Hori.transpose() * Sori.inverse();

  // (g) Correct the Euler angles
  eulFilter += Kori * yori;

  // (h) Update orientation covariance
  orientationCovariance_ = (Eigen::Matrix3d::Identity() - Kori * Hori) * orientationCovariance_;
  orientationCovariance_ = 0.5 * (orientationCovariance_ + orientationCovariance_.transpose());

  // (i) Convert corrected Euler angles back to the quaternion
  //     E.g., something like:
  quat_ = zyxToQuat(eulFilter);  // or your favorite internal function

  //------------------------------------------------------------------------------
  // 5) Done with position+orientation measurement updates.
  //    Now we can publish an updated Odometry (for debugging or further usage).
  //------------------------------------------------------------------------------
  auto odom = getOdomMsg();  // This uses xHat_ + quat_
  odom.header = msg->header;
  odom.child_frame_id = "base";
  publishMsgs(odom);

  // Mark that we handled an update
  topicUpdated_ = false;
}

void KalmanFilterEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg) {
  buffer_.writeFromNonRT(*msg);
  topicUpdated_ = true;
}

nav_msgs::Odometry KalmanFilterEstimate::getOdomMsg() {
  nav_msgs::Odometry odom;
  odom.pose.pose.position.x = xHat_.segment<3>(0)(0);
  odom.pose.pose.position.y = xHat_.segment<3>(0)(1);
  odom.pose.pose.position.z = xHat_.segment<3>(0)(2);
  odom.pose.pose.orientation.x = quat_.x();
  odom.pose.pose.orientation.y = quat_.y();
  odom.pose.pose.orientation.z = quat_.z();
  odom.pose.pose.orientation.w = quat_.w();
  odom.pose.pose.orientation.x = quat_.x();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      odom.pose.covariance[i * 6 + j] = p_(i, j);
      odom.pose.covariance[6 * (3 + i) + (3 + j)] = orientationCovariance_(i * 3 + j);
    }
  }
  //  The twist in this message should be specified in the coordinate frame given by the child_frame_id: "base"
  vector_t twist = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)).transpose() * xHat_.segment<3>(3);
  odom.twist.twist.linear.x = twist.x();
  odom.twist.twist.linear.y = twist.y();
  odom.twist.twist.linear.z = twist.z();
  odom.twist.twist.angular.x = angularVelLocal_.x();
  odom.twist.twist.angular.y = angularVelLocal_.y();
  odom.twist.twist.angular.z = angularVelLocal_.z();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      odom.twist.covariance[i * 6 + j] = p_.block<3, 3>(3, 3)(i, j);
      odom.twist.covariance[6 * (3 + i) + (3 + j)] = angularVelCovariance_(i * 3 + j);
    }
  }
  return odom;
}

void KalmanFilterEstimate::loadSettings(const std::string& taskFile, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "kalmanFilter.";
  if (verbose) {
    std::cerr << "\n #### Kalman Filter Noise:";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, footRadius_, prefix + "footRadius", verbose);
  loadData::loadPtreeValue(pt, imuProcessNoisePosition_, prefix + "imuProcessNoisePosition", verbose);
  loadData::loadPtreeValue(pt, imuProcessNoiseVelocity_, prefix + "imuProcessNoiseVelocity", verbose);
  loadData::loadPtreeValue(pt, footProcessNoisePosition_, prefix + "footProcessNoisePosition", verbose);
  loadData::loadPtreeValue(pt, footSensorNoisePosition_, prefix + "footSensorNoisePosition", verbose);
  loadData::loadPtreeValue(pt, footSensorNoiseVelocity_, prefix + "footSensorNoiseVelocity", verbose);
  loadData::loadPtreeValue(pt, footHeightSensorNoise_, prefix + "footHeightSensorNoise", verbose);
}

void KalmanFilterEstimate::getMap(const grid_map_msgs::GridMap& msg){
  grid_map::GridMapRosConverter::fromMessage(msg, map);
}
}  // namespace legged