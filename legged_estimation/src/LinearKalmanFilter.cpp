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
      numObserve_(2 * dimContacts_ + numContacts_),
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
    c_(2 * dimContacts_ + i, 6 + 3 * i + 2) = 1.0;
  }
  c_.block(0, 6, dimContacts_, dimContacts_) = -matrix_t::Identity(dimContacts_, dimContacts_);

  q_.setIdentity(numState_, numState_);
  p_ = 100. * q_;
  r_.setIdentity(numObserve_, numObserve_);
  feetHeights_.setZero(numContacts_);

  eeKinematics_->setPinocchioInterface(pinocchioInterface_);

  world2odom_.setRotation(tf2::Quaternion::getIdentity());
  sub_ = ros::NodeHandle().subscribe<nav_msgs::Odometry>("/tracking_camera/odom/sample", 10, &KalmanFilterEstimate::callback, this);

  map_sub = ros::NodeHandle().subscribe("elevation_mapping/elevation_map", 1, &KalmanFilterEstimate::getMap, this);

  heightChangeIMU = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/IMUdHeight", 10);
  heightChangeLegs = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/LegdHeight", 10);
  vz_pub = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/vz", 10);
  accz_pub = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/accz", 10);

  yz1 = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/yz1", 10);
  yz2 = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/yz2", 10);
  yz3 = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/yz3", 10);
  yz4 = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/yz4", 10);

  ymodelz1 = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/ymodelz1", 10);
  ymodelz2 = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/ymodelz2", 10);
  ymodelz3 = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/ymodelz3", 10);
  ymodelz4 = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/ymodelz4", 10);

  eyz1 = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/eyz1", 10);
  eyz2 = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/eyz2", 10);
  eyz3 = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/eyz3", 10);
  eyz4 = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/eyz4", 10);

  pz1 = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/pz1", 10);
  pz2 = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/pz2", 10);
  pz3 = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/pz3", 10);
  pz4 = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/pz4", 10);
  rz = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/rz", 10);

  heightChange = ros::NodeHandle().advertise<std_msgs::Float64>("state_estimation/heightChange", 10);
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
  r.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) =
      r_.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) * footHeightSensorNoise_;

  for (int i = 0; i < numContacts_; i++) {
    int i1 = 3 * i;

    int qIndex = 6 + i1;
    int rIndex1 = i1;
    int rIndex2 = dimContacts_ + i1;
    int rIndex3 = 2 * dimContacts_ + i;
    bool isContact = contactFlag_[i];
    bool isChangeHigh = contactChanged_[i] & contactFlag_[i];

    scalar_t high_suspect_number(1000000);
    scalar_t low_suspect_number(0.0001);
    q.block(qIndex, qIndex, 3, 3) = (isContact ? 1. : high_suspect_number) * q.block(qIndex, qIndex, 3, 3);
    r.block(rIndex1, rIndex1, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex1, rIndex1, 3, 3);
    r.block(rIndex2, rIndex2, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex2, rIndex2, 3, 3);
    r(rIndex3, rIndex3) = (isContact ? 1. : high_suspect_number) * r(rIndex3, rIndex3);

    // if(map.exists("elevation")){
    //   double planeHeight = map.atPosition("elevation", rbdState_.segment<2>(3) + ps_.segment(3 * i, 3).head<2>());
    //   if(!std::isnan(planeHeight)){
    //     if(abs(planeHeights_[i] - planeHeight) > 0.025){
    //       layerChanged_[i] = true;
    //     }
    //     if(isChangeHigh && layerChanged_[i]){
    //       layerChanged_[i] = false;

    //       r(rIndex1 + 2, rIndex1 + 2) = low_suspect_number * r(rIndex1 + 2, rIndex1 + 2);
    //       q(qIndex + 2, qIndex + 2) = low_suspect_number * q(qIndex + 2, qIndex + 2);
    //     }

    //     planeHeights_[i] = planeHeight;
    //   }
    // }

    // r(rIndex1 + 2, rIndex1 + 2) = (isChangeHigh ? 1. : low_suspect_number) * r(rIndex1 + 2, rIndex1 + 2);
    // q(qIndex + 2, qIndex + 2) = (isChangeHigh ? 1. : low_suspect_number) * q(qIndex + 2, qIndex + 2);

    ps_.segment(3 * i, 3) = -eePos[i];
    ps_.segment(3 * i, 3)[2] += footRadius_;
    vs_.segment(3 * i, 3) = -eeVel[i];
  }

  vector3_t g(0, 0, -9.81);
  vector3_t accel = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)) * linearAccelLocal_ + g;

  vector_t y(numObserve_);
  // z
  y << ps_, vs_, feetHeights_;

  // x_k|k-1 = f(x_k-1|k-1, u _k-1)
  std_msgs::Float64 heightIMU_msg;
  heightIMU_msg.data = ((a_ * xHat_ + b_ * accel) - xHat_)(2);
  heightChangeIMU.publish(heightIMU_msg);

  // std_msgs::Float64 vz_msg;
  // vz_msg.data = (a_ * xHat_ + b_ * accel)(5);
  // vz_pub.publish(vz_msg);

  // std_msgs::Float64 accz_msg;
  // accz_msg.data = accel(2);
  // accz_pub.publish(accz_msg);

  xHat_ = a_ * xHat_ + b_ * accel;
  matrix_t at = a_.transpose();

  // P = FPF^T + Q
  matrix_t pm = a_ * p_ * at + q;
  matrix_t cT = c_.transpose();
  vector_t yModel = c_ * xHat_;
  Eigen::Matrix<scalar_t, 3, 1> angVel = rbdState_.segment<3>(info_.generalizedCoordinatesNum);

  for (int i = 0; i < numContacts_; i++) {
    yModel.segment(3 * i, 3) -= dt*skewSymmetricMatrix(angVel) * (xHat_.segment(0, 3) - xHat_.segment(6 + 3 * i, 3));
  }

  // // // Convert y residual into body frame
  Eigen::Matrix<scalar_t, 3, 1> orientation = rbdState_.head<3>();
  matrix_t rot = getRotationMatrixFromZyxEulerAngles(orientation).transpose();
  for (int i = 0; i < numContacts_; i++) {
    y.segment(3 * i, 3) = rot * y.segment(3 * i, 3);
    // yModel.segment(3 * i + dimContacts_, 3) = rot.transpose() * yModel.segment(3 * i, 3);
    // ey(3*i + 2) = -ey(3*i + 2);
    // ey(3 * i + dimContacts_ + 2) = -ey(3 * i + dimContacts_ + 2);
  }

  // std_msgs::Float64 yz1_msg;
  // yz1_msg.data = y(2);
  // yz1.publish(yz1_msg);

  // std_msgs::Float64 yz2_msg;
  // yz2_msg.data = y(5);
  // yz2.publish(yz2_msg);

  // std_msgs::Float64 yz3_msg;
  // yz3_msg.data = y(8);
  // yz3.publish(yz3_msg);

  // std_msgs::Float64 yz4_msg;
  // yz4_msg.data = y(11);
  // yz4.publish(yz4_msg);

  // std_msgs::Float64 ymodelz1_msg;
  // ymodelz1_msg.data = yModel(2);
  // ymodelz1.publish(ymodelz1_msg);
  
  // std_msgs::Float64 ymodelz2_msg;
  // ymodelz2_msg.data = yModel(5);
  // ymodelz2.publish(ymodelz2_msg);
  
  // std_msgs::Float64 ymodelz3_msg;
  // ymodelz3_msg.data = yModel(8);
  // ymodelz3.publish(ymodelz3_msg);
  
  // std_msgs::Float64 ymodelz4_msg;
  // ymodelz4_msg.data = yModel(11);
  // ymodelz4.publish(ymodelz4_msg);
  // y = z - h(x_k|k-1)
  vector_t ey = y - yModel;
  // std_msgs::Float64 eyz1_msg;
  // eyz1_msg.data = ey(2);
  // eyz1.publish(eyz1_msg);

  // std_msgs::Float64 eyz2_msg;
  // eyz2_msg.data = ey(5);
  // eyz2.publish(eyz2_msg);

  // std_msgs::Float64 eyz3_msg;
  // eyz3_msg.data = ey(8);
  // eyz3.publish(eyz3_msg);

  // std_msgs::Float64 eyz4_msg;
  // eyz4_msg.data = ey(11);
  // eyz4.publish(eyz4_msg);
  
  // S = HPH^T + R
  matrix_t s = c_ * pm * cT + r;

  // std::cout << "rot: " << rot << std::endl;
  // std::cout << "orientation: " << rbdState_.head<3>().transpose() << std::endl;
  // std::cout << "y: " << y.transpose() << std::endl;
  // std::cout << "y model: " << yModel.transpose() << std::endl;
  // std::cout << "r model: " << (c_ * xHat_).transpose() << std::endl;
  // std::cout << "p model: " << yModel.transpose() << std::endl;
  // std::cout << "ey: " << ey.transpose() << std::endl;
  // std::cout << "contact flag: " << contactFlag_[0] << " " << contactFlag_[1] << " " << contactFlag_[2] << " " << contactFlag_[3] << std::endl;
  // std::cout << "contact change: " << contactChanged_[0] << " " << contactChanged_[1] << " " << contactChanged_[2] << " " << contactChanged_[3] << std::endl;
  // std::cout << "layer change: " << layerChanged_[0] << " " << layerChanged_[1] << " " << layerChanged_[2] << " " << layerChanged_[3] << std::endl;
  // if(map.exists("segmentation")){
  //   std::cout << "segmentation: " << map.get("segmentation") << std::endl;
  // }
  // std::cout << "xhat: " << xHat_.transpose() << std::endl;

  // Combined equivalent to x_k = x_k-1 + Ky = x_k-1 + PH^TS^-iy
  vector_t sEy = s.lu().solve(ey);
  // std::cout << "Sey: " << std::endl << sEy.transpose() << std::endl;
  // std::cout << "pm: " << std::endl << pm << std::endl;
  // std::cout << "cT: " << std::endl << cT << std::endl;
  vector_t Ky = pm * cT * sEy;
  // Ky.head(6).setZero();
  // std::cout << "Ky: " << Ky.transpose() << std::endl;
  // Ky(2) = -Ky(2);
  xHat_ += Ky;
  for (int i = 0; i < numContacts_; i++) {

  }
  // std::cout << "xHat: " << xHat_.transpose() << std::endl;

  std_msgs::Float64 pz1_msg;
  pz1_msg.data = xHat_(8);
  pz1.publish(pz1_msg);
  
  std_msgs::Float64 pz2_msg;
  pz2_msg.data = xHat_(11);
  pz2.publish(pz2_msg);  
  
  std_msgs::Float64 pz3_msg;
  pz3_msg.data = xHat_(14);
  pz3.publish(pz3_msg);  
  
  std_msgs::Float64 pz4_msg;
  pz4_msg.data = xHat_(17);
  pz4.publish(pz4_msg);  
  
  std_msgs::Float64 rz_msg;
  rz_msg.data = xHat_(2);
  rz.publish(rz_msg);

  std_msgs::Float64 heightLeg_msg;
  heightLeg_msg.data = Ky(2);
  heightChangeLegs.publish(heightLeg_msg);

  // Combined equivalent to P_k = (I-KH)P_k-1
  matrix_t sC = s.lu().solve(c_);
  p_ = (matrix_t::Identity(numState_, numState_) - pm * cT * sC) * pm;

  matrix_t pt = p_.transpose();
  p_ = (p_ + pt) / 2.0;

  //  if (p_.block(0, 0, 2, 2).determinant() > 0.000001) {
  //    p_.block(0, 2, 2, 16).setZero();
  //    p_.block(2, 0, 16, 2).setZero();
  //    p_.block(0, 0, 2, 2) /= 10.;
  //  }

  // std_msgs::Float64 heightChange_msg;
  // heightChange_msg.data = heightIMU_msg.data + heightLeg_msg.data;
  // heightChange.publish(heightChange_msg);

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
  auto* msg = buffer_.readFromRT();

  tf2::Transform world2sensor;
  world2sensor.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  world2sensor.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                                           msg->pose.pose.orientation.w));

  if (world2odom_.getRotation() == tf2::Quaternion::getIdentity())  // First received
  {
    tf2::Transform odom2sensor;
    try {
      geometry_msgs::TransformStamped tf_msg = tfBuffer_.lookupTransform("odom", msg->child_frame_id, msg->header.stamp);
      tf2::fromMsg(tf_msg.transform, odom2sensor);
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      return;
    }
    world2odom_ = world2sensor * odom2sensor.inverse();
  }
  tf2::Transform base2sensor;
  try {
    geometry_msgs::TransformStamped tf_msg = tfBuffer_.lookupTransform("base", msg->child_frame_id, msg->header.stamp);
    tf2::fromMsg(tf_msg.transform, base2sensor);
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  tf2::Transform odom2base = world2odom_.inverse() * world2sensor * base2sensor.inverse();
  vector3_t newPos(odom2base.getOrigin().x(), odom2base.getOrigin().y(), odom2base.getOrigin().z());

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  vector_t qPino(info_.generalizedCoordinatesNum);
  qPino.head<3>() = newPos;
  qPino.segment<3>(3) = rbdState_.head<3>();
  qPino.tail(info_.actuatedDofNum) = rbdState_.segment(6, info_.actuatedDofNum);
  pinocchio::forwardKinematics(model, data, qPino);
  pinocchio::updateFramePlacements(model, data);

  xHat_.segment<3>(0) = newPos;
  for (size_t i = 0; i < numContacts_; ++i) {
    xHat_.segment<3>(6 + i * 3) = eeKinematics_->getPosition(vector_t())[i];
    xHat_(6 + i * 3 + 2) -= footRadius_;
    if (contactFlag_[i]) {
      feetHeights_[i] = xHat_(6 + i * 3 + 2);
    }
  }

  auto odom = getOdomMsg();
  odom.header = msg->header;
  odom.child_frame_id = "base";
  publishMsgs(odom);
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