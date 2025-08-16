//
// Created by qiayuan on 2022/7/24.
//

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include "legged_estimation/ContactEstimate.h"

#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

namespace legged {

ContactEstimate::ContactEstimate(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                                            const PinocchioEndEffectorKinematics& eeKinematics): 
                                            pinocchioInterface_(pinocchioInterface),
                                            pinocchioInterfaceBody_(pinocchioInterface),
                                            pinocchioInterfaceDesired_(pinocchioInterface),
                                            info_(std::move(info)),
                                            mapping_(info_),
                                            eeKinematics_(eeKinematics.clone()),
                                            rbdState_(vector_t ::Zero(2 * info_.generalizedCoordinatesNum)){
  eeKinematics_->setPinocchioInterface(pinocchioInterface_);

  world2odom_.setRotation(tf2::Quaternion::getIdentity());

  const auto& model = pinocchioInterface_.getModel();
  
  // Make all disturbance vectors of size specified by model
  tau_filtered_prev = Eigen::MatrixXd::Zero(model.nv, 1);
  tau_filtered_curr = Eigen::MatrixXd::Zero(model.nv, 1);
  tau_unfiltered_prev = Eigen::MatrixXd::Zero(model.nv, 1);
  tau_measured_prev = Eigen::MatrixXd::Zero(model.nv, 1);
  tau_measured_curr = Eigen::MatrixXd::Zero(model.nv, 1);

  p_prev = Eigen::MatrixXd::Zero(model.nv, 1);

  qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  vMeasured_ = vector_t(info_.generalizedCoordinatesNum);

  ros::NodeHandle nh;

  // kalman_variance_force = getKalmanProbability(mean_force, contact_mean_force, contact_variance_force);
  // kalman_variance_force_sensors = getKalmanProbability(mean_force_sensor, contact_mean_force_sensor, contact_variance_force_sensor);

  leg_contact_pub = nh.advertise<std_msgs::Int32MultiArray>("contact_estimation/leg_contact", 10);
  leg_contact_prob_pub = nh.advertise<std_msgs::Float64MultiArray>("contact_estimation/leg_contact_prob", 10);
  leg_contact_prob_force_pub = nh.advertise<std_msgs::Float64MultiArray>("contact_estimation/leg_contact_force_prob", 10);
  leg_contact_prob_time_pub = nh.advertise<std_msgs::Float64MultiArray>("contact_estimation/leg_contact_time_prob", 10);
  leg_contact_prob_height_pub = nh.advertise<std_msgs::Float64MultiArray>("contact_estimation/leg_contact_height_prob", 10);
  leg_contact_prob_velocity_pub = nh.advertise<std_msgs::Float64MultiArray>("contact_estimation/leg_contact_velocity_prob", 10);
  leg_contact_prob_force_sensors_pub = nh.advertise<std_msgs::Float64MultiArray>("contact_estimation/leg_contact_force_sensors_prob", 10);
  leg_height_pub = nh.advertise<std_msgs::Float64MultiArray>("contact_estimation/leg_height", 10);
  leg_variance_pub = nh.advertise<std_msgs::Float64MultiArray>("contact_estimation/leg_variance", 10);
  leg_foothold_pub = nh.advertise<std_msgs::Float64MultiArray>("contact_estimation/leg_terrain", 10);
  leg_force_pub = nh.advertise<std_msgs::Float64MultiArray>("contact_estimation/leg_force", 10);
  leg_force_sensor_pub = nh.advertise<std_msgs::Float64MultiArray>("contact_estimation/leg_force_sensor", 10);
  leg_velocity_pub = nh.advertise<std_msgs::Float64MultiArray>("contact_estimation/leg_velocity", 10);
  map_debug_pub = nh.advertise<std_msgs::Float64MultiArray>("contact_estimation/map_debug", 10);
  map_debug2_pub = nh.advertise<std_msgs::Float64MultiArray>("contact_estimation/map_debug2", 10);

  // joint_state_sub = nh.subscribe("/unitree_hardware/joint_foot", 1, &ContactEstimate::getForceReadings, this);
  map_sub = nh.subscribe("elevation_mapping/elevation_map", 1, &ContactEstimate::getMap, this);
}

size_t ContactEstimate::update(scalar_t time, const ros::Duration& period, vector_t desiredState, vector_t input, const vector_t& rbdStateMeasured, vector_t torque, vector_t sensorForces, ModeSchedule modeSchedule_) {
  scalar_t dt = period.toSec();
  // Get state joint measurements from RBD
  qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);
  qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
  qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);
  vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
  vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
  vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);
  
  // Get pinocchio current model and data
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  // set up and get M(q_dot, q), C(q), g
  Eigen::MatrixXd M(info_.generalizedCoordinatesNum, info_.generalizedCoordinatesNum);
  Eigen::MatrixXd C(info_.generalizedCoordinatesNum, info_.generalizedCoordinatesNum);
  Eigen::MatrixXd g(info_.generalizedCoordinatesNum, 1);
  Eigen::MatrixXd p(info_.generalizedCoordinatesNum, 1);

  double zDomainCutoff = exp(-frequencyCutoff * dt); // gamma in paper, given by e^(-lambda*dt), currently using average of 0.001
  double beta = (1 - zDomainCutoff)/zDomainCutoff/dt;

  // Update algorithm, may be inefficient as this is also run in WbcBase
  M = pinocchio::crba(model, data, qMeasured_);
  C = pinocchio::computeCoriolisMatrix(model, data, qMeasured_, vMeasured_);
  g = pinocchio::computeGeneralizedGravity(model, data, qMeasured_);
  p = M * vMeasured_;

  // Calculate jacobian for applied forces
  pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::crba(model, data, qMeasured_);

  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);
  matrix_t j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }

  // Calculate selection matrix
  Eigen::MatrixXd ST = Eigen::MatrixXd::Zero(info_.generalizedCoordinatesNum, 3 * info_.numThreeDofContacts);
  ST.block(6, 0, 3*info_.numThreeDofContacts, 3*info_.numThreeDofContacts).setIdentity();
  
  // Update in zeta domain
  tau_measured_curr = beta*p + ST*torque + C.transpose()*vMeasured_ - g;
  tau_filtered_curr = beta*p - zDomainCutoff*beta*p_prev -(1 - zDomainCutoff)*tau_measured_curr + zDomainCutoff*tau_measured_prev; //Note: I am not 100% confident on the signs here

  // Calculate the estimation of the applied force... also make it negative idk why it came out all the wrong sign
  Eigen::MatrixXd force_estimated = -(ST.transpose()*j_.transpose()).partialPivLu().solve(ST.transpose()*tau_filtered_curr);
  if(std::isnan(force_estimated(0)) || std::isinf(force_estimated(0))){
    force_estimated = -(ST.transpose()*j_.transpose()).completeOrthogonalDecomposition().pseudoInverse()*ST.transpose()*tau_filtered_curr;
  }

  // update previous taus
  tau_filtered_prev = tau_filtered_curr;
  tau_measured_prev = tau_measured_curr;
  p_prev = p;

  // Begin Kalman correction measurement model (update phase)
  eeKinematics_->setPinocchioInterface(pinocchioInterface_);
  std::vector<vector3_t> footPos = eeKinematics_->getPosition(vector_t()); // I hate this notation but it's native to pinocchio interface
  
  // Get current foot velocity in body frame
  const auto& modelBody = pinocchioInterfaceBody_.getModel();
  auto& dataBody = pinocchioInterfaceBody_.getData();

  vector_t qBody(info_.generalizedCoordinatesNum), vBody(info_.generalizedCoordinatesNum);
  qBody = qMeasured_;
  qBody.head(3).setZero(); // only set angle
  vBody = vMeasured_;
  vBody.head(3).setZero(); // only set angular velocity

  pinocchio::forwardKinematics(modelBody, dataBody, qBody, vBody);
  pinocchio::updateFramePlacements(modelBody, dataBody);
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceBody_);
  std::vector<vector3_t> footVelBodyMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());

  // Get desired foot velocity in body frame
  const auto& modelDesired = pinocchioInterfaceDesired_.getModel();
  auto& dataDesired = pinocchioInterfaceDesired_.getData();

  mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
  vector_t qBodyDesired = mapping_.getPinocchioJointPosition(desiredState);
  pinocchio::forwardKinematics(modelDesired, dataDesired, qBodyDesired);
  pinocchio::computeJointJacobians(modelDesired, dataDesired, qBodyDesired);
  pinocchio::updateFramePlacements(modelDesired, dataDesired);
  updateCentroidalDynamics(pinocchioInterfaceDesired_, info_, qBodyDesired);
  vector_t vBodyDesired = mapping_.getPinocchioJointVelocity(desiredState, input);

  qBodyDesired.head(3).setZero();
  vBodyDesired.head(3).setZero();
  pinocchio::forwardKinematics(modelDesired, dataDesired, qBodyDesired, vBodyDesired);
  pinocchio::updateFramePlacements(modelDesired, dataDesired);
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
  std::vector<vector3_t> footVelBodyDesired = eeKinematics_->getVelocity(vector_t(), vector_t());

  if(map.exists("elevation")){
    std::vector<Eigen::MatrixXd> weightedHeight = sampleHeights(footPos, 0);
    std::vector<Eigen::MatrixXd> weightedVariance = weightVariances(footPos, 3);

    for(int i = 0; i < info_.numThreeDofContacts; i++){
      mean_zg[i] = weightedHeight[0](i);
      variance_zg[i] = joint_variance + weightedVariance[0](i);

      kalman_map_variance[i] = weightedVariance[0](i);
    }
  }

  // Begin kalman filtering
  // Begin time-based kalman filter (probability phase)
  Eigen::MatrixXd contact_probability_time = Eigen::MatrixXd(info_.numThreeDofContacts, 1);
  Eigen::MatrixXd contact_variance_time = Eigen::MatrixXd(info_.numThreeDofContacts, info_.numThreeDofContacts);
  Eigen::MatrixXd contact_probability_height = Eigen::MatrixXd(info_.numThreeDofContacts, 1);
  Eigen::MatrixXd contact_variance_height = Eigen::MatrixXd(info_.numThreeDofContacts, 1);
  Eigen::MatrixXd contact_probability_force = Eigen::MatrixXd(info_.numThreeDofContacts, 1);
  Eigen::MatrixXd contact_variance_force = Eigen::MatrixXd(info_.numThreeDofContacts, 1);
  Eigen::MatrixXd contact_probability_force_sensor = Eigen::MatrixXd(info_.numThreeDofContacts, 1);
  Eigen::MatrixXd contact_variance_force_sensor = Eigen::MatrixXd(info_.numThreeDofContacts, 1);
  Eigen::MatrixXd contact_probability_velocity = Eigen::MatrixXd(info_.numThreeDofContacts, 1);
  Eigen::MatrixXd contact_variance_velocity = Eigen::MatrixXd(info_.numThreeDofContacts, 1);

  for(int i = 0; i < info_.numThreeDofContacts; i++){
    contact_probability_time(i) = calculateContactProbabilityTime(modeNumber2StanceLeg(modeSchedule_.modeAtTime(time))[i], modeSchedule_.percentageAtTime(time));
    contact_variance_time(i,i) = kalman_variance_time + map_variance_weighting*kalman_map_variance[i];

    contact_probability_height(i) = calculateContactProbabilityFootHeight(footPos[i](2), i);
    contact_variance_height(i) = kalman_variance_height[i] + map_variance_weighting*kalman_map_variance[i];

    contact_probability_force(i) = calculateContactProbabilityFootForce(modeNumber2StanceLeg(modeSchedule_.modeAtTime(time))[i], force_estimated(2 + 3*i), input(2 + 3*i));
    contact_variance_force(i) = kalman_variance_force + map_variance_weighting*kalman_map_variance[i];

    contact_probability_force_sensor(i) = calculateContactProbabilityForceSensor(sensorForces(i));
    contact_variance_force_sensor(i) = kalman_variance_force_sensors - map_variance_weighting*kalman_map_variance[i];

    contact_probability_velocity(i) = calculateContactProbabilityFootVelocity(modeNumber2StanceLeg(modeSchedule_.modeAtTime(time))[i], footVelBodyDesired[i] - footVelBodyMeasured[i]);
    contact_variance_velocity(i) = kalman_variance_velocity - map_variance_weighting*kalman_map_variance[i];
  }

  Eigen::MatrixXd contact_probability_overall = Eigen::MatrixXd(info_.numThreeDofContacts, 1);
  Eigen::MatrixXd correction_probabilities = Eigen::MatrixXd(4*info_.numThreeDofContacts, 1);
  Eigen::VectorXd correction_variances = Eigen::VectorXd(4*info_.numThreeDofContacts, 1);
  correction_probabilities << contact_probability_height, contact_probability_force, contact_probability_force_sensor, contact_probability_velocity;
  correction_variances << contact_variance_height, contact_variance_force, contact_variance_force_sensor, contact_variance_velocity;
  contact_probability_overall = KalmanCorrection(4, correction_variances, correction_probabilities, contact_variance_time, contact_probability_time, info_.numThreeDofContacts)[0];

  for(int i = 0; i < info_.numThreeDofContacts; i++){
    if(contact[i]){
      contact[i] = contact_probability_overall(i) > contact_loss_likelihood_cutoff;
    }
    else{
      contact[i] = contact_probability_overall(i) > contact_likelihood_cutoff;
    }
  }

  int mode_detected = 0;
  for(int i = 0; i < info_.numThreeDofContacts; i++){
    if(contact[i]){
      mode_detected += (int) pow(2, i);
    }
  }
  // std::cerr<< "mode: " << mode_detected << std::endl;

  //   if(modeNumber2StanceLeg(modeSchedule_.modeAtTime(time))[i] && !contact[i]){
  //     contact_time_diff[i] = true;
  //   }

  //   if(contact[i] && contact_time_diff[i]){
  //     contact_time_diff[i] = false;

  //     const auto ind = lookup::findIndexInTimeArray(modeSchedule_.eventTimes, time);
  //     double time_diff = time - modeSchedule_.eventTimes[ind];
  //     for(int j = 0; j < modeSchedule_.eventTimes.size(); j++){
  //       modeSchedule_.eventTimes[j] += time_diff;
  //     }
  //   }
  // }

  // Fill ros msgs
  leg_contact.data.clear();
  leg_contact_prob.data.clear();
  leg_contact_prob_force.data.clear();

  leg_contact_prob_height.data.clear();
  leg_contact_prob_time.data.clear();
  leg_contact_prob_velocity.data.clear();
  leg_contact_prob_force_sensors.data.clear();
  leg_force.data.clear();
  leg_force_sensor.data.clear();
  leg_velocity.data.clear();
  leg_height.data.clear();
  leg_foothold.data.clear();
  leg_variance.data.clear();
  for(int i = 0; i < info_.numThreeDofContacts; i++){
    leg_contact.data.push_back(contact[i]);
    leg_contact_prob.data.push_back(contact_probability_overall(i));
    leg_contact_prob_force.data.push_back(contact_probability_force(i));

    leg_contact_prob_height.data.push_back(contact_probability_height(i));
    leg_contact_prob_time.data.push_back(contact_probability_time(i));
    leg_contact_prob_velocity.data.push_back(contact_probability_velocity(i));
    leg_contact_prob_force_sensors.data.push_back(contact_probability_force_sensor(i));
    leg_force.data.push_back(force_estimated(2 + 3*i) - input(2 + 3*i));
    leg_force_sensor.data.push_back(sensorForces(i));
    leg_velocity.data.push_back((footVelBodyDesired[i] - footVelBodyMeasured[i]).squaredNorm());

    leg_velocity.data.push_back(footVelBodyDesired[i].squaredNorm());
    leg_velocity.data.push_back(footVelBodyMeasured[i].squaredNorm());

    leg_height.data.push_back(footPos[i][2]);
    leg_foothold.data.push_back(mean_zg[i] - foot_offset);
    leg_variance.data.push_back(map_variance_weighting * kalman_map_variance[i]);
  }

  // Publish ros msgs
  leg_contact_pub.publish(leg_contact);
  leg_contact_prob_pub.publish(leg_contact_prob);
  leg_contact_prob_time_pub.publish(leg_contact_prob_time);
  leg_contact_prob_height_pub.publish(leg_contact_prob_height);
  leg_contact_prob_force_pub.publish(leg_contact_prob_force);
  leg_contact_prob_velocity_pub.publish(leg_contact_prob_velocity);
  leg_contact_prob_force_sensors_pub.publish(leg_contact_prob_force_sensors);
  leg_force_pub.publish(leg_force);
  leg_velocity_pub.publish(leg_velocity);
  leg_force_sensor_pub.publish(leg_force_sensor);
  leg_height_pub.publish(leg_height);
  leg_variance_pub.publish(leg_variance);
  leg_foothold_pub.publish(leg_foothold);

  return mode_detected;
}

double ContactEstimate::calculateContactProbabilityTime(double phase_switch, double phase_timer){
  return 0.5 * (phase_switch * (erf((phase_timer - mean_c0)/(variance_c0*sqrt(2))) + erf((mean_c1 - phase_timer)/(variance_c1*sqrt(2)))) + 
               (1 - phase_switch) * 0.5*(2 + erf((mean_not_c0 - phase_timer)/(variance_not_c0*sqrt(2))) + erf((phase_timer - mean_not_c1)/(variance_not_c1*sqrt(2)))));
}

double ContactEstimate::calculateContactProbabilityFootHeight(double foot_height, int leg){
  return 0.5 * (1 + erf((mean_zg[leg] - foot_height)/(variance_zg[leg]*sqrt(2))));
}

double ContactEstimate::calculateContactProbabilityFootForce(double phase_switch, double foot_force, double force_desired){
  double diff = foot_force - force_desired;
  return (phase_switch*(1 - erf(abs(diff - mean_force)/(variance_force*sqrt(2)))) +
                (1-phase_switch)*(1 - erf(abs(diff - mean_force_nc)/(variance_force_nc*sqrt(2)))));
}

double ContactEstimate::calculateContactProbabilityForceSensor(double foot_force){
  return 0.5 * (1 + erf((foot_force - mean_force_sensor)/(variance_force_sensor*sqrt(2))));
}

double ContactEstimate::calculateContactProbabilityFootVelocity(double phase_switch, vector3_t foot_vel){
  double velNorm = foot_vel.squaredNorm();
  return 0.5 * (phase_switch* (1 - erf((velNorm - mean_vel_c)/(variance_vel*sqrt(2)))) + 
                (1-phase_switch)*(1+ erf((velNorm - mean_vel_nc)/(variance_vel*sqrt(2)))));
}

// void ContactEstimate::getForceReadings(const sensor_msgs::JointState msg){
//   // Note again replace 4 with numlegs and 12 with numdof
//   for(int i = 0; i < info_.numThreeDofContacts; i++){
//     force_sensor_readings[i] = msg.effort[12 + i];
//   }
//   force_sensor_read = true;
// }

void ContactEstimate::getMap(const grid_map_msgs::GridMap& msg){
  grid_map::GridMapRosConverter::fromMessage(msg, map);
}

std::vector<Eigen::MatrixXd> ContactEstimate::KalmanCorrection(int nReadings, Eigen::MatrixXd correction_variances, Eigen::MatrixXd correction_probabilities, Eigen::MatrixXd prediction_variance, Eigen::MatrixXd prediction_probability, int numThreeDofContacts){
  Eigen::MatrixXd z = Eigen::MatrixXd(nReadings * numThreeDofContacts, 1);
  Eigen::MatrixXd K = Eigen::MatrixXd(nReadings * numThreeDofContacts, nReadings * numThreeDofContacts);
  Eigen::MatrixXd H = Eigen::MatrixXd(nReadings * numThreeDofContacts, numThreeDofContacts);


  for(int i = 0; i < nReadings; i++){
    H.block(i*numThreeDofContacts, 0, numThreeDofContacts, numThreeDofContacts) = Eigen::MatrixXd::Identity(numThreeDofContacts, numThreeDofContacts);
  }
  Eigen::MatrixXd sigma_vk = correction_variances.asDiagonal();
  z = correction_probabilities;

  K = ((H*prediction_variance*H.transpose() + sigma_vk).transpose().partialPivLu().solve((prediction_variance * H.transpose()).transpose())).transpose();
  
  Eigen::MatrixXd contact_probability_overall = prediction_probability + K*(z - H*prediction_probability);
  Eigen::MatrixXd contact_variance_overall = (Eigen::MatrixXd::Identity(nReadings*numThreeDofContacts, nReadings*numThreeDofContacts) - K*H) * prediction_variance;
  std::vector<Eigen::MatrixXd> result(2);
  result[0] = contact_probability_overall;
  result[1] = contact_variance_overall;
  return result;
}

std::vector<Eigen::MatrixXd> ContactEstimate::sampleHeights(std::vector<vector3_t> position, int radius){
  int nCells = (2*radius + 1)*(2*radius + 1);
  Eigen::MatrixXd correction_variances = Eigen::MatrixXd(nCells*info_.numThreeDofContacts, 1);
  Eigen::MatrixXd correction_heights = Eigen::MatrixXd(nCells*info_.numThreeDofContacts, 1);
  Eigen::MatrixXd prediction_variance = Eigen::MatrixXd(info_.numThreeDofContacts, info_.numThreeDofContacts); 
  Eigen::MatrixXd prediction_height = Eigen::MatrixXd(info_.numThreeDofContacts,1);

  for(int i = 0; i < info_.numThreeDofContacts; i++){
    Eigen::Array2i radMat;
    radMat << radius, radius;

    Eigen::Array2i centerIdx;
    map.getIndex(position[i].head<2>(), centerIdx);
    Eigen::Array2i startIdx = centerIdx - radMat;

    double planeHeight = map.at("elevation", centerIdx);
    double planeVariance = pow((planeHeight - map.at("lower_bound", centerIdx))/2.33, 2);
    if(std::isnan(planeHeight)){
      planeHeight = 0;
    }
    if(std::isnan(planeVariance)){
      planeVariance = 0.001;
    }

    planeHeight += foot_offset;

    for(int y = 0; y < 2*radius + 1; y++){
      for(int x = 0; x < 2*radius + 1; x++){
        Eigen::Array2i displacement;
        displacement << x, y;

        Eigen::Array2i idx = startIdx + displacement;

        double height = map.at("elevation", idx);
        double variance = pow((height - map.at("lower_bound", idx))/2.33, 2);
        if(std::isnan(height)){
          height = 0;
        }
        if(std::isnan(variance)){
          variance = 0.001;
        }
  
        height += foot_offset;
        correction_heights(i*nCells + y*(2*radius + 1) + x) = height;
  
        double zDistance = map.getResolution()*sqrt(pow(centerIdx(0) - idx(0), 2) + pow(centerIdx(1) - idx(1), 2)) / sqrt(joint_variance);
        double distanceWeight = 0.5 * (1 + erf(zDistance/sqrt(2)));
        correction_variances(i*nCells + y*(2*radius + 1) + x) = distanceWeight*variance;
      }
    }  

    prediction_variance(i, i) = planeVariance;
    prediction_height(i) = planeHeight;
  }

  return KalmanCorrection(nCells, correction_variances, correction_heights, prediction_variance, prediction_height, info_.numThreeDofContacts);
}

std::vector<Eigen::MatrixXd> ContactEstimate::weightVariances(std::vector<vector3_t> position, int radius){
  int nCells = (2*radius + 1)*(2*radius + 1) - 1;
  Eigen::MatrixXd correction_weights = Eigen::MatrixXd(nCells*info_.numThreeDofContacts, 1);
  Eigen::MatrixXd correction_variances = Eigen::MatrixXd(nCells*info_.numThreeDofContacts, 1);
  Eigen::MatrixXd prediction_variance = Eigen::MatrixXd(info_.numThreeDofContacts, info_.numThreeDofContacts); 
  Eigen::MatrixXd prediction_weight = Eigen::MatrixXd(info_.numThreeDofContacts,1);

  for(int i = 0; i < info_.numThreeDofContacts; i++){
    Eigen::Array2i radMat;
    radMat << radius, radius;

    Eigen::Array2i centerIdx;
    map.getIndex(position[i].head<2>(), centerIdx);
    Eigen::Array2i startIdx = centerIdx - radMat;

    double planeHeight = map.at("elevation", centerIdx);
    double planeVariance = pow((planeHeight - map.at("lower_bound", centerIdx))/2.33, 2);
    if(std::isnan(planeHeight)){
      planeHeight = 0;
    }
    if(std::isnan(planeVariance)){
      planeVariance = 0.0001;
    }
    planeHeight += foot_offset;

    map_debug.data.clear();
    map_debug2.data.clear();
    int j = 0;
    for(int y = 0; y < 2*radius + 1; y++){
      for(int x = 0; x < 2*radius + 1; x++){
        Eigen::Array2i displacement;
        displacement << x, y;

        Eigen::Array2i idx = startIdx + displacement;
        if(idx[0] != centerIdx[0] || idx[1] != centerIdx[1]){
          double height = map.at("elevation", idx);
          double variance = pow((height - map.at("lower_bound", idx))/2.33, 2);

          if(std::isnan(height)){
            height = 0;
          }
          if(std::isnan(variance)){
            variance = 0.0001;
          }
          height += foot_offset;

          double zDistance = map.getResolution()*sqrt(pow(centerIdx(0) - idx(0), 2) + pow(centerIdx(1) - idx(1), 2)) / sqrt(joint_variance);
          double zHeight = (height - planeHeight) / sqrt(joint_variance);
          double distanceWeight = 0.5 * (1 + erf(zDistance/sqrt(2)));

          correction_weights(i*nCells + j) = (1 + abs(zHeight))*variance;

          if(i == 0){
            map_debug.data.push_back((1 + abs(zHeight))*variance);
            map_debug2.data.push_back(zDistance);
          }
          correction_variances(i*nCells + j) = zDistance;
          j++;
        }
      }
    }  

    map_debug_pub.publish(map_debug);
    map_debug2_pub.publish(map_debug2);

    prediction_variance(i, i) = 1;
    prediction_weight(i) = planeVariance;
  }

  return KalmanCorrection(nCells, correction_variances, correction_weights, prediction_variance, prediction_weight, info_.numThreeDofContacts);
}

double ContactEstimate::getKalmanProbability(double detection_mean, double measurement_mean, double measurement_variance){
  return 1 + erf((-abs(measurement_mean - detection_mean))/(measurement_variance*sqrt(2))); 
}
}  // namespace legged
