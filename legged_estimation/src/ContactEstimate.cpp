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

namespace legged {

ContactEstimate::ContactEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                            const PinocchioEndEffectorKinematics& eeKinematics): pinocchioInterface_(std::move(pinocchioInterface)),
                                            info_(std::move(info)),
                                            eeKinematics_(eeKinematics.clone()),
                                            rbdState_(vector_t ::Zero(2 * info_.generalizedCoordinatesNum)){
  eeKinematics_->setPinocchioInterface(pinocchioInterface_);

  world2odom_.setRotation(tf2::Quaternion::getIdentity());

  const auto& model = pinocchioInterface_.getModel();
  
  // Make all disturbance vectors of size specified by model
  tau_filtered_prev = Eigen::MatrixXd::Zero(model.nv, 1);
  tau_filtered_curr = Eigen::MatrixXd::Zero(model.nv, 1);
  tau_unfiltered_prev = Eigen::MatrixXd::Zero(model.nv, 1);
  tau_unfiltered_curr = Eigen::MatrixXd::Zero(model.nv, 1);
  tau_measured_prev = Eigen::MatrixXd::Zero(model.nv, 1);
  tau_measured_curr = Eigen::MatrixXd::Zero(model.nv, 1);

  p_prev = Eigen::MatrixXd::Zero(model.nv, 1);

  qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  vMeasured_ = vector_t(info_.generalizedCoordinatesNum);

  ros::NodeHandle nh;

  leg1_contact_pub = nh.advertise<std_msgs::Int16>("leg1_contact", 10);
  leg2_contact_pub = nh.advertise<std_msgs::Int16>("leg2_contact", 10);
  leg3_contact_pub = nh.advertise<std_msgs::Int16>("leg3_contact", 10);
  leg4_contact_pub = nh.advertise<std_msgs::Int16>("leg4_contact", 10);

  leg1_contact_prob_pub = nh.advertise<std_msgs::Float64>("leg1_contact_prob", 10);
  leg2_contact_prob_pub = nh.advertise<std_msgs::Float64>("leg2_contact_prob", 10);
  leg3_contact_prob_pub = nh.advertise<std_msgs::Float64>("leg3_contact_prob", 10);
  leg4_contact_prob_pub = nh.advertise<std_msgs::Float64>("leg4_contact_prob", 10);

  leg1_contact_prob_force_pub = nh.advertise<std_msgs::Float64>("leg1_contact_force_prob", 10);
  leg1_contact_prob_time_pub = nh.advertise<std_msgs::Float64>("leg1_contact_time_prob", 10);
  leg1_contact_prob_height_pub = nh.advertise<std_msgs::Float64>("leg1_contact_height_prob", 10);
}

size_t ContactEstimate::update(scalar_t time, const ros::Duration& period, vector_t input, const vector_t& rbdStateMeasured, vector_t torque, contact_flag_t contactFlag, ModeSchedule modeSchedule_) {
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

  double frequencyCutoff = 15; // lambda in the paper
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
  for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
    // Note: this uses the force sensors for contact identification rather than the mpc plan which is out of line with the rest of the program, consider if this is appropriate
    // if(contactFlag[i]){
    //   ST.block(6 + 3 * i, 3 * i, 3, 3).setIdentity();
    // }

    if(modeNumber2StanceLeg(modeSchedule_.modeAtTime(time))[i]){
      ST.block(6 + 3 * i, 3 * i, 3, 3).setIdentity();
    }
  }
  
  // Update current tau
  tau_unfiltered_curr = j_.transpose() * input.head(3*info_.numThreeDofContacts);

  // Update in laplace space

  // Update in zeta domain
  tau_measured_curr = beta*p + ST*torque + C.transpose()*vMeasured_ - g;
  tau_filtered_curr = -beta*p + zDomainCutoff*beta*p_prev -(1 - zDomainCutoff)*tau_measured_curr + zDomainCutoff*tau_measured_prev; //Note: I am not 100% confident on the signs here

  // Calculate the estimation of the applied force... also make it negative idk why it came out all the wrong sign
  Eigen::MatrixXd force_estimated = -(ST.transpose()*j_.transpose()).completeOrthogonalDecomposition().pseudoInverse()*ST.transpose()*tau_filtered_curr;

  // std::cout << "tau_filtered_curr: " << tau_filtered_curr.transpose() << std::endl;
  // std::cout << "tau_unfiltered_curr: " << tau_unfiltered_curr.transpose() << std::endl;
  // std::cout << "tau_measured_curr: " << tau_measured_curr.transpose() << std::endl;  
  // std::cout << "tau_filtered_prev: " << tau_filtered_prev.transpose() << std::endl;
  // std::cout << "tau_unfiltered_prev: " << tau_unfiltered_prev.transpose() << std::endl;  
  // std::cout << "tau_measured_prev: " << tau_measured_prev.transpose() << std::endl;

  // std::cout << "q: " << qMeasured_.transpose() << std::endl;
  // std::cout << "v: " << vMeasured_.transpose() << std::endl;

  // std::cout << "JT: " << std::endl << j_.transpose() << std::endl;
  // std::cout << "ST: " << std::endl << ST << std::endl;

  std::cout << "estimated force: " << std::endl << force_estimated.transpose() << std::endl;
  // std::cout << "applied force: " << std::endl << input.head(3*info_.numThreeDofContacts).transpose() << std::endl;
  
  // std::cout << "contactFlag: ";
  // for(size_t i = 0; i < info_.numThreeDofContacts; i++){
  //   std::cout << contactFlag[i] << " ";
  // }
  // std::cout << std::endl;

  // update previous taus
  tau_unfiltered_prev = tau_unfiltered_curr;
  tau_filtered_prev = tau_filtered_curr;
  tau_measured_prev = tau_measured_curr;
  p_prev = p;

  // Begin kalman filtering
  // Begin time-based kalman filter (probability phase)
  Eigen::MatrixXd contact_probability_time = Eigen::MatrixXd(info_.numThreeDofContacts, 1);
  
  for(int i = 0; i < info_.numThreeDofContacts; i++){
    // std::cout << modeSchedule_.percentageAtTime(time) << std::endl;
    contact_probability_time(i) = calculateContactProbabilityTime(modeNumber2StanceLeg(modeSchedule_.modeAtTime(time))[i], modeSchedule_.percentageAtTime(time));
  }

  // std::cout << "contact probability gait: " << std::endl << contact_probability_time.transpose() << std::endl;

  Eigen::MatrixXd contact_variance_time = Eigen::MatrixXd(info_.numThreeDofContacts, info_.numThreeDofContacts);
  for(int i = 0; i < info_.numThreeDofContacts; i++){

    if(modeNumber2StanceLeg(modeSchedule_.modeAtTime(time))[i]){
      contact_variance_time(i, i) = variance_c1;
    }
    else{
      contact_variance_time(i, i) = variance_c0;
    }
  }

  // Begin Kalman correction measurement model (update phase)
  std::vector<vector3_t> footPos = eeKinematics_->getPosition(vector_t()); // I hate this notation but it's native to pinocchio interface

  // std::cout << "foot position: " << std::endl;
  // for(int i = 0; i < info_.numThreeDofContacts; i++){
  //   std::cout << footPos[i].transpose() << " ";
  // }
  // std::cout << std::endl;

  Eigen::MatrixXd contact_probability_height = Eigen::MatrixXd(info_.numThreeDofContacts, 1);
  for(int i = 0; i < info_.numThreeDofContacts; i++){
    contact_probability_height(i) = calculateContactProbabilityFootHeight(footPos[i](2), i);
  }
  // std::cout << "contact probability foot height: " << std::endl << contact_probability_height.transpose() << std::endl;

  Eigen::MatrixXd contact_probability_force = Eigen::MatrixXd(info_.numThreeDofContacts, 1);
  for(int i = 0; i < info_.numThreeDofContacts; i++){
    contact_probability_force(i) = calculateContactProbabilityFootForce(force_estimated(2 + 3*i));
  }
  // std::cout << "contact probability force: " << std::endl << contact_probability_force.transpose() << std::endl;

  Eigen::MatrixXd z = Eigen::MatrixXd(2 * info_.numThreeDofContacts, 1);
  Eigen::MatrixXd K = Eigen::MatrixXd(2 * info_.numThreeDofContacts, 2 * info_.numThreeDofContacts);
  Eigen::MatrixXd H = Eigen::MatrixXd(2 * info_.numThreeDofContacts, info_.numThreeDofContacts);

  H.block(0, 0, info_.numThreeDofContacts, info_.numThreeDofContacts) = Eigen::MatrixXd::Identity(info_.numThreeDofContacts, info_.numThreeDofContacts);
  H.block(info_.numThreeDofContacts, 0, info_.numThreeDofContacts, info_.numThreeDofContacts) = Eigen::MatrixXd::Identity(info_.numThreeDofContacts, info_.numThreeDofContacts);

  z << contact_probability_height, contact_probability_force;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> sigma_vk = Eigen::MatrixXd(2 * info_.numThreeDofContacts, 2 * info_.numThreeDofContacts);
  sigma_vk.block(0, 0, info_.numThreeDofContacts, info_.numThreeDofContacts) = variance_zg * Eigen::MatrixXd::Identity(info_.numThreeDofContacts, info_.numThreeDofContacts);
  sigma_vk.block(info_.numThreeDofContacts, info_.numThreeDofContacts, info_.numThreeDofContacts, info_.numThreeDofContacts) = variance_force * Eigen::MatrixXd::Identity(info_.numThreeDofContacts, info_.numThreeDofContacts);
  sigma_vk.block(info_.numThreeDofContacts, 0, info_.numThreeDofContacts, info_.numThreeDofContacts) = Eigen::MatrixXd::Zero(info_.numThreeDofContacts, info_.numThreeDofContacts);
  sigma_vk.block(0, info_.numThreeDofContacts, info_.numThreeDofContacts, info_.numThreeDofContacts) = Eigen::MatrixXd::Zero(info_.numThreeDofContacts, info_.numThreeDofContacts);

  K = contact_variance_time * H.transpose() * (H*contact_variance_time*H.transpose() + sigma_vk).completeOrthogonalDecomposition().pseudoInverse();
  Eigen::MatrixXd contact_probability_overall = contact_probability_time + K*(z - H*contact_probability_time);
  Eigen::MatrixXd contact_variance_overall = (Eigen::MatrixXd::Identity(2*info_.numThreeDofContacts, 2*info_.numThreeDofContacts) - K*H) * contact_variance_time;

  std::cout << "Overall contact probability: " << std::endl << contact_probability_overall.transpose() << std::endl;
  // std::cout << "mean foot height 1: " << mean_zg[0] << std::endl;
  // std::cout << "Overall contact variance: " << std::endl << contact_variance_overall.transpose() << std::endl;

  int mode_detected = 0;
  for(int i = 0; i < info_.numThreeDofContacts; i++){
    if(contact_probability_overall(i) > contact_likelihood_cutoff){
      mode_detected += (int) pow(2, i);
      // mean_zg[i] = footPos[i](2) + 0.05;
    }
  }

  // Fill ros msgs
  leg1_contact.data = contact_probability_overall(0) > contact_likelihood_cutoff ? 1:0;
  leg2_contact.data = contact_probability_overall(1) > contact_likelihood_cutoff ? 1:0;
  leg3_contact.data = contact_probability_overall(2) > contact_likelihood_cutoff ? 1:0;
  leg4_contact.data = contact_probability_overall(3) > contact_likelihood_cutoff ? 1:0;

  leg1_contact_prob.data = contact_probability_overall(0);
  leg2_contact_prob.data = contact_probability_overall(1);
  leg3_contact_prob.data = contact_probability_overall(2);
  leg4_contact_prob.data = contact_probability_overall(3);

  leg1_contact_prob_force.data = contact_probability_force(0);
  leg1_contact_prob_height.data = contact_probability_height(0);
  leg1_contact_prob_time.data = contact_probability_time(0);


  // Publish ros msgs
  leg1_contact_pub.publish(leg1_contact);
  leg2_contact_pub.publish(leg2_contact);
  leg3_contact_pub.publish(leg3_contact);
  leg4_contact_pub.publish(leg4_contact);

  leg1_contact_prob_pub.publish(leg1_contact_prob);
  leg2_contact_prob_pub.publish(leg2_contact_prob);
  leg3_contact_prob_pub.publish(leg3_contact_prob);
  leg4_contact_prob_pub.publish(leg4_contact_prob);

  leg1_contact_prob_time_pub.publish(leg1_contact_prob_time);
  leg1_contact_prob_height_pub.publish(leg1_contact_prob_height);
  leg1_contact_prob_force_pub.publish(leg1_contact_prob_force);

  return mode_detected;
}

double ContactEstimate::calculateContactProbabilityTime(double phase_switch, double phase_timer){
  return 0.5 * (phase_switch * (erf((phase_timer - mean_c0)/(variance_c0*sqrt(2))) + erf((mean_c1 - phase_timer)/(variance_c1*sqrt(2)))) + 
               (1 - phase_switch) * 0.5*(2 + erf((mean_not_c0 - phase_timer)/(variance_not_c0*sqrt(2))) + erf((phase_timer - mean_not_c1)/(variance_not_c1*sqrt(2)))));
}

double ContactEstimate::calculateContactProbabilityFootHeight(double foot_height, int leg){
  return 0.5 * (1 + erf((mean_zg[leg] - foot_height)/(variance_zg*sqrt(2))));
}

double ContactEstimate::calculateContactProbabilityFootForce(double foot_force){
  return 0.5 * (1 + erf((foot_force - mean_force)/(variance_force*sqrt(2))));
}
}  // namespace legged
