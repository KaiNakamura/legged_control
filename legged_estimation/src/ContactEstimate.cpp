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

  qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  
  // Make all disturbance vectors of size specified by model
  tau_filtered_prev = Eigen::MatrixXd::Zero(model.nv, 1);
  tau_filtered_curr = Eigen::MatrixXd::Zero(model.nv, 1);
  tau_unfiltered_prev = Eigen::MatrixXd::Zero(model.nv, 1);
  tau_unfiltered_curr = Eigen::MatrixXd::Zero(model.nv, 1);
}

vector_t ContactEstimate::update(const ros::Time& time, const ros::Duration& period, const vector_t& rbdStateMeasured, vector_t input) {
  scalar_t dt = period.toSec();

  // Compute joint positions
  qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);
  qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
  qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);

  // Compute joint angular velocities
  vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
  vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
  vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

  // Get pinocchio current model and data
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  // Update algorithm, may be inefficient as this is also run in WbcBase
  pinocchio::crba(model, data, qMeasured_);

  // set up and get M(q_dot, q), C(q), g
  Eigen::MatrixXd M(info_.generalizedCoordinatesNum, info_.generalizedCoordinatesNum);
  Eigen::MatrixXd C(info_.generalizedCoordinatesNum, info_.generalizedCoordinatesNum);
  Eigen::MatrixXd g(info_.generalizedCoordinatesNum, 1);
  Eigen::MatrixXd p(info_.generalizedCoordinatesNum, 1);

  double frequencyCutoff = 100; // lambda in the paper
  double zDomainCutoff = 0.9048; // gamma in paper, given by e^(-lambda*dt), currently using average of 0.001
  double beta = (1 - zDomainCutoff)/zDomainCutoff/dt;

  M = data.M;
  C = pinocchio::computeCoriolisMatrix(model, data, qMeasured_, vMeasured_);
  g = pinocchio::computeGeneralizedGravity(model, data, qMeasured_);

  p = M * vMeasured_;

  // Update current tau
  tau_unfiltered_curr = input.head(9);

  // Update in laplace space

  // Update in zeta domain
  tau_filtered_curr = beta*p*tau_unfiltered_curr - beta*p*zDomainCutoff*tau_unfiltered_prev 
                      - (1 - zDomainCutoff) * (beta*p + tau_unfiltered_curr + C.transpose()*vMeasured_ - g)*tau_unfiltered_curr 
                      + zDomainCutoff*tau_filtered_prev; 

  // update previous taus
  tau_unfiltered_prev = tau_unfiltered_curr;
  tau_filtered_prev = tau_filtered_prev;

  return tau_filtered_curr;
}

}  // namespace legged
