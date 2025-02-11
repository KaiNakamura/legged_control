//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <realtime_tools/realtime_buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class ContactEstimate{
 public:
  ContactEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  size_t update(scalar_t time, const ros::Duration& period, vector_t input, const vector_t& rbdStateMeasured, vector_t torque, contact_flag_t contact_flag, ModeSchedule modeSchedule_);

  void loadSettings(const std::string& taskFile, bool verbose);

 protected:
  void updateFromTopic();
  double calculateContactProbabilityTime(double phase_switch, double phase_timer);
  double calculateContactProbabilityFootHeight(double foot_height, int leg);
  double calculateContactProbabilityFootForce(double foot_force);
  double calculateContactProbabilityForceSensor(double foot_force);
  void getForceReadings(const sensor_msgs::JointState msg);
  Eigen::MatrixXd KalmanCorrection(int nReadings, Eigen::MatrixXd correction_variances, Eigen::MatrixXd correction_probabilities, Eigen::MatrixXd prediction_variance, Eigen::MatrixXd prediction_probability, int numThreeDofContacts);

  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
  vector_t rbdState_;
  vector_t qMeasured_, vMeasured_, inputLast_;

  Eigen::Matrix<double, Eigen::Dynamic, 1> tau_filtered_prev;
  Eigen::Matrix<double, Eigen::Dynamic, 1> tau_filtered_curr;

  Eigen::Matrix<double, Eigen::Dynamic, 1> tau_unfiltered_prev;
  Eigen::Matrix<double, Eigen::Dynamic, 1> tau_unfiltered_curr;

  Eigen::Matrix<double, Eigen::Dynamic, 1> tau_measured_curr;
  Eigen::Matrix<double, Eigen::Dynamic, 1> tau_measured_prev;

  Eigen::Matrix<double, Eigen::Dynamic, 1> p_prev;

  // Initialize parameters as defined in contact estimation paper
  double mean_not_c0 = 0;
  double mean_not_c1 = 1;
  double mean_c0 = 0;
  double mean_c1 = 1;

  double variance_not_c0 = 0.05;
  double variance_not_c1 = 0.05;
  double variance_c0 = 0.05;
  double variance_c1 = 0.05;

  // Note: these can be updated based on other sensors (vision) and historical footsteps
  // Note: Replace 4 with num legs from somewhere
  double mean_zg[4] = {0.02, 0.02, 0.02, 0.02};
  double variance_zg = 0.075;

  double force_sensor_readings[4] = {-1, -1, -1, -1};
  bool force_sensor_read = false;

  double mean_force = 30;
  double variance_force = 15;

  double contact_likelihood_cutoff = 0.6;
  double contact_loss_likelihood_cutoff = 0.4;
  bool contact[4] = {false, false, false, false};

  double mean_force_sensor = 80;
  double variance_force_sensor = 40;

  ros::Publisher leg1_contact_pub;
  ros::Publisher leg2_contact_pub;
  ros::Publisher leg3_contact_pub;
  ros::Publisher leg4_contact_pub;

  ros::Publisher leg1_contact_prob_pub;
  ros::Publisher leg2_contact_prob_pub;
  ros::Publisher leg3_contact_prob_pub;
  ros::Publisher leg4_contact_prob_pub;

  ros::Publisher leg1_force_pub;
  ros::Publisher leg2_force_pub;
  ros::Publisher leg3_force_pub;
  ros::Publisher leg4_force_pub;

  ros::Publisher leg1_contact_prob_time_pub;
  ros::Publisher leg1_contact_prob_force_pub;
  ros::Publisher leg1_contact_prob_height_pub;
  ros::Publisher leg1_contact_prob_force_sensors_pub;

  ros::Publisher leg1_height_pub;

  std_msgs::Int16 leg1_contact;
  std_msgs::Int16 leg2_contact;
  std_msgs::Int16 leg3_contact;
  std_msgs::Int16 leg4_contact;

  std_msgs::Float64 leg1_contact_prob;
  std_msgs::Float64 leg2_contact_prob;
  std_msgs::Float64 leg3_contact_prob;
  std_msgs::Float64 leg4_contact_prob;

  std_msgs::Float64 leg1_contact_prob_time;
  std_msgs::Float64 leg1_contact_prob_force;
  std_msgs::Float64 leg1_contact_prob_height;
  std_msgs::Float64 leg1_contact_prob_force_sensors;

  std_msgs::Float64 leg1_height;

  std_msgs::Float64 leg1_force;
  std_msgs::Float64 leg2_force;
  std_msgs::Float64 leg3_force;
  std_msgs::Float64 leg4_force;

  ros::Subscriber joint_state_sub;

 private:
  // Topic
  bool topicUpdated_;
  tf2::Transform world2odom_;
};

}  // namespace legged
