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

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class ContactEstimate{
 public:
  ContactEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  vector_t update(const ros::Time& time, const ros::Duration& period, const vector_t& rbdStateMeasured, vector_t input);

  void loadSettings(const std::string& taskFile, bool verbose);

 protected:
  void updateFromTopic();

  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
  
  vector_t rbdState_;
  vector_t qMeasured_, vMeasured_;
  
  Eigen::Matrix<double, Eigen::Dynamic, 1> tau_filtered_prev;
  Eigen::Matrix<double, Eigen::Dynamic, 1> tau_filtered_curr;

  Eigen::Matrix<double, Eigen::Dynamic, 1> tau_unfiltered_prev;
  Eigen::Matrix<double, Eigen::Dynamic, 1> tau_unfiltered_curr;

 private:
  // Topic
  bool topicUpdated_;
  tf2::Transform world2odom_;
};

}  // namespace legged
