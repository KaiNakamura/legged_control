//
// Created by qiayuan on 2021/11/5.
//
#pragma once
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace legged {
class ContactSensorHandle {
 public:
  ContactSensorHandle() = default;

  ContactSensorHandle(const std::string& name, bool* isContact, double* force) : name_(name), isContact_(isContact), force_(force) {
    if (isContact == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. isContact pointer is null.");
    }
    if (force == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. force pointer is null.");
    }
  }

  std::string getName() const { return name_; }

  bool isContact() const {
    assert(isContact_);
    return *isContact_;
  }

  void setContact(bool cmd) const {
    assert(isContact_);
    *isContact_ = cmd;
  }

  void setForce(double cmd) const {
    assert(force_);
    *force_ = cmd;
  }

  double getForce() const{
    assert(force_);
    return *force_;
  }

 private:
  std::string name_;

  bool* isContact_ = {nullptr};
  double* force_ = {nullptr};
};

class ContactSensorInterface
    : public hardware_interface::HardwareResourceManager<ContactSensorHandle, hardware_interface::DontClaimResources> {};

}  // namespace legged
