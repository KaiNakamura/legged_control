//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/TrunkControllerBase.h"

namespace legged {

class WeightedTrunkController : public TrunkControllerBase {
 public:
  using TrunkControllerBase::TrunkControllerBase;

  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode) override;

  void loadTasksSetting(const std::string& taskFile, bool verbose) override;

 protected:
  virtual Task formulateConstraints();
  virtual Task formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired);

 private:
  scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;
};

}  // namespace legged