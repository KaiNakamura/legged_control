//
// Created by qiayuan on 22-12-23.
//
#pragma once

#include "legged_wbc/TrunkControllerBase.h"

namespace legged {

class HierarchicalTrunkController : public TrunkControllerBase {
 public:
  using TrunkControllerBase::TrunkControllerBase;

  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode, vector_t typeFlag) override;
};

}  // namespace legged
