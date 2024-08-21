#pragma once

#include <functional>
#include <vector>

#include "Auton.h"

namespace evAPI {
class AutonManager {
  private:
    std::vector<Auton> autons;

  public:
    void loadAutons(const char* path);
    // void populateUI(evAPI::autoSelectorUI& ui);
};

}  // namespace evAPI
