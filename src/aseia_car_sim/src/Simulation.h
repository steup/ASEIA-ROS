#pragma once

#include "Car.h"

#include <vector>
#include <cstdint>

namespace car {
  class Simulation {
    private:
      using Cars = std::vector<Car>;
      Cars mCars;
    public:
      using CtrlNames = std::vector<std::string>;
      Simulation(std::size_t carNum, const CtrlNames& ctrlNames);
      ~Simulation();
      void run();
  };
}
