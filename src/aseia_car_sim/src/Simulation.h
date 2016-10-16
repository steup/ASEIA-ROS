#pragma once

#include "Car.h"

#include <vector>

class Simulation {
  private:
    using CarVec = std::vector<Car>;
    Cars mCars;
  public:
    Simulation();
    void run();
};
