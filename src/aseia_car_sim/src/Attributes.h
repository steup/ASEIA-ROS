#pragma once

#include <ID.h>

namespace aseia_car_sim {

  struct Nurbs : public id::attribute::Base{
    static constexpr const id::attribute::ID value(){ return 100; }
  };

}
