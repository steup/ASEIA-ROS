#pragma once

#include <stdexcept>

namespace car {

class DataException : public std::runtime_error {
  public:
  enum  class Reason {
    vrepGone,
    badResult,
    serviceGone
  };
  const Reason reason;
  public:
    DataException(Reason reason);
};

}
