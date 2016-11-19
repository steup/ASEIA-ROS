#include "DataException.h"

namespace {
  const char*  msg(car::DataException::Reason r) {
    switch(r) {
      case(car::DataException::Reason::vrepGone)   : return "V-Rep is gone";
      case(car::DataException::Reason::badResult)  : return "Service call failed with bad result";
      case(car::DataException::Reason::serviceGone): return "Persistent service is gone";
      default                                      : return "Unknown error";
    }
  }
}

namespace car {

DataException::DataException(Reason reason)
  : std::runtime_error(msg(reason)),
    reason(reason)
{}

}
