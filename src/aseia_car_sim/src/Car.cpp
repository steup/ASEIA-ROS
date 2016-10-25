#include "Car.h"

namespace car {

  using namespace std;

  class RoadScan : public Data {

  };

  class LaneControl : public Controller {
    private:
      Data mData;
    public:
      LaneControl() {}

      void reference(const Data& ref) {

      }

      const Data& reference() const {
        return mData;
      }

      void operator()(DataMap& data) {

      }
  };

  void Car::feed(const Data& data) {

  }

  void Car::update() {

  }

  Data Car::speed() const {
    return Data();
  }

  Data Car::angle() const {
    return Data();
  }

}
