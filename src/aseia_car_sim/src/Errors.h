#include <aseia_car_sim/ErrorConfig.h>

#include <dynamic_reconfigure/server.h>

#include <ID.h>
#include <IDIO.h>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>

#include <cmath>

class ErrorGenerator {
  private:
    boost::random::mt11213b mEngine;
    float mTimeSigma, mDistSigma, mPosSigma, mOriSigma, mConfidence;
    dynamic_reconfigure::Server<aseia_car_sim::ErrorConfig> mDynReConfServer;
    void dynReConfCallback(aseia_car_sim::ErrorConfig& config, uint32_t level);
    ErrorGenerator();
  public:
    static ErrorGenerator& instance();
    boost::random::mt11213b& engine() { return mEngine; }
    template<typename T>
    float sigma(T) const { return 0.0f; }
    float sigma(id::attribute::Time) const { return mTimeSigma; }
    float sigma(id::attribute::Distance) const { return mDistSigma; }
    float sigma(id::attribute::Position) const { return mPosSigma; }
    float sigma(id::attribute::Orientation) const { return mOriSigma; }
    float confidence() const { return mConfidence; }
};

template<typename Attribute>
class NormalError {
  private:
    ErrorGenerator& mGen;
    float erfInv(float x){
      float tt1, tt2, lnx, sgn;
      sgn = (x < 0) ? -1.0f : 1.0f;

      x = (1 - x)*(1 + x);        // x = 1 - x*x;
      lnx = logf(x);

      tt1 = 2/(M_PI*0.147) + 0.5f * lnx;
      tt2 = 1/(0.147) * lnx;

      return(sgn*sqrtf(-tt1 + sqrtf(tt1*tt1 - tt2)));
    }
  public:
    using T = typename Attribute::ValueType::BaseType::VType;
    using ID        = typename Attribute::IDType;
    NormalError() : mGen(ErrorGenerator::instance()) {}
    Attribute operator()() {
      Attribute a;
      auto& v = a.value();
      float sigma = mGen.sigma(ID());
      if(sigma!=0) {
        T interval = sqrt(2.0)*sigma*erfInv(2*mGen.confidence()-1);
        boost::random::normal_distribution<T> dist(0, sigma);
        for(size_t i=0;i<v.rows();i++)
          for(size_t j=0;j<v.cols();j++) {
            v(i,j).value(dist(mGen.engine()));
            v(i,j).uncertainty(interval);
          }
      } else
        v=v.zero();
      ROS_DEBUG_STREAM("Inducing error on attribute " << id::attribute::name(ID()) << ": " << a);
      return a;
    }
};
