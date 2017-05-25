#pragma once

#include <Value.h>
#include <MetaValue.h>

#include <Eigen/Core>
#include <stdexcept>
#include <utility>

namespace aseia_car_sim {

class NURBCurve {
  public:
    using KnotsType  = Value<float, Eigen::Dynamic, 1, false>;
    using PointsType = Value<float, Eigen::Dynamic, 3, false>;
  private:

    const KnotsType mKnots;
    const PointsType mPoints;
    size_t mDim;
  public:
    using Point = Value<float, 1, 3, false>;
    NURBCurve() : mKnots(), mPoints(), mDim(0) {}

    NURBCurve(size_t dim, const KnotsType& knots, const PointsType& points)
      : mKnots(knots), mPoints(points), mDim(dim) {}

    Point sample(float u) const {

      auto f=[this](size_t i, size_t n, float u) {
          return (u-mKnots(i,0))/(mKnots(i+n,0)-mKnots(i,0));
      };
      auto g=[this](size_t i, size_t n, float u) {
          return (mKnots(i+n,0)-u)/(mKnots(i+n,0)-mKnots(i,0));
      };
      std::function<float(size_t,size_t,float)> N =
        [this, &f, &g, &N](size_t i, size_t n, float u) -> float {
          if(n==0)
              return (u >= mKnots(i,0) && u < mKnots(i+1,0))?1.0f:0.0f;
          else
              return f(i,n, u)*N(i, n-1, u) + g(i+1,n, u)*N(i+1, n-1, u);
      };

      if(u<0 || u>1)
        throw std::out_of_range("Curve index out of range [0,1]");

      Point point = Point().zero();
      for(size_t j=0; j<(size_t)mPoints.rows(); j++)
        point+=N(j,mDim,u)*Point(mPoints.row(j));
      return point;
    }

    operator bool() const {
      return mKnots.rows()>0 && mPoints.rows()>0;
    }
};

class MetaNURBCurve {
  public:
    using KnotsType  = MetaValue;
    using PointsType = MetaValue;
  private:

    const KnotsType mKnots;
    const PointsType mPoints;
    size_t mDim;
  public:
    using Point = MetaValue;
    MetaNURBCurve() : mKnots(), mPoints(), mDim(0) {}

    MetaNURBCurve(size_t dim, const KnotsType& knots, const PointsType& points)
      : mKnots(knots), mPoints(points), mDim(dim) {}

    Point sample(const MetaValue& u) const {

      auto f=[this](size_t i, size_t n, const MetaValue& u) {
          return (u-mKnots(i,0))/(mKnots(i+n,0)-mKnots(i,0));
      };
      auto g=[this](size_t i, size_t n, const MetaValue& u) {
          return (mKnots(i+n,0)-u)/(mKnots(i+n,0)-mKnots(i,0));
      };
      std::function<MetaValue(size_t,size_t,const MetaValue&)> N =
        [this, &f, &g, &N](size_t i, size_t n, const MetaValue& u) -> MetaValue {
          if(n!=0)
              return f(i,n, u)*N(i, n-1, u) + g(i+1,n, u)*N(i+1, n-1, u);
          else
              return (u >= mKnots(i,0) && u < mKnots(i+1,0))?1.0:0.0;
      };

      if(u<0.0 || u>1.0f)
        throw std::out_of_range("Curve index out of range [0,1]");

      Point point = {{{0.0}, {0.0}, {0.0}}};
      for(size_t j=0; j<(size_t)mPoints.rows(); j++)
        point+=N(j,mDim,u)*Point(mPoints.row(j));
      return point;
    }

    operator bool() const {
      return mKnots.rows()>0 && mPoints.rows()>0;
    }
};
}
