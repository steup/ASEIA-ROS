#pragma once

#include <Eigen/Core>
#include <stdexcept>
#include <utility>

namespace aseia_car_sim {

template<typename KnotsType, typename PointsType, typename PointType>
class NURBCurve {
  private:
    const KnotsType mKnots;
    const PointsType mPoints;
    size_t mDim;
  public:
    using Point = PointType;
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

      Point point = Point::Zeros();
      for(ssize_t j=0; j<mPoints.rows(); j++)
        point+=N(j,mDim,u)*Point(mPoints.row(j));
      return point;
    }

    operator bool() const {
      return mKnots.rows()>0 && mPoints.rows()>0;
    }
};
}
