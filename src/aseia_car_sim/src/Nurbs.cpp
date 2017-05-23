#include "Nurbs.h"

#include <stdexcept>
#include <utility>

namespace aseia_car_sim {
  using namespace std;

  NURBCurve::NURBCurve(size_t dim, decltype(mKnots)&& knots, decltype(mPoints)&& points)
    : mKnots(move(knots)), mPoints(move(points)), mDim(dim)
  {}

  NURBCurve::Point NURBCurve::sample(float u) const {

    auto f=[this](size_t i, size_t n, float u) {
        return (u-mKnots[i])/(mKnots[i+n]-mKnots[i]);
    };
    auto g=[this](size_t i, size_t n, float u) {
        return (mKnots[i+n]-u)/(mKnots[i+n]-mKnots[i]);
    };
    std::function<float(size_t,size_t,float)> N =
      [this, &f, &g, &N](size_t i, size_t n, float u) -> float {
        if(n==0)
            return (u >= mKnots[i] && u < mKnots[i+1])?1.0f:0.0f;
        else
            return f(i,n, u)*N(i, n-1, u) + g(i+1,n, u)*N(i+1, n-1, u);
    };

    if(u<0 || u>1)
      throw out_of_range("Curve index out of range [0,1]");

    Point point = Point::Zero();
    for(ssize_t j=0; j<mPoints.rows(); j++)
      point+=N(j,3,u)*mPoints.row(j);
    return point;
  }
}
