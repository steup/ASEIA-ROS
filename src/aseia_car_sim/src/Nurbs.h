#pragma once

#include <Eigen/Core>

namespace aseia_car_sim {

class NURBCurve {
  private:
    const Eigen::Matrix<float, Eigen::Dynamic, 1> mKnots;
    const Eigen::Matrix<float, Eigen::Dynamic, 3> mPoints;
    const size_t mDim;
  public:
    using Point = Eigen::Matrix<float, 3, 1>;

    NURBCurve(size_t dim, const decltype(mKnots)& knots, const decltype(mPoints)& points)
      : mKnots(knots), mPoints(points), mDim(dim) {}
    NURBCurve(size_t dim, decltype(mKnots)&& knots, decltype(mPoints)&& points);

    Point sample(float u) const;
};
}
