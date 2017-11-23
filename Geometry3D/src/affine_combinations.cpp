#include "affine_combinations.hpp"
#include "Vector3.hpp"
#include <cassert>
#include <iostream>
#include <numeric>

namespace Impact {
namespace Geometry3D {

const double EPSILON = 1.0e-06;

Point getAffineCombination(const std::vector<Point>&    points,
                           const std::vector<imp_float>& weights)
{
    imp_uint n_points = static_cast<imp_uint>(points.size());
    assert(static_cast<imp_uint>(weights.size()) == n_points);
    assert(abs(std::accumulate<std::vector<imp_float>::const_iterator, imp_float>(weights.begin(), weights.end(), 0) - 1) < EPSILON);
    assert(n_points > 1);

    Point point = points[0];
    for (imp_uint i = 1; i < n_points; i++)
    {
        point += (points[i] - points[0])*weights[i];
    }

    return point;
}

Point getAffineCombination(const Point& A,
                           const Point& B,
                           imp_float weight_A, imp_float weight_B)
{
    assert(abs(weight_A + weight_B - 1) < EPSILON);
    return A + (B - A)*weight_B;
}

Point getAffineCombination(const Point& A,
                           const Point& B,
                           const Point& C,
                           imp_float weight_A, imp_float weight_B, imp_float weight_C)
{
    assert(abs(weight_A + weight_B + weight_C - 1) < EPSILON);
    return A + (B - A)*weight_B + (C - A)*weight_C;
}

} // Geometry3D
} // Impact
