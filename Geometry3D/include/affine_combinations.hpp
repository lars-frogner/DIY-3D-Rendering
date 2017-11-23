#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include <vector>

namespace Impact {
namespace Geometry3D {

Point getAffineCombination(const std::vector<Point>&    points,
                           const std::vector<imp_float>& weights);

Point getAffineCombination(const Point& A,
                           const Point& B,
                           imp_float weight_A, imp_float weight_B);

Point getAffineCombination(const Point& A,
                           const Point& B,
                           const Point& C,
                           imp_float weight_A, imp_float weight_B, imp_float weight_C);

} // Geometry3D
} // Impact
