#pragma once
#include <vector>
#include <assert.h>
#include "Point.hpp"

namespace Geometry3D {

template <typename F>
Point<F> getAffineCombination(const std::vector< Point<F> >& points,
                              const std::vector<F>&          weights);

template <typename F>
Point<F> getAffineCombination(const Point<F>& A,
                              const Point<F>& B,
                              F weight_A, F weight_B);

template <typename F>
Point<F> getAffineCombination(const Point<F>& A,
                              const Point<F>& B,
                              const Point<F>& C,
                              F weight_A, F weight_B, F weight_C);

const double EPSILON = 1.0e-06;

template <typename F>
Point<F> getAffineCombination(const std::vector< Point<F> >& points,
                              const std::vector<F>&          weights)
{
    size_t n_points = points.size();
    assert(weights.size() == n_points);
    assert(abs(std::accumulate(weights.begin(), weights.end(), 0) - 1) < EPSILON);
    assert(n_points > 1);

    Point<F> point = points[0];
    for (size_t i = 1; i < n_points; i++)
    {
        point += (points[i] - points[0])*weights[i];
    }

    return point;
}

template <typename F>
Point<F> getAffineCombination(const Point<F>& A,
                              const Point<F>& B,
                              F weight_A, F weight_B)
{
    assert(abs(weight_A + weight_B - 1) < EPSILON);
    return A + (B - A)*weight_B;
}

template <typename F>
Point<F> getAffineCombination(const Point<F>& A,
                              const Point<F>& B,
                              const Point<F>& C,
                              F weight_A, F weight_B, F weight_C)
{
    std::cout << abs(weight_A + weight_B + weight_C - 1) << std::endl;
    assert(abs(weight_A + weight_B + weight_C - 1) < EPSILON);
    return A + (B - A)*weight_B + (C - A)*weight_C;
}

} // Geometry3D
