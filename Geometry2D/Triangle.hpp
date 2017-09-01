#pragma once
#include <assert.h>
#include <string>
#include <sstream>
#include "Point.hpp"
#include "Vector.hpp"
#include "AxisAlignedRectangle.hpp"

namespace Geometry2D {

template <typename F>
class Triangle {

private:
    Point<F> _A, _B, _C;
    Vector<F> _AB, _AC;
    F _normalization;

public:
    Triangle<F>(const Point<F>& new_A,
                const Point<F>& new_B,
                const Point<F>& new_C);

    const Point<F>& getPointA() const;
    const Point<F>& getPointB() const;
    const Point<F>& getPointC() const;

    void getBarycentricCoordinates(const Point<F> X,
                                   F& alpha, F& beta, F& gamma) const;

    Triangle<F>& setPointA(const Point<F>& new_A);
    Triangle<F>& setPointB(const Point<F>& new_B);
    Triangle<F>& setPointC(const Point<F>& new_C);
    Triangle<F>& setPoints(const Point<F>& new_A,
                           const Point<F>& new_B,
                           const Point<F>& new_C);

    Triangle<F>& translate(F dx, F dy);
    Triangle<F>& translate(const Vector<F>& displacement);

    Triangle<F>& precomputeBarycentricQuantities();
    
    AxisAlignedRectangle<F> getAABB() const;
    AxisAlignedRectangle<F> getAABB(const Point<F>& lower_corner_limit,
                                    const Point<F>& upper_corner_limit) const;

    std::string toString() const;
};

template <typename F>
Triangle<F>::Triangle(const Point<F>& new_A,
                      const Point<F>& new_B,
                      const Point<F>& new_C)
    : _A(new_A), _B(new_B), _C(new_C) {}

template <typename F>
const Point<F>& Triangle<F>::getPointA() const
{
    return _A;
}

template <typename F>
const Point<F>& Triangle<F>::getPointB() const
{
    return _B;
}

template <typename F>
const Point<F>& Triangle<F>::getPointC() const
{
    return _C;
}

template <typename F>
void Triangle<F>::getBarycentricCoordinates(const Point<F> X,
                                            F& alpha, F& beta, F& gamma) const
{
    const Vector<F>& AX = X - _A;

    beta = (AX.x*_AC.y - _AC.x*AX.y)*_normalization;
    gamma = (_AB.x*AX.y - AX.x*_AB.y)*_normalization;
    alpha = 1 - beta - gamma;
}

template <typename F>
Triangle<F>& Triangle<F>::setPointA(const Point<F>& new_A)
{
    _A = new_A;
}

template <typename F>
Triangle<F>& Triangle<F>::setPointB(const Point<F>& new_B)
{
    _B = new_B;
}

template <typename F>
Triangle<F>& Triangle<F>::setPointC(const Point<F>& new_C)
{
    _C = new_C;
}

template <typename F>
Triangle<F>& Triangle<F>::setPoints(const Point<F>& new_A,
                                    const Point<F>& new_B,
                                    const Point<F>& new_C)
{
    _A = new_A; _B = new_B; _C = new_C;
}

template <typename F>
Triangle<F>& Triangle<F>::translate(F dx, F dy)
{
    _A.translate(dx, dy);
    _B.translate(dx, dy);
    _C.translate(dx, dy);
    return *this;
}

template <typename F>
Triangle<F>& Triangle<F>::translate(const Vector<F>& displacement)
{
    _A += displacement;
    _B += displacement;
    _C += displacement;
    return *this;
}

template <typename F>
Triangle<F>& Triangle<F>::precomputeBarycentricQuantities()
{
    _AB = _B - _A;
    _AC = _C - _A;

    _normalization = 1/(_AB.x*_AC.y - _AC.x*_AB.y);

    return *this;
}

template <typename F>
AxisAlignedRectangle<F> Triangle<F>::getAABB() const
{
    Point<F> min_point = _A;
    Point<F> max_point = min_point;

    min_point.useSmallestCoordinates(_B);
    max_point.useLargestCoordinates(_B);
    min_point.useSmallestCoordinates(_C);
    max_point.useLargestCoordinates(_C);

    return AxisAlignedRectangle<F>(min_point, max_point);
}

template <typename F>
AxisAlignedRectangle<F> Triangle<F>::getAABB(const Point<F>& lower_corner_limit,
                                             const Point<F>& upper_corner_limit) const
{
    Point<F> min_point = _A;
    Point<F> max_point = min_point;

    min_point.useSmallestCoordinates(_B);
    max_point.useLargestCoordinates(_B);
    min_point.useSmallestCoordinates(_C);
    max_point.useLargestCoordinates(_C);

    min_point.useLargestCoordinates(lower_corner_limit);
    max_point.useSmallestCoordinates(upper_corner_limit);

    return AxisAlignedRectangle<F>(min_point, max_point);
}

template <typename F>
std::string Triangle<F>::toString() const
{
    std::ostringstream string_stream;
    string_stream << "<" << _A.toString() << ", " << _B.toString() << ", " << _C.toString() << ">";
    return string_stream.str();
}

} // Geometry2D
