#include "Triangle2.hpp"
#include <cassert>
#include <sstream>

namespace Impact {
namespace Geometry2D {

Triangle::Triangle(const Point& new_A,
                   const Point& new_B,
                   const Point& new_C)
    : _A(new_A), _B(new_B), _C(new_C) {}

const Point& Triangle::getPointA() const
{
    return _A;
}

const Point& Triangle::getPointB() const
{
    return _B;
}

const Point& Triangle::getPointC() const
{
    return _C;
}

void Triangle::getBarycentricCoordinates(const Point X,
                                         imp_float& alpha, imp_float& beta, imp_float& gamma) const
{
    const Vector& AX = X - _A;

    beta = (AX.x*_AC.y - _AC.x*AX.y)*_normalization;
    gamma = (_AB.x*AX.y - AX.x*_AB.y)*_normalization;
    alpha = 1 - beta - gamma;
}

Triangle& Triangle::setPointA(const Point& new_A)
{
    _A = new_A;
	return *this;
}

Triangle& Triangle::setPointB(const Point& new_B)
{
    _B = new_B;
	return *this;
}

Triangle& Triangle::setPointC(const Point& new_C)
{
    _C = new_C;
	return *this;
}

Triangle& Triangle::setPoints(const Point& new_A,
                              const Point& new_B,
                              const Point& new_C)
{
    _A = new_A; _B = new_B; _C = new_C;
	return *this;
}

Triangle& Triangle::translate(imp_float dx, imp_float dy)
{
    _A.translate(dx, dy);
    _B.translate(dx, dy);
    _C.translate(dx, dy);
    return *this;
}

Triangle& Triangle::translate(const Vector& displacement)
{
    _A += displacement;
    _B += displacement;
    _C += displacement;
    return *this;
}

Triangle& Triangle::precomputeBarycentricQuantities()
{
    _AB = _B - _A;
    _AC = _C - _A;

    _normalization = 1/(_AB.x*_AC.y - _AC.x*_AB.y);

    return *this;
}

AxisAlignedRectangle Triangle::getAABR() const
{
    Point min_point = _A;
    Point max_point = min_point;

    min_point.useSmallestCoordinates(_B);
    max_point.useLargestCoordinates(_B);
    min_point.useSmallestCoordinates(_C);
    max_point.useLargestCoordinates(_C);

    return AxisAlignedRectangle(min_point, max_point);
}

AxisAlignedRectangle Triangle::getAABR(const Point& lower_corner_limit,
                                       const Point& upper_corner_limit) const
{
    Point min_point = _A;
    Point max_point = min_point;

    min_point.useSmallestCoordinates(_B);
    max_point.useLargestCoordinates(_B);
    min_point.useSmallestCoordinates(_C);
    max_point.useLargestCoordinates(_C);

    min_point.useLargestCoordinates(lower_corner_limit);
    max_point.useSmallestCoordinates(upper_corner_limit);

    return AxisAlignedRectangle(min_point, max_point);
}

Point Triangle::getCentroid() const
{
    return _A + ((_B - _A) + (_C - _A))/3;
}

std::string Triangle::toString() const
{
    std::ostringstream string_stream;
    string_stream << "<" << _A.toString() << ", " << _B.toString() << ", " << _C.toString() << ">";
    return string_stream.str();
}

} // Geometry2D
} // Impact
