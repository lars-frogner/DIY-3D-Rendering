#include "Triangle3.hpp"
#include "affine_combinations.hpp"
#include <armadillo>
#include <cassert>
#include <sstream>

namespace Impact {
namespace Geometry3D {

Triangle::Triangle(const Point& new_A,
                   const Point& new_B,
                   const Point& new_C)
    : _A(new_A), _B(new_B), _C(new_C) {}

Point Triangle::operator()(imp_float alpha, imp_float beta, imp_float gamma) const
{
    return getAffineCombination(_A, _B, _C, alpha, beta, gamma);
}

Point Triangle::operator()(imp_float s, imp_float t) const
{
    imp_float one_minus_s = 1 - s;
    return getAffineCombination(_A, _B, _C,
                                one_minus_s*(1 - t), one_minus_s*t, s);
}

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

const Vector& Triangle::getNormalVector() const
{
    return _normal;
}

imp_float Triangle::getArea() const
{
    return _area;
}

Vector Triangle::areaVector(const Point& A,
                            const Point& B,
                            const Point& C)
{
    // Return normal vector with length equal to area
    return Vector((A.y*B.z - A.z*B.y + B.y*C.z - B.z*C.y + C.y*A.z - C.z*A.y)*0.5f,
                  (A.z*B.x - A.x*B.z + B.z*C.x - B.x*C.z + C.z*A.x - C.x*A.z)*0.5f,
                  (A.x*B.y - A.y*B.x + B.x*C.y - B.y*C.x + C.x*A.y - C.y*A.x)*0.5f);
}

Vector Triangle::getNormalVector(const Point& A,
								 const Point& B,
								 const Point& C)
{
    return Vector((A.y*B.z - A.z*B.y + B.y*C.z - B.z*C.y + C.y*A.z - C.z*A.y),
                  (A.z*B.x - A.x*B.z + B.z*C.x - B.x*C.z + C.z*A.x - C.x*A.z),
                  (A.x*B.y - A.y*B.x + B.x*C.y - B.y*C.x + C.x*A.y - C.y*A.x)).getNormalized();

}

void Triangle::getBarycentricCoordinatesInside(const Point X,
                                               imp_float& alpha, imp_float& beta, imp_float& gamma) const
{
    /*
    The projection of X on the triangle plane is assumed to
    lie inside the triangle.
    */

    //const imp_float eps = -1.e-3f;

    gamma = _AB_normal.dot(X - _B);
    //assert(gamma >= eps);
    beta = _AC_normal.dot(X - _C);
    //assert(beta >= eps);
    alpha = 1 - (beta + gamma);
    //assert(alpha >= eps);
}

void Triangle::getBarycentricCoordinates(const Point X,
                                         imp_float& alpha, imp_float& beta, imp_float& gamma) const
{
    arma::Mat<imp_float> coefficients = {{_A.x, _B.x, _C.x}, {_A.y, _B.y, _C.y}, {_A.z, _B.z, _C.z}, {1, 1, 1}};
    arma::Col<imp_float> rhs = {X.x, X.y, X.z, 1};
    arma::Col<imp_float> coordinates = arma::pinv(coefficients)*rhs;
    alpha = coordinates(0);
    beta = coordinates(1);
    gamma = coordinates(2);
}

Triangle& Triangle::setPointA(const Point& new_A)
{
    _A = new_A;
    return computeNormalVectors();
}

Triangle& Triangle::setPointB(const Point& new_B)
{
    _B = new_B;
    return computeNormalVectors();
}

Triangle& Triangle::setPointC(const Point& new_C)
{
    _C = new_C;
    return computeNormalVectors();
}

Triangle& Triangle::setPoints(const Point& new_A,
                              const Point& new_B,
                              const Point& new_C)
{
    _A = new_A; _B = new_B; _C = new_C;
    return computeNormalVectors();
}

Triangle& Triangle::translate(imp_float dx, imp_float dy, imp_float dz)
{
    _A.translate(dx, dy, dz);
    _B.translate(dx, dy, dz);
    _C.translate(dx, dy, dz);
    return *this;
}

Triangle& Triangle::translate(const Vector& displacement)
{
    _A += displacement;
    _B += displacement;
    _C += displacement;
    return *this;
}

Triangle& Triangle::computeNormalVectors()
{
    _normal = areaVector(_A, _B, _C);
    _area = _normal.getLength();

    if (_area <= 0)
	{
		_is_degenerate = true;
		return *this;
	}

	_is_degenerate = false;

    _normal /= _area;

    const Vector& AB = _B - _A;
    const Vector& AC = _C - _A;
    _AB_normal = _normal.cross(AB);
    _AB_normal /= _AB_normal.dot(AC);
    _AC_normal = AC.cross(_normal);
    _AC_normal /= _AC_normal.dot(AB);

    return *this;
}

bool Triangle::isDegenerate() const
{
	return _is_degenerate;
}

Plane Triangle::getPlane() const
{
    return Plane(_A, _B - _A, _C - _A);
}

Plane Triangle::getPlaneNoBasis() const
{
    return Plane(_A, _normal);
}

AxisAlignedBox Triangle::getAABB() const
{
    Point min_point = _A;
    Point max_point = min_point;

    min_point.useSmallestCoordinates(_B);
    max_point.useLargestCoordinates(_B);
    min_point.useSmallestCoordinates(_C);
    max_point.useLargestCoordinates(_C);

    return AxisAlignedBox(min_point, max_point);
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

} // Geometry3D
} // Impact
