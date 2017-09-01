#pragma once
#include <armadillo>
#include <assert.h>
#include <string>
#include <sstream>
#include "Point.hpp"
#include "Vector.hpp"
#include "Plane.hpp"
#include "AxisAlignedBox.hpp"
#include "affine_combinations.hpp"

namespace Geometry3D {

template <typename F>
class Triangle {

private:
    Point<F> _A, _B, _C;
    Vector<F> _normal, _AB_normal, _AC_normal;
    F _area;

public:
    Triangle<F>(const Point<F>& new_A,
                const Point<F>& new_B,
                const Point<F>& new_C);

    Point<F> operator()(F alpha, F beta, F gamma) const;
    Point<F> operator()(F s, F t) const;

    const Point<F>& getPointA() const;
    const Point<F>& getPointB() const;
    const Point<F>& getPointC() const;
    const Vector<F>& getNormalVector() const;
    F getArea() const;

    static Vector<F> areaVector(const Point<F>& A,
                                const Point<F>& B,
                                const Point<F>& C);

    void getBarycentricCoordinatesInside(const Point<F> X,
                                         F& alpha, F& beta, F& gamma) const;

    void getBarycentricCoordinates(const Point<F> X,
                                   F& alpha, F& beta, F& gamma) const;

    Triangle<F>& setPointA(const Point<F>& new_A);
    Triangle<F>& setPointB(const Point<F>& new_B);
    Triangle<F>& setPointC(const Point<F>& new_C);
    Triangle<F>& setPoints(const Point<F>& new_A,
                           const Point<F>& new_B,
                           const Point<F>& new_C);

    Triangle<F>& translate(F dx, F dy, F dz);
    Triangle<F>& translate(const Vector<F>& displacement);

    Triangle<F>& computeNormals();

    Plane<F> getPlane() const;
    Plane<F> getPlaneNoBasis() const;
    AxisAlignedBox<F> getAABB() const;

    std::string toString() const;
};

template <typename F>
Triangle<F>::Triangle(const Point<F>& new_A,
                      const Point<F>& new_B,
                      const Point<F>& new_C)
    : _A(new_A), _B(new_B), _C(new_C) {}

template <typename F>
Point<F> Triangle<F>::operator()(F alpha, F beta, F gamma) const
{
    return getAffineCombination(_A, _B, _C, alpha, beta, gamma);
}

template <typename F>
Point<F> Triangle<F>::operator()(F s, F t) const
{
    F one_minus_s = 1 - s;
    return getAffineCombination(_A, _B, _C,
                                one_minus_s*(1 - t), one_minus_s*t, s);
}

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
const Vector<F>& Triangle<F>::getNormalVector() const
{
    return _normal;
}

template <typename F>
F Triangle<F>::getArea() const
{
    return _area;
}

template <typename F>
Vector<F> Triangle<F>::areaVector(const Point<F>& A,
                                  const Point<F>& B,
                                  const Point<F>& C)
{
    // Return normal vector with length equal to area
    return Vector<F>((A.y*B.z - A.z*B.y + B.y*C.z - B.z*C.y + C.y*A.z - C.z*A.y)*0.5f,
                     (A.z*B.x - A.x*B.z + B.z*C.x - B.x*C.z + C.z*A.x - C.x*A.z)*0.5f,
                     (A.x*B.y - A.y*B.x + B.x*C.y - B.y*C.x + C.x*A.y - C.y*A.x)*0.5f);
}

template <typename F>
void Triangle<F>::getBarycentricCoordinatesInside(const Point<F> X,
                                                  F& alpha, F& beta, F& gamma) const
{
    /*
    The projection of X on the triangle plane is assumed to
    lie inside the triangle.
    */

    const F eps = -1.e-4f;

    gamma = _AB_normal.dot(X - _B);
    assert(gamma >= eps);
    beta = _AC_normal.dot(X - _C);
    assert(beta >= eps);
    alpha = 1 - (beta + gamma);
    assert(alpha >= eps);
}

template <typename F>
void Triangle<F>::getBarycentricCoordinates(const Point<F> X,
                                            F& alpha, F& beta, F& gamma) const
{
    arma::Mat<F> coefficients = {{_A.x, _B.x, _C.x}, {_A.y, _B.y, _C.y}, {_A.z, _B.z, _C.z}, {1, 1, 1}};
    arma::Col<F> rhs = {X.x, X.y, X.z, 1};
    arma::Col<F> coordinates = arma::pinv(coefficients)*rhs;
    alpha = coordinates(0);
    beta = coordinates(1);
    gamma = coordinates(2);
}

template <typename F>
Triangle<F>& Triangle<F>::setPointA(const Point<F>& new_A)
{
    _A = new_A;
    return computeNormals();
}

template <typename F>
Triangle<F>& Triangle<F>::setPointB(const Point<F>& new_B)
{
    _B = new_B;
    return computeNormals();
}

template <typename F>
Triangle<F>& Triangle<F>::setPointC(const Point<F>& new_C)
{
    _C = new_C;
    return computeNormals();
}

template <typename F>
Triangle<F>& Triangle<F>::setPoints(const Point<F>& new_A,
                                    const Point<F>& new_B,
                                    const Point<F>& new_C)
{
    _A = new_A; _B = new_B; _C = new_C;
    return computeNormals();
}

template <typename F>
Triangle<F>& Triangle<F>::translate(F dx, F dy, F dz)
{
    _A.translate(dx, dy, dz);
    _B.translate(dx, dy, dz);
    _C.translate(dx, dy, dz);
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
Triangle<F>& Triangle<F>::computeNormals()
{
    _normal = areaVector(_A, _B, _C);
    _area = _normal.getLength();
    assert(_area > 0);
    _normal /= _area;

    const Vector<F>& AB = _B - _A;
    const Vector<F>& AC = _C - _A;
    _AB_normal = _normal.cross(AB);
    _AB_normal /= _AB_normal.dot(AC);
    _AC_normal = AC.cross(_normal);
    _AC_normal /= _AC_normal.dot(AB);

    return *this;
}

template <typename F>
Plane<F> Triangle<F>::getPlane() const
{
    return Plane<F>(_A, _B - _A, _C - _A);
}

template <typename F>
Plane<F> Triangle<F>::getPlaneNoBasis() const
{
    return Plane<F>(_A, _normal);
}

template <typename F>
AxisAlignedBox<F> Triangle<F>::getAABB() const
{
    Point<F> min_point = _A;
    Point<F> max_point = min_point;

    min_point.useSmallestCoordinates(_B);
    max_point.useLargestCoordinates(_B);
    min_point.useSmallestCoordinates(_C);
    max_point.useLargestCoordinates(_C);

    return AxisAlignedBox<F>(min_point, max_point);
}

template <typename F>
std::string Triangle<F>::toString() const
{
    std::ostringstream string_stream;
    string_stream << "<" << _A.toString() << ", " << _B.toString() << ", " << _C.toString() << ">";
    return string_stream.str();
}

} // Geometry3D
