#pragma once
#define _USE_MATH_DEFINES
#include <armadillo>
#include <math.h>
#include <string>
#include "Transformation.hpp"
#include "../Geometry3D/Point.hpp"
#include "../Geometry3D/Vector.hpp"
#include "../Geometry3D/Triangle.hpp"
#include "../Geometry3D/Plane.hpp"
#include "../Geometry3D/Ray.hpp"
#include "../Geometry3D/Box.hpp"

namespace Geometry3D {

// Forward declaration of Camera class
template <typename F>
class Camera;

} // Geometry3D

namespace Transformations3D {

// Forward declaration of AffineTransformation class
template <typename F>
class AffineTransformation;

// Forward declaration of ProjectiveTransformation class
template <typename F>
class ProjectiveTransformation;

template <typename F>
class LinearTransformation : public Transformation<F> {

friend AffineTransformation<F>;
friend ProjectiveTransformation<F>;

private:
    typedef Geometry3D::Point<F> Point;
    typedef Geometry3D::Vector<F> Vector;
    typedef Geometry3D::Triangle<F> Triangle;
    typedef Geometry3D::Plane<F> Plane;
    typedef Geometry3D::Ray<F> Ray;
    typedef Geometry3D::Box<F> Box;
    typedef Geometry3D::Camera<F> Camera;

protected:
    arma::Mat<F> _matrix;
    arma::Mat<F> _normal_transform_matrix;

    LinearTransformation<F>(const arma::Mat<F>& new_matrix);

public:
    LinearTransformation<F>();
    
    static LinearTransformation<F> scaling(F scale_x, F scale_y, F scale_z);
    static LinearTransformation<F> rotationAboutAxis(const Vector& axis, F angle);
    static LinearTransformation<F> rotationFromVectorToVector(const Vector& from_vector,
                                                              const Vector& to_vector);
    static LinearTransformation<F> rotationFromXToY(F angle);
    static LinearTransformation<F> rotationFromYToZ(F angle);
    static LinearTransformation<F> rotationFromZToX(F angle);
    static LinearTransformation<F> shear(F dxdy, F dxdz, F dydx, F dydz, F dzdx, F dzdy);
    static LinearTransformation<F> vectorsToVectors(const Vector& from_vec_1,
                                                    const Vector& from_vec_2,
                                                    const Vector& from_vec_3,
                                                    const Vector& to_vec_1,
                                                    const Vector& to_vec_2,
                                                    const Vector& to_vec_3);

    LinearTransformation<F> operator*(const LinearTransformation<F>& other) const;
    AffineTransformation<F> operator*(const AffineTransformation<F>& other) const;
    ProjectiveTransformation<F> operator*(const ProjectiveTransformation<F>& other) const;

    Point operator*(const Point& point) const;
    Vector operator*(const Vector& vector) const;
    Triangle operator*(const Triangle& triangle) const;
    Plane operator*(const Plane& plane) const;
    Ray operator*(const Ray& ray) const;
    Box operator*(const Box& box) const;
    Camera operator*(const Camera& camera) const;

    LinearTransformation<F> getInverse() const;
    const arma::Mat<F>& getMatrix() const;
    const arma::Mat<F>& getNormalTransformMatrix() const;

    std::string getTransformationType() const;
};

template <typename F>
LinearTransformation<F>::LinearTransformation()
    : _matrix(4, 4, arma::fill::eye),
      _normal_transform_matrix(3, 3, arma::fill::eye) {}

template <typename F>
LinearTransformation<F>::LinearTransformation(const arma::Mat<F>& new_matrix)
    : _matrix(new_matrix),
      _normal_transform_matrix(new_matrix.submat(0, 0, 2, 2).t().i()) {}

template <typename F>
LinearTransformation<F> LinearTransformation<F>::scaling(F scale_x, F scale_y, F scale_z)
{
    LinearTransformation<F> transformation;
    transformation._matrix(0, 0) = scale_x;
    transformation._matrix(1, 1) = scale_y;
    transformation._matrix(2, 2) = scale_z;

    return transformation;
}

template <typename F>
LinearTransformation<F> LinearTransformation<F>::rotationAboutAxis(const Vector& axis, F angle)
{
    arma::Mat<F> cross_product_matrix = {{0, -axis.z, axis.y},
                                         {axis.z, 0, -axis.x},
                                         {-axis.y, axis.x, 0}};
        
    LinearTransformation<F> transformation;
    transformation._matrix.submat(0, 0, 2, 2) = transformation._matrix.submat(0, 0, 2, 2) +
                                                sin(angle)*cross_product_matrix +
                                                (1 - cos(angle))*cross_product_matrix*cross_product_matrix;

    return transformation;
}

template <typename F>
LinearTransformation<F> LinearTransformation<F>::rotationFromVectorToVector(const Vector& from_vector,
                                                                            const Vector& to_vector)
{
    const F eps = static_cast<F>(1e-6);
    Vector reference = Vector::zero();

    F angle = acos(from_vector.getNormalized().dot(to_vector.getNormalized()));

    if (angle < eps)
    {
        return LinearTransformation<F>();
    }
    else if (M_PI - angle < eps)
    {
        size_t smallest_idx = from_vector.getSmallestComponentIndex();
        if      (smallest_idx == 0) reference.x = 1;
        else if (smallest_idx == 1) reference.y = 1;
        else                        reference.z = 1;
    }
    else
    {
        reference = to_vector;
    }
    
    const Vector& axis = from_vector.cross(reference).normalize();

    return rotationAboutAxis(axis, angle);
}

template <typename F>
LinearTransformation<F> LinearTransformation<F>::rotationFromXToY(F angle)
{
    F cos_angle = cos(angle);
    F sin_angle = sin(angle);
    
    LinearTransformation<F> transformation;
    transformation._matrix(0, 0) = cos_angle;
    transformation._matrix(0, 1) = -sin_angle;
    transformation._matrix(1, 0) = sin_angle;
    transformation._matrix(1, 1) = cos_angle;

    return transformation;
}

template <typename F>
LinearTransformation<F> LinearTransformation<F>::rotationFromYToZ(F angle)
{
    F cos_angle = cos(angle);
    F sin_angle = sin(angle);
    
    LinearTransformation<F> transformation;
    transformation._matrix(1, 1) = cos_angle;
    transformation._matrix(1, 2) = -sin_angle;
    transformation._matrix(2, 1) = sin_angle;
    transformation._matrix(2, 2) = cos_angle;

    return transformation;
}

template <typename F>
LinearTransformation<F> LinearTransformation<F>::rotationFromZToX(F angle)
{
    F cos_angle = cos(angle);
    F sin_angle = sin(angle);
    
    LinearTransformation<F> transformation;
    transformation._matrix(0, 0) = cos_angle;
    transformation._matrix(0, 2) = sin_angle;
    transformation._matrix(2, 0) = -sin_angle;
    transformation._matrix(2, 2) = cos_angle;

    return transformation;
}

template <typename F>
LinearTransformation<F> LinearTransformation<F>::shear(F dxdy, F dxdz, F dydx, F dydz, F dzdx, F dzdy)
{
    LinearTransformation<F> transformation;
    transformation._matrix(0, 1) = dxdy;
    transformation._matrix(0, 2) = dxdz;
    transformation._matrix(1, 0) = dydx;
    transformation._matrix(1, 2) = dydz;
    transformation._matrix(2, 0) = dzdx;
    transformation._matrix(2, 1) = dzdy;

    return transformation;
}

template <typename F>
LinearTransformation<F> LinearTransformation<F>::vectorsToVectors(const Vector& from_vec_1,
                                                                  const Vector& from_vec_2,
                                                                  const Vector& from_vec_3,
                                                                  const Vector& to_vec_1,
                                                                  const Vector& to_vec_2,
                                                                  const Vector& to_vec_3)
{
    arma::Mat<F> from_mat = {{from_vec_1.x, from_vec_2.x, from_vec_3.x},
                             {from_vec_1.y, from_vec_2.y, from_vec_3.y},
                             {from_vec_1.z, from_vec_2.z, from_vec_3.z}};
    
    arma::Mat<F> to_mat = {{to_vec_1.x, to_vec_2.x, to_vec_3.x},
                           {to_vec_1.y, to_vec_2.y, to_vec_3.y},
                           {to_vec_1.z, to_vec_2.z, to_vec_3.z}};
    
    LinearTransformation<F> transformation;
    transformation._matrix.submat(0, 0, 2, 2) = to_mat*(from_mat.i());

    return transformation;
}

template <typename F>
LinearTransformation<F> LinearTransformation<F>::operator*(const LinearTransformation<F>& other) const
{
    LinearTransformation<F> transformation;
    transformation._matrix.submat(0, 0, 2, 2) = _matrix.submat(0, 0, 2, 2)*other._matrix.submat(0, 0, 2, 2);
    return transformation;
}

template <typename F>
AffineTransformation<F> LinearTransformation<F>::operator*(const AffineTransformation<F>& other) const
{
    return AffineTransformation<F>(_matrix*other._matrix);
}

template <typename F>
ProjectiveTransformation<F> LinearTransformation<F>::operator*(const ProjectiveTransformation<F>& other) const
{
    return ProjectiveTransformation<F>(_matrix*other._matrix);
}

template <typename F>
Geometry3D::Point<F> LinearTransformation<F>::operator*(const Point& point) const
{
    return Point(_matrix(0, 0)*point.x + _matrix(0, 1)*point.y + _matrix(0, 2)*point.z,
                 _matrix(1, 0)*point.x + _matrix(1, 1)*point.y + _matrix(1, 2)*point.z,
                 _matrix(2, 0)*point.x + _matrix(2, 1)*point.y + _matrix(2, 2)*point.z);
}

template <typename F>
Geometry3D::Vector<F> LinearTransformation<F>::operator*(const Vector& vector) const
{
    return Vector(_matrix(0, 0)*vector.x + _matrix(0, 1)*vector.y + _matrix(0, 2)*vector.z,
                  _matrix(1, 0)*vector.x + _matrix(1, 1)*vector.y + _matrix(1, 2)*vector.z,
                  _matrix(2, 0)*vector.x + _matrix(2, 1)*vector.y + _matrix(2, 2)*vector.z);
}

template <typename F>
Geometry3D::Triangle<F> LinearTransformation<F>::operator*(const Triangle& triangle) const
{
    return Triangle((*this)*triangle.getPointA(),
                    (*this)*triangle.getPointB(),
                    (*this)*triangle.getPointC());
}

template <typename F>
Geometry3D::Plane<F> LinearTransformation<F>::operator*(const Plane& plane) const
{
    if (plane.hasBasis())
    {
        Plane new_plane((*this)*plane.origin,
                        (*this)*plane.getBasisVector1(),
                        (*this)*plane.getBasisVector2());
    }
    else
    {
        Plane new_plane((*this)*plane.origin,
                        ((*this)*plane.getNormalVector()).normalize());
    }
    return new_plane;
}

template <typename F>
Geometry3D::Ray<F> LinearTransformation<F>::operator*(const Ray& ray) const
{
    return Ray((*this)*ray.origin, ((*this)*ray.direction).normalize());
}

template <typename F>
Geometry3D::Box<F> LinearTransformation<F>::operator*(const Box& box) const
{
    return Box((*this)*box.origin,
               (*this)*box.getWidthVector(),
               (*this)*box.getHeightVector(),
               (*this)*box.getDepthVector());
}

template <typename F>
Geometry3D::Camera<F> LinearTransformation<F>::operator*(const Camera& camera) const
{
    return Camera((*this)*camera.ray,
                  (*this)*camera.up_direction,
                  camera.near_plane_distance,
                  camera.far_plane_distance,
                  camera.field_of_view,
                  camera.aspect_ratio);
}

template <typename F>
LinearTransformation<F> LinearTransformation<F>::getInverse() const
{
    LinearTransformation<F> transformation;
    transformation._matrix.submat(0, 0, 2, 2) = _matrix.submat(0, 0, 2, 2).i();
    return transformation;
}

template <typename F>
const arma::Mat<F>& LinearTransformation<F>::getMatrix() const
{
    return _matrix;
}

template <typename F>
const arma::Mat<F>& LinearTransformation<F>::getNormalTransformMatrix() const
{
    return _normal_transform_matrix;
}

template <typename F>
std::string LinearTransformation<F>::getTransformationType() const
{
    return std::string("Linear transformation");
}

} // Transformations3D
