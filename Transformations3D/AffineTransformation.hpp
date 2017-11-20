#pragma once
#include <armadillo>
#include <string>
#include "Transformation.hpp"
#include "LinearTransformation.hpp"
#include "../Geometry3D/Point.hpp"
#include "../Geometry3D/Vector.hpp"
#include "../Geometry3D/Triangle.hpp"
#include "../Geometry3D/Plane.hpp"
#include "../Geometry3D/Ray.hpp"
#include "../Geometry3D/Box.hpp"
#include "../Geometry3D/CoordinateFrame.hpp"

namespace Transformations3D {

template <typename F>
class AffineTransformation : public Transformation<F> {

friend LinearTransformation<F>;
friend ProjectiveTransformation<F>;

private:
    typedef Geometry3D::Point<F> Point;
    typedef Geometry3D::Vector<F> Vector;
    typedef Geometry3D::Triangle<F> Triangle;
    typedef Geometry3D::Plane<F> Plane;
    typedef Geometry3D::Ray<F> Ray;
    typedef Geometry3D::Box<F> Box;
    typedef Geometry3D::CoordinateFrame<F> CoordinateFrame;
    typedef Geometry3D::Camera<F> Camera;

protected:
    arma::Mat<F> _matrix;
    arma::Mat<F> _normal_transform_matrix;

    AffineTransformation<F>(const arma::Mat<F>& new_matrix);

public:
    AffineTransformation<F>();

    static AffineTransformation<F> translation(F dx, F dy, F dz);
    static AffineTransformation<F> translation(const Vector& displacement);
    static AffineTransformation<F> rotationAboutRay(const Ray& ray, F angle);
    static AffineTransformation<F> pointAndVectorsToPointAndVectors(const Point& from_pt,
                                                                    const Vector& from_vec_1,
                                                                    const Vector& from_vec_2,
                                                                    const Vector& from_vec_3,
                                                                    const Point& to_pt,
                                                                    const Vector& to_vec_1,
                                                                    const Vector& to_vec_2,
                                                                    const Vector& to_vec_3);
    static AffineTransformation<F> pointsToPoints(const Point& from_pt_1,
                                                  const Point& from_pt_2,
                                                  const Point& from_pt_3,
                                                  const Point& from_pt_4,
                                                  const Point& to_pt_1,
                                                  const Point& to_pt_2,
                                                  const Point& to_pt_3,
                                                  const Point& to_pt_4);
    static AffineTransformation<F> toCoordinateFrame(const CoordinateFrame& cframe);
    static AffineTransformation<F> windowing(F width, F height);
    
    AffineTransformation<F> operator*(const LinearTransformation<F>& other) const;
    AffineTransformation<F> operator*(const AffineTransformation<F>& other) const;
    ProjectiveTransformation<F> operator*(const ProjectiveTransformation<F>& other) const;

    Point operator*(const Point& point) const;
    Vector operator*(const Vector& vector) const;
    Triangle operator*(const Triangle& triangle) const;
    Plane operator*(const Plane& plane) const;
    Ray operator*(const Ray& ray) const;
    Box operator*(const Box& box) const;
    Camera operator*(const Camera& camera) const;

    Vector normalTransform(const Vector& normal) const;

	void setToIdentity();

    AffineTransformation<F> getInverse() const;
    const arma::Mat<F>& getMatrix() const;
    const arma::Mat<F>& getNormalTransformMatrix() const;

    std::string getTransformationType() const;
};

template <typename F>
AffineTransformation<F>::AffineTransformation()
    : _matrix(4, 4, arma::fill::eye),
      _normal_transform_matrix(3, 3, arma::fill::eye) {}

template <typename F>
AffineTransformation<F>::AffineTransformation(const arma::Mat<F>& new_matrix)
    : _matrix(new_matrix),
      _normal_transform_matrix(new_matrix.submat(0, 0, 2, 2).t().i()) {}

template <typename F>
AffineTransformation<F> AffineTransformation<F>::translation(F dx, F dy, F dz)
{
    AffineTransformation<F> transformation;
    transformation._matrix(0, 3) = dx;
    transformation._matrix(1, 3) = dy;
    transformation._matrix(2, 3) = dz;

    return transformation;
}

template <typename F>
AffineTransformation<F> AffineTransformation<F>::translation(const Vector& displacement)
{
    AffineTransformation<F> transformation;
    transformation._matrix(0, 3) = displacement.x;
    transformation._matrix(1, 3) = displacement.y;
    transformation._matrix(2, 3) = displacement.z;

    return transformation;
}

template <typename F>
AffineTransformation<F> AffineTransformation<F>::rotationAboutRay(const Ray& ray, F angle)
{
    return translation(ray.origin.x, ray.origin.y, ray.origin.z)
           *LinearTransformation<F>::rotationAboutAxis(ray.direction, angle)
           *translation(-ray.origin.x, -ray.origin.y, -ray.origin.z);
}

template <typename F>
AffineTransformation<F> AffineTransformation<F>::pointAndVectorsToPointAndVectors(const Point& from_pt,
                                                                                  const Vector& from_vec_1,
                                                                                  const Vector& from_vec_2,
                                                                                  const Vector& from_vec_3,
                                                                                  const Point& to_pt,
                                                                                  const Vector& to_vec_1,
                                                                                  const Vector& to_vec_2,
                                                                                  const Vector& to_vec_3)
{
    return translation(to_pt.x, to_pt.y, to_pt.z)
           *LinearTransformation<F>::vectorsToVectors(from_vec_1, from_vec_2, from_vec_3,
                                                      to_vec_1, to_vec_2, to_vec_3)
           *translation(-from_pt.x, -from_pt.y, -from_pt.z);
}

template <typename F>
AffineTransformation<F> AffineTransformation<F>::pointsToPoints(const Point& from_pt_1,
                                                                const Point& from_pt_2,
                                                                const Point& from_pt_3,
                                                                const Point& from_pt_4,
                                                                const Point& to_pt_1,
                                                                const Point& to_pt_2,
                                                                const Point& to_pt_3,
                                                                const Point& to_pt_4)
{
    return pointAndVectorsToPointAndVectors(from_pt_1,
                                            from_pt_2 - from_pt_1,
                                            from_pt_3 - from_pt_1,
                                            from_pt_4 - from_pt_1,
                                            to_pt_1,
                                            to_pt_2 - to_pt_1,
                                            to_pt_3 - to_pt_1,
                                            to_pt_4 - to_pt_1);
}

template <typename F>
AffineTransformation<F> AffineTransformation<F>::toCoordinateFrame(const CoordinateFrame& cframe)
{
    return pointAndVectorsToPointAndVectors(cframe.origin,   cframe.basis_1,  cframe.basis_2,  cframe.basis_3,
                                            Point::origin(), Vector::unitX(), Vector::unitY(), Vector::unitZ());
}

template <typename F>
AffineTransformation<F> AffineTransformation<F>::windowing(F width, F height)
{
    F half_width = width/2;
    F half_height = height/2;

    AffineTransformation<F> transformation;
    transformation._matrix(0, 0) = half_width;
    transformation._matrix(0, 3) = half_width;
    transformation._matrix(1, 1) = half_height;
    transformation._matrix(1, 3) = half_height;

    return transformation;
}

template <typename F>
AffineTransformation<F> AffineTransformation<F>::operator*(const LinearTransformation<F>& other) const
{
    return AffineTransformation<F>(_matrix*other._matrix);
}

template <typename F>
AffineTransformation<F> AffineTransformation<F>::operator*(const AffineTransformation<F>& other) const
{
    return AffineTransformation<F>(_matrix*other._matrix);
}

template <typename F>
ProjectiveTransformation<F> AffineTransformation<F>::operator*(const ProjectiveTransformation<F>& other) const
{
    return ProjectiveTransformation<F>(_matrix*other._matrix);
}

template <typename F>
Geometry3D::Point<F> AffineTransformation<F>::operator*(const Point& point) const
{
    return Point(_matrix(0, 0)*point.x + _matrix(0, 1)*point.y + _matrix(0, 2)*point.z + _matrix(0, 3),
                 _matrix(1, 0)*point.x + _matrix(1, 1)*point.y + _matrix(1, 2)*point.z + _matrix(1, 3),
                 _matrix(2, 0)*point.x + _matrix(2, 1)*point.y + _matrix(2, 2)*point.z + _matrix(2, 3));
}

template <typename F>
Geometry3D::Vector<F> AffineTransformation<F>::operator*(const Vector& vector) const
{
    return Vector(_matrix(0, 0)*vector.x + _matrix(0, 1)*vector.y + _matrix(0, 2)*vector.z,
                  _matrix(1, 0)*vector.x + _matrix(1, 1)*vector.y + _matrix(1, 2)*vector.z,
                  _matrix(2, 0)*vector.x + _matrix(2, 1)*vector.y + _matrix(2, 2)*vector.z);
}

template <typename F>
Geometry3D::Triangle<F> AffineTransformation<F>::operator*(const Triangle& triangle) const
{
    return Triangle((*this)*triangle.getPointA(),
                    (*this)*triangle.getPointB(),
                    (*this)*triangle.getPointC());
}

template <typename F>
Geometry3D::Plane<F> AffineTransformation<F>::operator*(const Plane& plane) const
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
Geometry3D::Ray<F> AffineTransformation<F>::operator*(const Ray& ray) const
{
    return Ray((*this)*ray.origin, ((*this)*ray.direction).normalize());
}

template <typename F>
Geometry3D::Box<F> AffineTransformation<F>::operator*(const Box& box) const
{
    return Box((*this)*box.origin,
               (*this)*box.getWidthVector(),
               (*this)*box.getHeightVector(),
               (*this)*box.getDepthVector());
}

template <typename F>
Geometry3D::Camera<F> AffineTransformation<F>::operator*(const Camera& camera) const
{
    return Camera((*this)*camera.ray,
                  (*this)*camera.up_direction,
                  camera.near_plane_distance,
                  camera.far_plane_distance,
                  camera.field_of_view,
                  camera.aspect_ratio);
}

template <typename F>
Geometry3D::Vector<F> AffineTransformation<F>::normalTransform(const Vector& normal) const
{
    return Vector(_normal_transform_matrix(0, 0)*normal.x + _normal_transform_matrix(0, 1)*normal.y + _normal_transform_matrix(0, 2)*normal.z,
                  _normal_transform_matrix(1, 0)*normal.x + _normal_transform_matrix(1, 1)*normal.y + _normal_transform_matrix(1, 2)*normal.z,
                  _normal_transform_matrix(2, 0)*normal.x + _normal_transform_matrix(2, 1)*normal.y + _normal_transform_matrix(2, 2)*normal.z);
}

template <typename F>
void AffineTransformation<F>::setToIdentity()
{
	_matrix.eye();
	_normal_transform_matrix.eye();
}

template <typename F>
AffineTransformation<F> AffineTransformation<F>::getInverse() const
{
    return AffineTransformation<F>(_matrix.i());
}

template <typename F>
const arma::Mat<F>& AffineTransformation<F>::getMatrix() const
{
    return _matrix;
}

template <typename F>
const arma::Mat<F>& AffineTransformation<F>::getNormalTransformMatrix() const
{
    return _normal_transform_matrix;
}

template <typename F>
std::string AffineTransformation<F>::getTransformationType() const
{
    return std::string("Affine transformation");
}

} // Transformations3D
