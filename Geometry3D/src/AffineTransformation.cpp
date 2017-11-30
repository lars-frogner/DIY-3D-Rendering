#include "AffineTransformation.hpp"
#include "ProjectiveTransformation.hpp"

namespace Impact {
namespace Geometry3D {

AffineTransformation::AffineTransformation()
    : _matrix(4, 4, arma::fill::eye),
      _normal_transform_matrix(3, 3, arma::fill::eye) {}

AffineTransformation::AffineTransformation(const LinearTransformation& other)
	: _matrix(other._matrix),
	  _normal_transform_matrix(other._normal_transform_matrix) {}

AffineTransformation::AffineTransformation(const arma::Mat<imp_float>& new_matrix)
    : _matrix(new_matrix),
      _normal_transform_matrix(new_matrix.submat(0, 0, 2, 2).t().i()) {}

AffineTransformation::AffineTransformation(const arma::Mat<imp_float>& new_matrix,
										   const arma::Mat<imp_float>& new_normal_transform_matrix)
    : _matrix(new_matrix),
      _normal_transform_matrix(new_normal_transform_matrix) {}

AffineTransformation AffineTransformation::translation(imp_float dx, imp_float dy, imp_float dz)
{
    arma::Mat<imp_float> matrix(4, 4, arma::fill::eye);

    matrix(0, 3) = dx;
    matrix(1, 3) = dy;
    matrix(2, 3) = dz;

    return AffineTransformation(matrix);
}

AffineTransformation AffineTransformation::translation(const Vector& displacement)
{
    arma::Mat<imp_float> matrix(4, 4, arma::fill::eye);

    matrix(0, 3) = displacement.x;
    matrix(1, 3) = displacement.y;
    matrix(2, 3) = displacement.z;
	
    return AffineTransformation(matrix);
}

AffineTransformation AffineTransformation::translationTo(const Point& position)
{
    arma::Mat<imp_float> matrix(4, 4, arma::fill::eye);

    matrix(0, 3) = position.x;
    matrix(1, 3) = position.y;
    matrix(2, 3) = position.z;
	
    return AffineTransformation(matrix);
}

AffineTransformation AffineTransformation::rotationAboutRay(const Ray& ray, imp_float angle)
{
    return translation(ray.origin.x, ray.origin.y, ray.origin.z)
           *LinearTransformation::rotation(ray.direction, angle)
           *translation(-ray.origin.x, -ray.origin.y, -ray.origin.z);
}

AffineTransformation AffineTransformation::pointAndVectorsToPointAndVectors(const Point& from_pt,
                                                                            const Vector& from_vec_1,
                                                                            const Vector& from_vec_2,
                                                                            const Vector& from_vec_3,
                                                                            const Point& to_pt,
                                                                            const Vector& to_vec_1,
                                                                            const Vector& to_vec_2,
                                                                            const Vector& to_vec_3)
{
    return translation(to_pt.x, to_pt.y, to_pt.z)
           *LinearTransformation::vectorsToVectors(from_vec_1, from_vec_2, from_vec_3,
                                                   to_vec_1, to_vec_2, to_vec_3)
           *translation(-from_pt.x, -from_pt.y, -from_pt.z);
}

AffineTransformation AffineTransformation::pointsToPoints(const Point& from_pt_1,
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

AffineTransformation AffineTransformation::toCoordinateFrame(const CoordinateFrame& cframe)
{
    return pointAndVectorsToPointAndVectors(cframe.origin,   cframe.basis_1,  cframe.basis_2,  cframe.basis_3,
                                            Point::origin(), Vector::unitX(), Vector::unitY(), Vector::unitZ());
}

AffineTransformation AffineTransformation::windowing(imp_float width, imp_float height)
{
    imp_float half_width = width/2;
    imp_float half_height = height/2;
	
    arma::Mat<imp_float> matrix(4, 4, arma::fill::eye);

    matrix(0, 0) = half_width;
    matrix(0, 3) = half_width;
    matrix(1, 1) = half_height;
    matrix(1, 3) = half_height;
	
    return AffineTransformation(matrix);
}

AffineTransformation AffineTransformation::operator*(const LinearTransformation& other) const
{
    return AffineTransformation(_matrix*other._matrix,
								_normal_transform_matrix*other._normal_transform_matrix);
}

AffineTransformation AffineTransformation::operator*(const AffineTransformation& other) const
{
    return AffineTransformation(_matrix*other._matrix,
								_normal_transform_matrix*other._normal_transform_matrix);
}

ProjectiveTransformation AffineTransformation::operator*(const ProjectiveTransformation& other) const
{
    return ProjectiveTransformation(_matrix*other._matrix);
}

Point AffineTransformation::operator*(const Point& point) const
{
    return Point(_matrix(0, 0)*point.x + _matrix(0, 1)*point.y + _matrix(0, 2)*point.z + _matrix(0, 3),
                 _matrix(1, 0)*point.x + _matrix(1, 1)*point.y + _matrix(1, 2)*point.z + _matrix(1, 3),
                 _matrix(2, 0)*point.x + _matrix(2, 1)*point.y + _matrix(2, 2)*point.z + _matrix(2, 3));
}

Vector AffineTransformation::operator*(const Vector& vector) const
{
    return Vector(_matrix(0, 0)*vector.x + _matrix(0, 1)*vector.y + _matrix(0, 2)*vector.z,
                  _matrix(1, 0)*vector.x + _matrix(1, 1)*vector.y + _matrix(1, 2)*vector.z,
                  _matrix(2, 0)*vector.x + _matrix(2, 1)*vector.y + _matrix(2, 2)*vector.z);
}

Triangle AffineTransformation::operator*(const Triangle& triangle) const
{
    return Triangle((*this)*triangle.getPointA(),
                    (*this)*triangle.getPointB(),
                    (*this)*triangle.getPointC());
}

Plane AffineTransformation::operator*(const Plane& plane) const
{
    if (plane.hasBasis())
    {
        Plane new_plane((*this)*plane.origin,
                        (*this)*plane.getBasisVector1(),
                        (*this)*plane.getBasisVector2());

		return new_plane;
    }
    else
    {
        Plane new_plane((*this)*plane.origin,
                        ((*this)*plane.getNormalVector()).normalize());

		return new_plane;
    }
}

Ray AffineTransformation::operator*(const Ray& ray) const
{
    return Ray((*this)*ray.origin, ((*this)*ray.direction).normalize());
}

Box AffineTransformation::operator*(const Box& box) const
{
    return Box((*this)*box.origin,
               (*this)*box.getWidthVector(),
               (*this)*box.getHeightVector(),
               (*this)*box.getDepthVector());
}

Vector AffineTransformation::normalTransform(const Vector& normal) const
{
    return Vector(_normal_transform_matrix(0, 0)*normal.x + _normal_transform_matrix(0, 1)*normal.y + _normal_transform_matrix(0, 2)*normal.z,
                  _normal_transform_matrix(1, 0)*normal.x + _normal_transform_matrix(1, 1)*normal.y + _normal_transform_matrix(1, 2)*normal.z,
                  _normal_transform_matrix(2, 0)*normal.x + _normal_transform_matrix(2, 1)*normal.y + _normal_transform_matrix(2, 2)*normal.z);
}

void AffineTransformation::setToIdentity()
{
	_matrix.eye();
	_normal_transform_matrix.eye();
}

AffineTransformation AffineTransformation::getInverse() const
{
    return AffineTransformation(_matrix.i(), _normal_transform_matrix.i());
}

const arma::Mat<imp_float>& AffineTransformation::getMatrix() const
{
    return _matrix;
}

const arma::Mat<imp_float>& AffineTransformation::getNormalTransformMatrix() const
{
    return _normal_transform_matrix;
}

std::string AffineTransformation::getTransformationType() const
{
    return std::string("Affine transformation");
}

} // Geometry3D
} // Impact
