#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"
#include "ProjectiveTransformation.hpp"
#include <cmath>

namespace Impact {
namespace Geometry3D {

LinearTransformation::LinearTransformation()
    : _matrix(4, 4, arma::fill::eye),
      _normal_transform_matrix(3, 3, arma::fill::eye) {}

LinearTransformation::LinearTransformation(const arma::Mat<imp_float>& new_matrix)
    : _matrix(new_matrix),
      _normal_transform_matrix(new_matrix.submat(0, 0, 2, 2).t().i()) {}

LinearTransformation LinearTransformation::scaling(imp_float scale_x, imp_float scale_y, imp_float scale_z)
{
    LinearTransformation transformation;
    transformation._matrix(0, 0) = scale_x;
    transformation._matrix(1, 1) = scale_y;
    transformation._matrix(2, 2) = scale_z;

    return transformation;
}

LinearTransformation LinearTransformation::scaling(imp_float scale)
{
    return scaling(scale, scale, scale);
}

LinearTransformation LinearTransformation::rotationAboutAxis(const Vector& axis, imp_float angle)
{
    arma::Mat<imp_float> cross_product_matrix = {{0, -axis.z, axis.y},
                                                 {axis.z, 0, -axis.x},
                                                 {-axis.y, axis.x, 0}};
        
    LinearTransformation transformation;
    transformation._matrix.submat(0, 0, 2, 2) = transformation._matrix.submat(0, 0, 2, 2) +
                                                sin(angle)*cross_product_matrix +
                                                (1 - cos(angle))*cross_product_matrix*cross_product_matrix;

    return transformation;
}

LinearTransformation LinearTransformation::rotationFromVectorToVector(const Vector& from_vector,
                                                                      const Vector& to_vector)
{
    const imp_float eps = static_cast<imp_float>(1e-6);
    Vector reference = Vector::zero();

    imp_float angle = acos(from_vector.getNormalized().dot(to_vector.getNormalized()));

    if (angle < eps)
    {
        return LinearTransformation();
    }
    else if (IMP_PI - angle < eps)
    {
        imp_uint smallest_idx = from_vector.getSmallestComponentIndex();
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

LinearTransformation LinearTransformation::rotationFromXToY(imp_float angle)
{
    imp_float cos_angle = cos(angle);
    imp_float sin_angle = sin(angle);
    
    LinearTransformation transformation;
    transformation._matrix(0, 0) = cos_angle;
    transformation._matrix(0, 1) = -sin_angle;
    transformation._matrix(1, 0) = sin_angle;
    transformation._matrix(1, 1) = cos_angle;

    return transformation;
}

LinearTransformation LinearTransformation::rotationFromYToZ(imp_float angle)
{
    imp_float cos_angle = cos(angle);
    imp_float sin_angle = sin(angle);
    
    LinearTransformation transformation;
    transformation._matrix(1, 1) = cos_angle;
    transformation._matrix(1, 2) = -sin_angle;
    transformation._matrix(2, 1) = sin_angle;
    transformation._matrix(2, 2) = cos_angle;

    return transformation;
}

LinearTransformation LinearTransformation::rotationFromZToX(imp_float angle)
{
    imp_float cos_angle = cos(angle);
    imp_float sin_angle = sin(angle);
    
    LinearTransformation transformation;
    transformation._matrix(0, 0) = cos_angle;
    transformation._matrix(0, 2) = sin_angle;
    transformation._matrix(2, 0) = -sin_angle;
    transformation._matrix(2, 2) = cos_angle;

    return transformation;
}

LinearTransformation LinearTransformation::shear(imp_float dxdy, imp_float dxdz, imp_float dydx, imp_float dydz, imp_float dzdx, imp_float dzdy)
{
    LinearTransformation transformation;
    transformation._matrix(0, 1) = dxdy;
    transformation._matrix(0, 2) = dxdz;
    transformation._matrix(1, 0) = dydx;
    transformation._matrix(1, 2) = dydz;
    transformation._matrix(2, 0) = dzdx;
    transformation._matrix(2, 1) = dzdy;

    return transformation;
}

LinearTransformation LinearTransformation::vectorsToVectors(const Vector& from_vec_1,
                                                            const Vector& from_vec_2,
                                                            const Vector& from_vec_3,
                                                            const Vector& to_vec_1,
                                                            const Vector& to_vec_2,
                                                            const Vector& to_vec_3)
{
    arma::Mat<imp_float> from_mat = {{from_vec_1.x, from_vec_2.x, from_vec_3.x},
									 {from_vec_1.y, from_vec_2.y, from_vec_3.y},
									 {from_vec_1.z, from_vec_2.z, from_vec_3.z}};
    
    arma::Mat<imp_float> to_mat = {{to_vec_1.x, to_vec_2.x, to_vec_3.x},
								   {to_vec_1.y, to_vec_2.y, to_vec_3.y},
								   {to_vec_1.z, to_vec_2.z, to_vec_3.z}};
    
    LinearTransformation transformation;
    transformation._matrix.submat(0, 0, 2, 2) = to_mat*(from_mat.i());

    return transformation;
}

LinearTransformation LinearTransformation::operator*(const LinearTransformation& other) const
{
    LinearTransformation transformation;
    transformation._matrix.submat(0, 0, 2, 2) = _matrix.submat(0, 0, 2, 2)*other._matrix.submat(0, 0, 2, 2);
    return transformation;
}

AffineTransformation LinearTransformation::operator*(const AffineTransformation& other) const
{
    return AffineTransformation(_matrix*other._matrix);
}

ProjectiveTransformation LinearTransformation::operator*(const ProjectiveTransformation& other) const
{
    return ProjectiveTransformation(_matrix*other._matrix);
}

Point LinearTransformation::operator*(const Point& point) const
{
    return Point(_matrix(0, 0)*point.x + _matrix(0, 1)*point.y + _matrix(0, 2)*point.z,
                 _matrix(1, 0)*point.x + _matrix(1, 1)*point.y + _matrix(1, 2)*point.z,
                 _matrix(2, 0)*point.x + _matrix(2, 1)*point.y + _matrix(2, 2)*point.z);
}

Vector LinearTransformation::operator*(const Vector& vector) const
{
    return Vector(_matrix(0, 0)*vector.x + _matrix(0, 1)*vector.y + _matrix(0, 2)*vector.z,
                  _matrix(1, 0)*vector.x + _matrix(1, 1)*vector.y + _matrix(1, 2)*vector.z,
                  _matrix(2, 0)*vector.x + _matrix(2, 1)*vector.y + _matrix(2, 2)*vector.z);
}

Triangle LinearTransformation::operator*(const Triangle& triangle) const
{
    return Triangle((*this)*triangle.getPointA(),
                    (*this)*triangle.getPointB(),
                    (*this)*triangle.getPointC());
}

Plane LinearTransformation::operator*(const Plane& plane) const
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

Ray LinearTransformation::operator*(const Ray& ray) const
{
    return Ray((*this)*ray.origin, ((*this)*ray.direction).normalize());
}

Box LinearTransformation::operator*(const Box& box) const
{
    return Box((*this)*box.origin,
               (*this)*box.getWidthVector(),
               (*this)*box.getHeightVector(),
               (*this)*box.getDepthVector());
}

LinearTransformation LinearTransformation::getInverse() const
{
    LinearTransformation transformation;
    transformation._matrix.submat(0, 0, 2, 2) = _matrix.submat(0, 0, 2, 2).i();
    return transformation;
}

const arma::Mat<imp_float>& LinearTransformation::getMatrix() const
{
    return _matrix;
}

const arma::Mat<imp_float>& LinearTransformation::getNormalTransformMatrix() const
{
    return _normal_transform_matrix;
}

std::string LinearTransformation::getTransformationType() const
{
    return std::string("Linear transformation");
}

} // Geometry3D
} // Impact
