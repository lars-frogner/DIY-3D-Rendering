#include "LinearTransformation.hpp"
#include "Matrix4.hpp"
#include "AffineTransformation.hpp"
#include "ProjectiveTransformation.hpp"
#include <armadillo>
#include <cmath>

namespace Impact {
namespace Geometry3D {

LinearTransformation::LinearTransformation()
    : _matrix() {}

LinearTransformation::LinearTransformation(const Matrix3& new_matrix)
    : _matrix(new_matrix) {}

LinearTransformation LinearTransformation::identity()
{
	return LinearTransformation();
}

LinearTransformation LinearTransformation::scaling(imp_float scale_x, imp_float scale_y, imp_float scale_z)
{
	assert(scale_x != 0 && scale_y != 0 && scale_z != 0);

    Matrix3 matrix;

    matrix.a11 = scale_x;
    matrix.a22 = scale_y;
    matrix.a33 = scale_z;
	
    return LinearTransformation(matrix);
}

LinearTransformation LinearTransformation::scaling(imp_float scale)
{
    return scaling(scale, scale, scale);
}

LinearTransformation LinearTransformation::rotation(const Vector& axis, imp_float angle)
{
    Matrix3 matrix;

    Matrix3 cross_product_matrix(      0, -axis.z,  axis.y,
                                  axis.z,       0, -axis.x,
                                 -axis.y,  axis.x,       0);

    matrix += sin(angle)*cross_product_matrix +
              (1 - cos(angle))*cross_product_matrix*cross_product_matrix;
	
    return LinearTransformation(matrix);
}

LinearTransformation LinearTransformation::rotation(const Quaternion& q)
{
    return LinearTransformation(Matrix3(1 - 2*(q.y*q.y + q.z*q.z),	   2*(q.x*q.y + q.z*q.w),	  2*(q.x*q.z - q.y*q.w),
											2*(q.x*q.y - q.z*q.w), 1 - 2*(q.x*q.x + q.z*q.z),	  2*(q.y*q.z + q.x*q.w),
											2*(q.x*q.z + q.y*q.w),	   2*(q.y*q.z - q.x*q.w), 1 - 2*(q.x*q.x + q.y*q.y)));
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

    return rotation(axis, angle);
}

LinearTransformation LinearTransformation::rotationFromXToY(imp_float angle)
{
    imp_float cos_angle = cos(angle);
    imp_float sin_angle = sin(angle);
    
    Matrix3 matrix;

    matrix.a11 = cos_angle;
    matrix.a12 = -sin_angle;
    matrix.a21 = sin_angle;
    matrix.a22 = cos_angle;
	
    return LinearTransformation(matrix);
}

LinearTransformation LinearTransformation::rotationFromYToZ(imp_float angle)
{
    imp_float cos_angle = cos(angle);
    imp_float sin_angle = sin(angle);
    
    Matrix3 matrix;

    matrix.a22 = cos_angle;
    matrix.a23 = -sin_angle;
    matrix.a32 = sin_angle;
    matrix.a33 = cos_angle;
	
    return LinearTransformation(matrix);
}

LinearTransformation LinearTransformation::rotationFromZToX(imp_float angle)
{
    imp_float cos_angle = cos(angle);
    imp_float sin_angle = sin(angle);
    
    Matrix3 matrix;

    matrix.a11 = cos_angle;
    matrix.a13 = sin_angle;
    matrix.a31 = -sin_angle;
    matrix.a33 = cos_angle;
	
    return LinearTransformation(matrix);
}

LinearTransformation LinearTransformation::shear(imp_float dxdy, imp_float dxdz, imp_float dydx, imp_float dydz, imp_float dzdx, imp_float dzdy)
{
    Matrix3 matrix;

    matrix.a12 = dxdy;
    matrix.a13 = dxdz;
    matrix.a21 = dydx;
    matrix.a23 = dydz;
    matrix.a31 = dzdx;
    matrix.a32 = dzdy;
	
    return LinearTransformation(matrix);
}

LinearTransformation LinearTransformation::vectorsToVectors(const Vector& from_vec_1,
                                                            const Vector& from_vec_2,
                                                            const Vector& from_vec_3,
                                                            const Vector& to_vec_1,
                                                            const Vector& to_vec_2,
                                                            const Vector& to_vec_3)
{
    Matrix3 from_mat(from_vec_1, from_vec_2, from_vec_3);
    Matrix3 to_mat(to_vec_1, to_vec_2, to_vec_3);

    const Matrix3& matrix = to_mat*(from_mat.getInverse());
	
    return LinearTransformation(matrix);
}

LinearTransformation LinearTransformation::operator()(const LinearTransformation& other) const
{
    return LinearTransformation(_matrix*other._matrix);
}

AffineTransformation LinearTransformation::operator()(const AffineTransformation& other) const
{
    return AffineTransformation(Matrix4(_matrix)*other.getMatrix());
}

ProjectiveTransformation LinearTransformation::operator()(const ProjectiveTransformation& other) const
{

    return ProjectiveTransformation(_matrix.toArma4x4Matrix()*other.getMatrix());
}

Point LinearTransformation::operator()(const Point& point) const
{
    return _matrix*point;
}

Vector LinearTransformation::operator()(const Vector& vector) const
{
    return _matrix*vector;
}

Triangle LinearTransformation::operator()(const Triangle& triangle) const
{
    return Triangle(_matrix*triangle.getPointA(),
                    _matrix*triangle.getPointB(),
                    _matrix*triangle.getPointC());
}

Plane LinearTransformation::operator()(const Plane& plane) const
{
    if (plane.hasBasis())
    {
        Plane new_plane(_matrix*plane.origin,
                        _matrix*plane.getBasisVector1(),
                        _matrix*plane.getBasisVector2());
		
		return new_plane;
    }
    else
    {
        Plane new_plane(_matrix*plane.origin,
                        (_matrix*plane.getNormalVector()).normalize());
		
		return new_plane;
    }
}

Ray LinearTransformation::operator()(const Ray& ray) const
{
    return Ray(_matrix*ray.origin, (_matrix*ray.direction).normalize());
}

Box LinearTransformation::operator()(const Box& box) const
{
    return Box(_matrix*box.origin,
               _matrix*box.getWidthVector(),
               _matrix*box.getHeightVector(),
               _matrix*box.getDepthVector());
}

LinearTransformation& LinearTransformation::setToIdentity()
{
	_matrix.setToIdentity();
    return *this;
}

LinearTransformation& LinearTransformation::invert()
{
	_matrix.invert();
    return *this;
}

LinearTransformation LinearTransformation::getInverse() const
{
    return LinearTransformation(_matrix.getInverse());
}

const Matrix3& LinearTransformation::getMatrix() const
{
    return _matrix;
}

Matrix3 LinearTransformation::getNormalTransformMatrix() const
{
    return _matrix.getTranspose().getInverse();
}

} // Geometry3D
} // Impact
