#include "AffineTransformation.hpp"
#include "ProjectiveTransformation.hpp"
#include <armadillo>

namespace Impact {
namespace Geometry3D {

AffineTransformation::AffineTransformation()
    : _matrix() {}

AffineTransformation::AffineTransformation(const Matrix4& new_matrix)
    : _matrix(new_matrix) {}

AffineTransformation::AffineTransformation(const LinearTransformation& other)
	: _matrix(other.getMatrix()) {}

AffineTransformation AffineTransformation::identity()
{
	return AffineTransformation();
}

AffineTransformation AffineTransformation::translation(imp_float dx, imp_float dy, imp_float dz)
{
    Matrix4 matrix;

    matrix.a14 = dx;
    matrix.a24 = dy;
    matrix.a34 = dz;

    return AffineTransformation(matrix);
}

AffineTransformation AffineTransformation::translation(const Vector& displacement)
{
    Matrix4 matrix;

    matrix.a14 = displacement.x;
    matrix.a24 = displacement.y;
    matrix.a34 = displacement.z;
	
    return AffineTransformation(matrix);
}

AffineTransformation AffineTransformation::translationTo(const Point& position)
{
    Matrix4 matrix;

    matrix.a14 = position.x;
    matrix.a24 = position.y;
    matrix.a34 = position.z;
	
    return AffineTransformation(matrix);
}

AffineTransformation AffineTransformation::rotationAboutRay(const Ray& ray, imp_float angle)
{
    return translation(ray.origin.x, ray.origin.y, ray.origin.z)(
           LinearTransformation::rotation(ray.direction, angle)(
           translation(-ray.origin.x, -ray.origin.y, -ray.origin.z)));
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
    return translation(to_pt.x, to_pt.y, to_pt.z)(
           LinearTransformation::vectorsToVectors(from_vec_1, from_vec_2, from_vec_3,
                                                  to_vec_1, to_vec_2, to_vec_3)(
           translation(-from_pt.x, -from_pt.y, -from_pt.z)));
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
	
    Matrix4 matrix;

    matrix.a11 = half_width;
    matrix.a14 = half_width;
    matrix.a22 = half_height;
    matrix.a24 = half_height;
	
    return AffineTransformation(matrix);
}

AffineTransformation AffineTransformation::operator()(const LinearTransformation& other) const
{
    return AffineTransformation(_matrix*Matrix4(other.getMatrix()));
}

AffineTransformation AffineTransformation::operator()(const AffineTransformation& other) const
{
    return AffineTransformation(_matrix*other._matrix);
}

ProjectiveTransformation AffineTransformation::operator()(const ProjectiveTransformation& other) const
{
	return ProjectiveTransformation(_matrix.toArma4x4Matrix()*other.getMatrix());
}

Point AffineTransformation::operator()(const Point& point) const
{
    return _matrix*point;
}

Vector AffineTransformation::operator()(const Vector& vector) const
{
    return _matrix*vector;
}

Triangle AffineTransformation::operator()(const Triangle& triangle) const
{
    return Triangle(_matrix*triangle.getPointA(),
                    _matrix*triangle.getPointB(),
                    _matrix*triangle.getPointC());
}

Plane AffineTransformation::operator()(const Plane& plane) const
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

Ray AffineTransformation::operator()(const Ray& ray) const
{
    return Ray(_matrix*ray.origin, (_matrix*ray.direction).normalize());
}

Box AffineTransformation::operator()(const Box& box) const
{
    return Box(_matrix*box.origin,
               _matrix*box.getWidthVector(),
               _matrix*box.getHeightVector(),
               _matrix*box.getDepthVector());
}

AffineTransformation& AffineTransformation::setToIdentity()
{
	_matrix.setToIdentity();
	return *this;
}

AffineTransformation& AffineTransformation::invert()
{
	_matrix.invert();
    return *this;
}

AffineTransformation AffineTransformation::getInverse() const
{
    return AffineTransformation(_matrix.getInverse());
}

const Matrix4& AffineTransformation::getMatrix() const
{
    return _matrix;
}

Matrix3 AffineTransformation::getNormalTransformMatrix() const
{
    return _matrix.getLinearPart().getTranspose().getInverse();
}

} // Geometry3D
} // Impact
