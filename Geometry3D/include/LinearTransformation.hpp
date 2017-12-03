#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Quaternion.hpp"
#include "Triangle3.hpp"
#include "Plane.hpp"
#include "Ray.hpp"
#include "Box.hpp"
#include "Matrix3.hpp"
#include <string>

namespace Impact {
namespace Geometry3D {

// Forward declaration of AffineTransformation class
class AffineTransformation;

// Forward declaration of ProjectiveTransformation class
class ProjectiveTransformation;

class LinearTransformation {

protected:
    Matrix3 _matrix;

public:
    LinearTransformation();
    LinearTransformation(const Matrix3& new_matrix);
    
	static LinearTransformation identity();
    static LinearTransformation scaling(imp_float scale_x, imp_float scale_y, imp_float scale_z);
    static LinearTransformation scaling(imp_float scale);
    static LinearTransformation rotation(const Vector& axis, imp_float angle);
    static LinearTransformation rotation(const Quaternion& quaternion);
    static LinearTransformation rotationFromVectorToVector(const Vector& from_vector,
                                                           const Vector& to_vector);
    static LinearTransformation rotationFromXToY(imp_float angle);
    static LinearTransformation rotationFromYToZ(imp_float angle);
    static LinearTransformation rotationFromZToX(imp_float angle);
    static LinearTransformation shear(imp_float dxdy, imp_float dxdz, imp_float dydx, imp_float dydz, imp_float dzdx, imp_float dzdy);
    static LinearTransformation vectorsToVectors(const Vector& from_vec_1,
                                                 const Vector& from_vec_2,
                                                 const Vector& from_vec_3,
                                                 const Vector& to_vec_1,
                                                 const Vector& to_vec_2,
                                                 const Vector& to_vec_3);

    LinearTransformation operator()(const LinearTransformation& other) const;
    AffineTransformation operator()(const AffineTransformation& other) const;
    ProjectiveTransformation operator()(const ProjectiveTransformation& other) const;

    Point operator()(const Point& point) const;
    Vector operator()(const Vector& vector) const;
    Triangle operator()(const Triangle& triangle) const;
    Plane operator()(const Plane& plane) const;
    Ray operator()(const Ray& ray) const;
    Box operator()(const Box& box) const;
	
	LinearTransformation& setToIdentity();

	LinearTransformation& invert();
    LinearTransformation getInverse() const;

    const Matrix3& getMatrix() const;
    Matrix3 getNormalTransformMatrix() const;
};

} // Geometry3D
} // Impact
