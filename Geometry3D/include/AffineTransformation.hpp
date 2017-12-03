#pragma once
#include "precision.hpp"
#include "LinearTransformation.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Triangle3.hpp"
#include "Plane.hpp"
#include "Ray.hpp"
#include "Box.hpp"
#include "CoordinateFrame.hpp"
#include "Matrix4.hpp"
#include <string>

namespace Impact {
namespace Geometry3D {

class AffineTransformation {

protected:
    Matrix4 _matrix;

public:
    AffineTransformation();
    AffineTransformation(const Matrix4& new_matrix);
    AffineTransformation(const LinearTransformation& other);

	static AffineTransformation identity();
    static AffineTransformation translation(imp_float dx, imp_float dy, imp_float dz);
    static AffineTransformation translation(const Vector& displacement);
    static AffineTransformation translationTo(const Point& position);
    static AffineTransformation rotationAboutRay(const Ray& ray, imp_float angle);
    static AffineTransformation pointAndVectorsToPointAndVectors(const Point& from_pt,
                                                                 const Vector& from_vec_1,
                                                                 const Vector& from_vec_2,
                                                                 const Vector& from_vec_3,
                                                                 const Point& to_pt,
                                                                 const Vector& to_vec_1,
                                                                 const Vector& to_vec_2,
                                                                 const Vector& to_vec_3);
    static AffineTransformation pointsToPoints(const Point& from_pt_1,
                                               const Point& from_pt_2,
                                               const Point& from_pt_3,
                                               const Point& from_pt_4,
                                               const Point& to_pt_1,
                                               const Point& to_pt_2,
                                               const Point& to_pt_3,
                                               const Point& to_pt_4);
    static AffineTransformation toCoordinateFrame(const CoordinateFrame& cframe);
    static AffineTransformation windowing(imp_float width, imp_float height);
    
    AffineTransformation operator()(const LinearTransformation& other) const;
    AffineTransformation operator()(const AffineTransformation& other) const;
    ProjectiveTransformation operator()(const ProjectiveTransformation& other) const;

    Point operator()(const Point& point) const;
    Vector operator()(const Vector& vector) const;
    Triangle operator()(const Triangle& triangle) const;
    Plane operator()(const Plane& plane) const;
    Ray operator()(const Ray& ray) const;
    Box operator()(const Box& box) const;

	AffineTransformation& setToIdentity();
	
    AffineTransformation& invert();
    AffineTransformation getInverse() const;

    const Matrix4& getMatrix() const;
    Matrix3 getNormalTransformMatrix() const;
};

} // Geometry3D
} // Impact
