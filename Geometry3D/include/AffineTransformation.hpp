#pragma once
#include "precision.hpp"
#include "Transformation.hpp"
#include "LinearTransformation.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Triangle3.hpp"
#include "Plane.hpp"
#include "Ray.hpp"
#include "Box.hpp"
#include "CoordinateFrame.hpp"
#include <armadillo>
#include <string>

namespace Impact {
namespace Geometry3D {

class AffineTransformation : public Transformation {

friend LinearTransformation;
friend ProjectiveTransformation;

protected:
    arma::Mat<imp_float> _matrix;
    arma::Mat<imp_float> _normal_transform_matrix;

    AffineTransformation(const arma::Mat<imp_float>& new_matrix);

public:
    AffineTransformation();

    static AffineTransformation translation(imp_float dx, imp_float dy, imp_float dz);
    static AffineTransformation translation(const Vector& displacement);
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
    
    AffineTransformation operator*(const LinearTransformation& other) const;
    AffineTransformation operator*(const AffineTransformation& other) const;
    ProjectiveTransformation operator*(const ProjectiveTransformation& other) const;

    Point operator*(const Point& point) const;
    Vector operator*(const Vector& vector) const;
    Triangle operator*(const Triangle& triangle) const;
    Plane operator*(const Plane& plane) const;
    Ray operator*(const Ray& ray) const;
    Box operator*(const Box& box) const;

    Vector normalTransform(const Vector& normal) const;

	void setToIdentity();

    AffineTransformation getInverse() const;
    const arma::Mat<imp_float>& getMatrix() const;
    const arma::Mat<imp_float>& getNormalTransformMatrix() const;

    std::string getTransformationType() const;
};

} // Geometry3D
} // Impact
