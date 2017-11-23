#pragma once
#include "precision.hpp"
#include "Transformation.hpp"
#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"
#include "Point3.hpp"
#include "Triangle3.hpp"
#include <armadillo>
#include <string>

namespace Impact {
namespace Geometry3D {

class ProjectiveTransformation : public Transformation {

friend LinearTransformation;
friend AffineTransformation;

protected:
    arma::Mat<imp_float> _matrix;

    ProjectiveTransformation(const arma::Mat<imp_float>& new_matrix);

public:
    ProjectiveTransformation();
    ProjectiveTransformation(const LinearTransformation& other);
    ProjectiveTransformation(const AffineTransformation& other);

    static ProjectiveTransformation pointsToPoints(const Point& from_pt_1,
                                                   const Point& from_pt_2,
                                                   const Point& from_pt_3,
                                                   const Point& from_pt_4,
                                                   const Point& from_pt_5,
                                                   const Point& to_pt_1,
                                                   const Point& to_pt_2,
                                                   const Point& to_pt_3,
                                                   const Point& to_pt_4,
                                                   const Point& to_pt_5);
    static ProjectiveTransformation unhinging(imp_float near_plane_distance,
                                              imp_float far_plane_distance);
    
    ProjectiveTransformation operator*(const LinearTransformation& other) const;
    ProjectiveTransformation operator*(const AffineTransformation& other) const;
    ProjectiveTransformation operator*(const ProjectiveTransformation& other) const;

    Point operator*(const Point& point) const;
    Triangle operator*(const Triangle& triangle) const;

    ProjectiveTransformation getInverse() const;
    const arma::Mat<imp_float>& getMatrix() const;

    std::string getTransformationType() const;
};

} // Geometry3D
} // Impact
