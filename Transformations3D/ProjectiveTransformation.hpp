#pragma once
#include <armadillo>
#include <string>
#include <assert.h>
#include "Transformation.hpp"
#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"
#include "../Geometry3D/Point.hpp"
#include "../Geometry3D/Triangle.hpp"

namespace Transformations3D {

template <typename F>
class ProjectiveTransformation : public Transformation<F> {

friend LinearTransformation<F>;
friend AffineTransformation<F>;

private:
    typedef Geometry3D::Point<F> Point;
    typedef Geometry3D::Triangle<F> Triangle;

protected:
    arma::Mat<F> _matrix;

    ProjectiveTransformation<F>(const arma::Mat<F>& new_matrix);

public:
    ProjectiveTransformation<F>();
    ProjectiveTransformation<F>(const LinearTransformation<F>& other);
    ProjectiveTransformation<F>(const AffineTransformation<F>& other);

    static ProjectiveTransformation<F> pointsToPoints(const Point& from_pt_1,
                                                      const Point& from_pt_2,
                                                      const Point& from_pt_3,
                                                      const Point& from_pt_4,
                                                      const Point& from_pt_5,
                                                      const Point& to_pt_1,
                                                      const Point& to_pt_2,
                                                      const Point& to_pt_3,
                                                      const Point& to_pt_4,
                                                      const Point& to_pt_5);
    static ProjectiveTransformation<F> unhinging(F near_plane_distance,
                                                 F far_plane_distance);
    
    ProjectiveTransformation<F> operator*(const LinearTransformation<F>& other) const;
    ProjectiveTransformation<F> operator*(const AffineTransformation<F>& other) const;
    ProjectiveTransformation<F> operator*(const ProjectiveTransformation<F>& other) const;

    Point operator*(const Point& point) const;
    Triangle operator*(const Triangle& triangle) const;

    ProjectiveTransformation<F> getInverse() const;
    const arma::Mat<F>& getMatrix() const;

    std::string getTransformationType() const;
};

template <typename F>
ProjectiveTransformation<F>::ProjectiveTransformation()
    : _matrix(4, 4, arma::fill::eye) {}

template <typename F>
ProjectiveTransformation<F>::ProjectiveTransformation(const LinearTransformation<F>& other)
    : _matrix(other._matrix) {}

template <typename F>
ProjectiveTransformation<F>::ProjectiveTransformation(const AffineTransformation<F>& other)
    : _matrix(other._matrix) {}

template <typename F>
ProjectiveTransformation<F>::ProjectiveTransformation(const arma::Mat<F>& new_matrix)
    : _matrix(new_matrix) {}

template <typename F>
ProjectiveTransformation<F> ProjectiveTransformation<F>::pointsToPoints(const Point& from_pt_1,
                                                                        const Point& from_pt_2,
                                                                        const Point& from_pt_3,
                                                                        const Point& from_pt_4,
                                                                        const Point& from_pt_5,
                                                                        const Point& to_pt_1,
                                                                        const Point& to_pt_2,
                                                                        const Point& to_pt_3,
                                                                        const Point& to_pt_4,
                                                                        const Point& to_pt_5)
{
    arma::Mat<F> from_mat = {{from_pt_1.x, from_pt_2.x, from_pt_3.x, from_pt_4.x},
                             {from_pt_1.y, from_pt_2.y, from_pt_3.y, from_pt_4.y},
                             {from_pt_1.z, from_pt_2.z, from_pt_3.z, from_pt_4.z},
                             {     1,           1,           1,           1     }};

    arma::Col<F> from_vec_5 = {from_pt_5.x, from_pt_5.y, from_pt_5.z, 1};

    const arma::Col<F>& from_coefs = arma::solve(from_mat, from_vec_5);

    from_mat.col(0) = from_mat.col(0)*from_coefs(0);
    from_mat.col(1) = from_mat.col(1)*from_coefs(1);
    from_mat.col(2) = from_mat.col(2)*from_coefs(2);
    from_mat.col(3) = from_mat.col(3)*from_coefs(3);

    arma::Mat<F> to_mat = {{to_pt_1.x, to_pt_2.x, to_pt_3.x, to_pt_4.x},
                           {to_pt_1.y, to_pt_2.y, to_pt_3.y, to_pt_4.y},
                           {to_pt_1.z, to_pt_2.z, to_pt_3.z, to_pt_4.z},
                           {    1,         1,         1,         1    }};

    arma::Col<F> to_vec_5 = {to_pt_5.x, to_pt_5.y, to_pt_5.z, 1};

    const arma::Col<F>& to_coefs = arma::solve(to_mat, to_vec_5);

    to_mat.col(0) = to_mat.col(0)*to_coefs(0);
    to_mat.col(1) = to_mat.col(1)*to_coefs(1);
    to_mat.col(2) = to_mat.col(2)*to_coefs(2);
    to_mat.col(3) = to_mat.col(3)*to_coefs(3);

    return ProjectiveTransformation<F>(to_mat*(from_mat.i()));
}

template <typename F>
ProjectiveTransformation<F> ProjectiveTransformation<F>::unhinging(F near_plane_distance,
                                                                   F far_plane_distance)
{
    F difference = far_plane_distance - near_plane_distance;

    arma::Mat<F> matrix(4, 4, arma::fill::zeros);
    matrix(0, 0) = difference;
    matrix(1, 1) = difference;
    matrix(2, 2) = far_plane_distance;
    matrix(2, 3) = near_plane_distance;
    matrix(3, 2) = -difference;

    return ProjectiveTransformation<F>(matrix);
}

template <typename F>
ProjectiveTransformation<F> ProjectiveTransformation<F>::operator*(const LinearTransformation<F>& other) const
{
    return ProjectiveTransformation<F>(_matrix*other._matrix);
}

template <typename F>
ProjectiveTransformation<F> ProjectiveTransformation<F>::operator*(const AffineTransformation<F>& other) const
{
    return ProjectiveTransformation<F>(_matrix*other._matrix);
}

template <typename F>
ProjectiveTransformation<F> ProjectiveTransformation<F>::operator*(const ProjectiveTransformation<F>& other) const
{
    return ProjectiveTransformation<F>(_matrix*other._matrix);
}

template <typename F>
Geometry3D::Point<F> ProjectiveTransformation<F>::operator*(const Point& point) const
{
    F w = _matrix(3, 0)*point.x + _matrix(3, 1)*point.y + _matrix(3, 2)*point.z + _matrix(3, 3);
    assert(w != 0);
    F w_inv = 1/w;

    return Point((_matrix(0, 0)*point.x + _matrix(0, 1)*point.y + _matrix(0, 2)*point.z + _matrix(0, 3))*w_inv,
                 (_matrix(1, 0)*point.x + _matrix(1, 1)*point.y + _matrix(1, 2)*point.z + _matrix(1, 3))*w_inv,
                 (_matrix(2, 0)*point.x + _matrix(2, 1)*point.y + _matrix(2, 2)*point.z + _matrix(2, 3))*w_inv);
}

template <typename F>
Geometry3D::Triangle<F> ProjectiveTransformation<F>::operator*(const Triangle& triangle) const
{
    return Triangle((*this)*triangle.getPointA(),
                    (*this)*triangle.getPointB(),
                    (*this)*triangle.getPointC());
}

template <typename F>
ProjectiveTransformation<F> ProjectiveTransformation<F>::getInverse() const
{
    return ProjectiveTransformation<F>(_matrix.i());
}

template <typename F>
const arma::Mat<F>& ProjectiveTransformation<F>::getMatrix() const
{
    return _matrix;
}

template <typename F>
std::string ProjectiveTransformation<F>::getTransformationType() const
{
    return std::string("Projective transformation");
}

} // Transformations3D
