#include "ProjectiveTransformation.hpp"
#include <cassert>

namespace Impact {
namespace Geometry3D {

ProjectiveTransformation::ProjectiveTransformation()
    : _matrix(4, 4, arma::fill::eye) {}

ProjectiveTransformation::ProjectiveTransformation(const LinearTransformation& other)
    : _matrix(other.getMatrix().toArma4x4Matrix()) {}

ProjectiveTransformation::ProjectiveTransformation(const AffineTransformation& other)
    : _matrix(other.getMatrix().toArma4x4Matrix()) {}

ProjectiveTransformation::ProjectiveTransformation(const arma::Mat<imp_float>& new_matrix)
    : _matrix(new_matrix) {}

ProjectiveTransformation ProjectiveTransformation::pointsToPoints(const Point& from_pt_1,
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
    arma::Mat<imp_float> from_mat = {{from_pt_1.x, from_pt_2.x, from_pt_3.x, from_pt_4.x},
									 {from_pt_1.y, from_pt_2.y, from_pt_3.y, from_pt_4.y},
									 {from_pt_1.z, from_pt_2.z, from_pt_3.z, from_pt_4.z},
									 {     1,           1,           1,           1     }};

    arma::Col<imp_float> from_vec_5 = {from_pt_5.x, from_pt_5.y, from_pt_5.z, 1};

    const arma::Col<imp_float>& from_coefs = arma::solve(from_mat, from_vec_5);

    from_mat.col(0) = from_mat.col(0)*from_coefs(0);
    from_mat.col(1) = from_mat.col(1)*from_coefs(1);
    from_mat.col(2) = from_mat.col(2)*from_coefs(2);
    from_mat.col(3) = from_mat.col(3)*from_coefs(3);

    arma::Mat<imp_float> to_mat = {{to_pt_1.x, to_pt_2.x, to_pt_3.x, to_pt_4.x},
								   {to_pt_1.y, to_pt_2.y, to_pt_3.y, to_pt_4.y},
								   {to_pt_1.z, to_pt_2.z, to_pt_3.z, to_pt_4.z},
								   {    1,         1,         1,         1    }};

    arma::Col<imp_float> to_vec_5 = {to_pt_5.x, to_pt_5.y, to_pt_5.z, 1};

    const arma::Col<imp_float>& to_coefs = arma::solve(to_mat, to_vec_5);

    to_mat.col(0) = to_mat.col(0)*to_coefs(0);
    to_mat.col(1) = to_mat.col(1)*to_coefs(1);
    to_mat.col(2) = to_mat.col(2)*to_coefs(2);
    to_mat.col(3) = to_mat.col(3)*to_coefs(3);

    return ProjectiveTransformation(to_mat*(from_mat.i()));
}

ProjectiveTransformation ProjectiveTransformation::unhinging(imp_float near_plane_distance,
                                                             imp_float far_plane_distance)
{
    imp_float difference = far_plane_distance - near_plane_distance;

    arma::Mat<imp_float> matrix(4, 4, arma::fill::zeros);
    matrix(0, 0) = difference;
    matrix(1, 1) = difference;
    matrix(2, 2) = far_plane_distance;
    matrix(2, 3) = near_plane_distance;
    matrix(3, 2) = -difference;

    return ProjectiveTransformation(matrix);
}

ProjectiveTransformation ProjectiveTransformation::operator()(const LinearTransformation& other) const
{
    return ProjectiveTransformation(_matrix*other.getMatrix().toArma4x4Matrix());
}

ProjectiveTransformation ProjectiveTransformation::operator()(const AffineTransformation& other) const
{
    return ProjectiveTransformation(_matrix*other.getMatrix().toArma4x4Matrix());
}

ProjectiveTransformation ProjectiveTransformation::operator()(const ProjectiveTransformation& other) const
{
    return ProjectiveTransformation(_matrix*other._matrix);
}

Point ProjectiveTransformation::operator()(const Point& point) const
{
    imp_float w = _matrix(3, 0)*point.x + _matrix(3, 1)*point.y + _matrix(3, 2)*point.z + _matrix(3, 3);
    assert(w != 0);
    imp_float w_inv = 1/w;

    return Point((_matrix(0, 0)*point.x + _matrix(0, 1)*point.y + _matrix(0, 2)*point.z + _matrix(0, 3))*w_inv,
                 (_matrix(1, 0)*point.x + _matrix(1, 1)*point.y + _matrix(1, 2)*point.z + _matrix(1, 3))*w_inv,
                 (_matrix(2, 0)*point.x + _matrix(2, 1)*point.y + _matrix(2, 2)*point.z + _matrix(2, 3))*w_inv);
}

Triangle ProjectiveTransformation::operator()(const Triangle& triangle) const
{
    return Triangle((*this)(triangle.getPointA()),
                    (*this)(triangle.getPointB()),
                    (*this)(triangle.getPointC()));
}

ProjectiveTransformation& ProjectiveTransformation::invert()
{
    _matrix = _matrix.i();
	return *this;
}

ProjectiveTransformation ProjectiveTransformation::getInverse() const
{
    return ProjectiveTransformation(_matrix.i());
}

const arma::Mat<imp_float>& ProjectiveTransformation::getMatrix() const
{
    return _matrix;
}

} // Geometry3D
} // Impact
