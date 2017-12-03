#pragma once
#include "precision.hpp"
#include "Vector3.hpp"
#include <string>
#include <armadillo>

namespace Impact {
namespace Geometry3D {

class Matrix3 {

public:
	imp_float a11, a12, a13,
			  a21, a22, a23,
			  a31, a32, a33;

    Matrix3();

    Matrix3(imp_float new_a11, imp_float new_a12, imp_float new_a13,
			imp_float new_a21, imp_float new_a22, imp_float new_a23,
			imp_float new_a31, imp_float new_a32, imp_float new_a33);

    Matrix3(const Vector& column_1,
			const Vector& column_2,
		    const Vector& column_3);

    static Matrix3 identity();

	Matrix3& setToIdentity();

    Matrix3  operator+ (const Matrix3& other)  const;
    Matrix3  operator- ()					   const;
    Matrix3  operator- (const Matrix3& other)  const;
    Matrix3& operator+=(const Matrix3& other);
    Matrix3& operator-=(const Matrix3& other);
    Matrix3  operator* (const Matrix3& other)  const;
    Vector   operator* (const Vector& vector)  const;
    Point    operator* (const Point& vector)   const;
    Matrix3  operator* (imp_float factor)      const;
    Matrix3  operator/ (imp_float divisor)	   const;
	Matrix3& operator*=(const Matrix3& other);
    Matrix3& operator*=(imp_float factor);
    Matrix3& operator/=(imp_float divisor);

    Matrix3& swap(Matrix3& other);

    Matrix3& setColumns(const Vector& column_1,
						const Vector& column_2,
						const Vector& column_3);
    Matrix3& setRows(const Vector& row_1,
					 const Vector& row_2,
					 const Vector& row_3);

    imp_float getDeterminant() const;
	
    Matrix3& transpose();

    Matrix3& invert();

	Matrix3 getTranspose() const;
	Matrix3 getInverse() const;

	arma::Mat<imp_float> toArma3x3Matrix() const;
	arma::Mat<imp_float> toArma4x4Matrix() const;

    std::string toString() const;
};

Matrix3 operator*(imp_float factor, const Matrix3& matrix);

} // Geometry3D
} // Impact
