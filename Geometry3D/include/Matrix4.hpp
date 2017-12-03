#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Vector4.hpp"
#include "Matrix3.hpp"
#include <string>

namespace Impact {
namespace Geometry3D {

class Matrix4 {

public:
	imp_float a11, a12, a13, a14,
			  a21, a22, a23, a24,
			  a31, a32, a33, a34;
	//		    0,   0,   0,   1

    Matrix4();

    Matrix4(imp_float new_a11, imp_float new_a12, imp_float new_a13, imp_float new_a14,
			imp_float new_a21, imp_float new_a22, imp_float new_a23, imp_float new_a24,
			imp_float new_a31, imp_float new_a32, imp_float new_a33, imp_float new_a34);

    Matrix4(const Vector& column_1,
			const Vector& column_2,
		    const Vector& column_3,
		    const Vector& column_4);

	Matrix4(const Matrix3& other);

    static Matrix4 identity();

	Matrix4& setToIdentity();

    Matrix4  operator* (const Matrix4& other)  const;
    Vector   operator* (const Vector& vector)  const;
    Point    operator* (const Point& vector)   const;
    Vector4  operator* (const Vector4& vector) const;
	Matrix4& operator*=(const Matrix4& other);

    Matrix4& swap(Matrix4& other);

    Matrix4& setColumns(const Vector& column_1,
						const Vector& column_2,
						const Vector& column_3,
						const Vector& column_4);
    Matrix4& setRows(const Vector4& row_1,
					 const Vector4& row_2,
					 const Vector4& row_3);

	Matrix3 getLinearPart() const;

    imp_float getDeterminant() const;

    Matrix4& invert();
	Matrix4 getInverse() const;
	
	arma::Mat<imp_float> toArma4x4Matrix() const;

    std::string toString() const;
};

} // Geometry3D
} // Impact
