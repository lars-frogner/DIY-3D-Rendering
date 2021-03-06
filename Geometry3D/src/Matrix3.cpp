#include "Matrix3.hpp"
#include <cassert>
#include <sstream>
#include <iomanip>

namespace Impact {
namespace Geometry3D {

Matrix3::Matrix3()
	: a11(1),
	  a12(0),
	  a13(0),
	  a21(0),
	  a22(1),
	  a23(0),
	  a31(0),
	  a32(0),
	  a33(1) {}

Matrix3::Matrix3(imp_float new_a11, imp_float new_a12, imp_float new_a13,
				 imp_float new_a21, imp_float new_a22, imp_float new_a23,
				 imp_float new_a31, imp_float new_a32, imp_float new_a33)
	: a11(new_a11),
	  a12(new_a12),
	  a13(new_a13),
	  a21(new_a21),
	  a22(new_a22),
	  a23(new_a23),
	  a31(new_a31),
	  a32(new_a32),
	  a33(new_a33) {}

Matrix3::Matrix3(const Vector& column_1,
				 const Vector& column_2,
			     const Vector& column_3)
	: a11(column_1.x),
	  a12(column_2.x),
	  a13(column_3.x),
	  a21(column_1.y),
	  a22(column_2.y),
	  a23(column_3.y),
	  a31(column_1.z),
	  a32(column_2.z),
	  a33(column_3.z) {}

Matrix3 Matrix3::identity()
{
    return Matrix3();
}

Matrix3& Matrix3::setToIdentity()
{
	a11 = 1;
	a12 = 0;
	a13 = 0;
	a21 = 0;
	a22 = 1;
	a23 = 0;
	a31 = 0;
	a32 = 0;
	a33 = 1;

	return *this;
}

Matrix3 Matrix3::operator+(const Matrix3& other) const
{
    return Matrix3(a11 + other.a11,
				   a12 + other.a12,
				   a13 + other.a13,
				   a21 + other.a21,
				   a22 + other.a22,
				   a23 + other.a23,
				   a31 + other.a31,
				   a32 + other.a32,
				   a33 + other.a33);
}

Matrix3 Matrix3::operator-() const
{
    return Matrix3(-a11,
				   -a12,
				   -a13,
				   -a21,
				   -a22,
				   -a23,
				   -a31,
				   -a32,
				   -a33);
}

Matrix3 Matrix3::operator-(const Matrix3& other) const
{
    return Matrix3(a11 - other.a11,
				   a12 - other.a12,
				   a13 - other.a13,
				   a21 - other.a21,
				   a22 - other.a22,
				   a23 - other.a23,
				   a31 - other.a31,
				   a32 - other.a32,
				   a33 - other.a33);
}

Matrix3& Matrix3::operator+=(const Matrix3& other)
{
	a11 += other.a11;
	a12 += other.a12;
	a13 += other.a13;
	a21 += other.a21;
	a22 += other.a22;
	a23 += other.a23;
	a31 += other.a31;
	a32 += other.a32;
	a33 += other.a33;

    return *this;
}

Matrix3& Matrix3::operator-=(const Matrix3& other)
{
	a11 -= other.a11;
	a12 -= other.a12;
	a13 -= other.a13;
	a21 -= other.a21;
	a22 -= other.a22;
	a23 -= other.a23;
	a31 -= other.a31;
	a32 -= other.a32;
	a33 -= other.a33;

    return *this;
}

Matrix3 Matrix3::operator*(const Matrix3& other) const
{
	return Matrix3(other.a11*a11 + other.a21*a12 + other.a31*a13,
				   other.a12*a11 + other.a22*a12 + other.a32*a13,
				   other.a13*a11 + other.a23*a12 + other.a33*a13,
				   other.a11*a21 + other.a21*a22 + other.a31*a23,
				   other.a12*a21 + other.a22*a22 + other.a32*a23,
				   other.a13*a21 + other.a23*a22 + other.a33*a23,
				   other.a11*a31 + other.a21*a32 + other.a31*a33,
				   other.a12*a31 + other.a22*a32 + other.a32*a33,
				   other.a13*a31 + other.a23*a32 + other.a33*a33);
}

Vector Matrix3::operator*(const Vector& vector) const
{
	return Vector(vector.x*a11 + vector.y*a12 + vector.z*a13,
				  vector.x*a21 + vector.y*a22 + vector.z*a23,
				  vector.x*a31 + vector.y*a32 + vector.z*a33);
}

Point Matrix3::operator*(const Point& point) const
{
	return Point(point.x*a11 + point.y*a12 + point.z*a13,
				 point.x*a21 + point.y*a22 + point.z*a23,
				 point.x*a31 + point.y*a32 + point.z*a33);
}

Matrix3 Matrix3::operator*(imp_float factor) const
{
    return Matrix3(a11*factor,
				   a12*factor,
				   a13*factor,
				   a21*factor,
				   a22*factor,
				   a23*factor,
				   a31*factor,
				   a32*factor,
				   a33*factor);
}

Matrix3 Matrix3::operator/(imp_float divisor) const
{
	assert(divisor != 0);

    imp_float factor = 1/divisor;

    return Matrix3(a11*factor,
				   a12*factor,
				   a13*factor,
				   a21*factor,
				   a22*factor,
				   a23*factor,
				   a31*factor,
				   a32*factor,
				   a33*factor);
}

Matrix3& Matrix3::operator*=(const Matrix3& other)
{
	*this = (*this)*other;
    return *this;
}

Matrix3& Matrix3::operator*=(imp_float factor)
{
	a11 *= factor;
	a12 *= factor;
	a13 *= factor;
	a21 *= factor;
	a22 *= factor;
	a23 *= factor;
	a31 *= factor;
	a32 *= factor;
	a33 *= factor;

    return *this;
}

Matrix3& Matrix3::operator/=(imp_float divisor)
{
	assert(divisor != 0);

    imp_float factor = 1/divisor;

	a11 *= factor;
	a12 *= factor;
	a13 *= factor;
	a21 *= factor;
	a22 *= factor;
	a23 *= factor;
	a31 *= factor;
	a32 *= factor;
	a33 *= factor;

    return *this;
}

Matrix3& Matrix3::swap(Matrix3& other)
{
    imp_float temp = other.a11;
    other.a11 = a11;
    a11 = temp;

    temp = other.a12;
    other.a12 = a12;
    a12 = temp;

    temp = other.a13;
    other.a13 = a13;
    a13 = temp;

    temp = other.a21;
    other.a21 = a21;
    a21 = temp;

    temp = other.a22;
    other.a22 = a22;
    a22 = temp;

    temp = other.a23;
    other.a23 = a23;
    a23 = temp;

    temp = other.a31;
    other.a31 = a31;
    a31 = temp;

    temp = other.a32;
    other.a32 = a32;
    a32 = temp;

    temp = other.a33;
    other.a33 = a33;
    a33 = temp;

    return *this;
}

Matrix3& Matrix3::setColumns(const Vector& column_1,
							 const Vector& column_2,
							 const Vector& column_3)
{
    a11 = column_1.x;
    a12 = column_2.x;
    a13 = column_3.x;
    a21 = column_1.y;
    a22 = column_2.y;
    a23 = column_3.y;
    a31 = column_1.z;
    a32 = column_2.z;
    a33 = column_3.z;

    return *this;
}

Matrix3& Matrix3::setRows(const Vector& row_1,
						  const Vector& row_2,
						  const Vector& row_3)
{
    a11 = row_1.x;
    a12 = row_1.y;
    a13 = row_1.z;
    a21 = row_2.x;
    a22 = row_2.y;
    a23 = row_2.z;
    a31 = row_3.x;
    a32 = row_3.y;
    a33 = row_3.z;

    return *this;
}

imp_float Matrix3::getDeterminant() const
{
	return a11*(a22*a33 - a23*a32) +
		   a12*(a23*a31 - a21*a33) +
		   a13*(a21*a32 - a22*a31);
}

Matrix3& Matrix3::transpose()
{
    imp_float temp = a12;
    a12 = a21;
    a21 = temp;

	temp = a13;
    a13 = a31;
    a31 = temp;

	temp = a23;
    a23 = a32;
    a32 = temp;

	return *this;
}

Matrix3& Matrix3::invert()
{
	*this = getInverse();
    return *this;
}

Matrix3 Matrix3::getTranspose() const
{
	return Matrix3(a11,
				   a21,
				   a31,
				   a12,
				   a22,
				   a32,
				   a13,
				   a23,
				   a33);
}

Matrix3 Matrix3::getInverse() const
{
	imp_float diff1 = a22*a33 - a23*a32;
	imp_float diff2 = a23*a31 - a21*a33;
	imp_float diff3 = a21*a32 - a22*a31;

	// (Not inverted yet)
	imp_float inverse_determinant = a11*diff1 +
									a12*diff2 +
								    a13*diff3;

	assert(inverse_determinant != 0);

	inverse_determinant = 1/inverse_determinant;

	return Matrix3(diff1*inverse_determinant,
				   (a13*a32 - a12*a33)*inverse_determinant,
				   (a12*a23 - a13*a22)*inverse_determinant,
				   diff2*inverse_determinant,
				   (a11*a33 - a13*a31)*inverse_determinant,
				   (a13*a21 - a11*a23)*inverse_determinant,
				   diff3*inverse_determinant,
				   (a12*a31 - a11*a32)*inverse_determinant,
				   (a11*a22 - a12*a21)*inverse_determinant);
}

arma::Mat<imp_float> Matrix3::toArma3x3Matrix() const
{
	return arma::Mat<imp_float>({{a11, a12, a13},
								 {a21, a22, a23},
								 {a31, a32, a33}});
}

arma::Mat<imp_float> Matrix3::toArma4x4Matrix() const
{
	return arma::Mat<imp_float>({{a11, a12, a13, 0},
								 {a21, a22, a23, 0},
								 {a31, a32, a33, 0},
								 {  0,   0,	  0, 1}});
}

std::string Matrix3::toString() const
{
    std::ostringstream string_stream;
    string_stream << "|" << std::setw(7) << std::setprecision(4) << a11 << " " << std::setw(7) << std::setprecision(4) << a12 << " " << std::setw(7) << std::setprecision(4) << a13 << "|\n"
				  << "|" << std::setw(7) << std::setprecision(4) << a21 << " " << std::setw(7) << std::setprecision(4) << a22 << " " << std::setw(7) << std::setprecision(4) << a23 << "|\n"
				  << "|" << std::setw(7) << std::setprecision(4) << a31 << " " << std::setw(7) << std::setprecision(4) << a32 << " " << std::setw(7) << std::setprecision(4) << a33 << "|";
    return string_stream.str();
}

Matrix3 operator*(imp_float factor, const Matrix3& matrix)
{
    return matrix*factor;
}

} // Geometry3D
} // Impact
