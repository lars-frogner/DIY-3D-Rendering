#include "Matrix4.hpp"
#include <cassert>
#include <sstream>
#include <iomanip>

namespace Impact {
namespace Geometry3D {

Matrix4::Matrix4()
	: a11(1),
	  a12(0),
	  a13(0),
	  a14(0),
	  a21(0),
	  a22(1),
	  a23(0),
	  a24(0),
	  a31(0),
	  a32(0),
	  a33(1),
	  a34(0) {}

Matrix4::Matrix4(imp_float new_a11, imp_float new_a12, imp_float new_a13, imp_float new_a14,
				 imp_float new_a21, imp_float new_a22, imp_float new_a23, imp_float new_a24,
				 imp_float new_a31, imp_float new_a32, imp_float new_a33, imp_float new_a34)
	: a11(new_a11),
	  a12(new_a12),
	  a13(new_a13),
	  a14(new_a14),
	  a21(new_a21),
	  a22(new_a22),
	  a23(new_a23),
	  a24(new_a24),
	  a31(new_a31),
	  a32(new_a32),
	  a33(new_a33),
	  a34(new_a34) {}

Matrix4::Matrix4(const Vector& column_1,
				 const Vector& column_2,
			     const Vector& column_3,
			     const Vector& column_4)
	: a11(column_1.x),
	  a12(column_2.x),
	  a13(column_3.x),
	  a14(column_4.x),
	  a21(column_1.y),
	  a22(column_2.y),
	  a23(column_3.y),
	  a24(column_4.y),
	  a31(column_1.z),
	  a32(column_2.z),
	  a33(column_3.z),
	  a34(column_4.z) {}

Matrix4::Matrix4(const Matrix3& other)
	: a11(other.a11),
	  a12(other.a12),
	  a13(other.a13),
	  a14(0),
	  a21(other.a21),
	  a22(other.a22),
	  a23(other.a23),
	  a24(0),
	  a31(other.a31),
	  a32(other.a32),
	  a33(other.a33),
	  a34(0) {}

Matrix4 Matrix4::identity()
{
    return Matrix4();
}

Matrix4& Matrix4::setToIdentity()
{
	a11 = 1;
	a12 = 0;
	a13 = 0;
	a14 = 0;
	a21 = 0;
	a22 = 1;
	a23 = 0;
	a24 = 0;
	a31 = 0;
	a32 = 0;
	a33 = 1;
	a34 = 0;

	return *this;
}

Matrix4 Matrix4::operator*(const Matrix4& other) const
{
	return Matrix4(other.a11*a11 + other.a21*a12 + other.a31*a13,
				   other.a12*a11 + other.a22*a12 + other.a32*a13,
				   other.a13*a11 + other.a23*a12 + other.a33*a13,
				   other.a14*a11 + other.a24*a12 + other.a34*a13 + a14,
				   other.a11*a21 + other.a21*a22 + other.a31*a23,
				   other.a12*a21 + other.a22*a22 + other.a32*a23,
				   other.a13*a21 + other.a23*a22 + other.a33*a23,
				   other.a14*a21 + other.a24*a22 + other.a34*a23 + a24,
				   other.a11*a31 + other.a21*a32 + other.a31*a33,
				   other.a12*a31 + other.a22*a32 + other.a32*a33,
				   other.a13*a31 + other.a23*a32 + other.a33*a33,
				   other.a14*a31 + other.a24*a32 + other.a34*a33 + a34);
}

Vector Matrix4::operator*(const Vector& vector) const
{
	return Vector(vector.x*a11 + vector.y*a12 + vector.z*a13,
				  vector.x*a21 + vector.y*a22 + vector.z*a23,
				  vector.x*a31 + vector.y*a32 + vector.z*a33);
}

Point Matrix4::operator*(const Point& point) const
{
	return Point(point.x*a11 + point.y*a12 + point.z*a13 + a14,
				 point.x*a21 + point.y*a22 + point.z*a23 + a24,
				 point.x*a31 + point.y*a32 + point.z*a33 + a34);
}

Vector4 Matrix4::operator*(const Vector4& vector) const
{
	return Vector4(vector.x*a11 + vector.y*a12 + vector.z*a13 + vector.w*a14,
				   vector.x*a21 + vector.y*a22 + vector.z*a23 + vector.w*a24,
				   vector.x*a31 + vector.y*a32 + vector.z*a33 + vector.w*a34,
				   vector.w);
}

Matrix4& Matrix4::operator*=(const Matrix4& other)
{
	*this = (*this)*other;
    return *this;
}

Matrix4& Matrix4::swap(Matrix4& other)
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

    temp = other.a14;
    other.a14 = a14;
    a14 = temp;

    temp = other.a21;
    other.a21 = a21;
    a21 = temp;

    temp = other.a22;
    other.a22 = a22;
    a22 = temp;

    temp = other.a23;
    other.a23 = a23;
    a23 = temp;

    temp = other.a24;
    other.a24 = a24;
    a24 = temp;

    temp = other.a31;
    other.a31 = a31;
    a31 = temp;

    temp = other.a32;
    other.a32 = a32;
    a32 = temp;

    temp = other.a33;
    other.a33 = a33;
    a33 = temp;

    temp = other.a34;
    other.a34 = a34;
    a34 = temp;

    return *this;
}

Matrix4& Matrix4::setColumns(const Vector& column_1,
							 const Vector& column_2,
							 const Vector& column_3,
							 const Vector& column_4)
{
    a11 = column_1.x;
    a12 = column_2.x;
    a13 = column_3.x;
    a14 = column_4.x;
    a21 = column_1.y;
    a22 = column_2.y;
    a23 = column_3.y;
    a24 = column_4.y;
    a31 = column_1.z;
    a32 = column_2.z;
    a33 = column_3.z;
    a34 = column_4.z;

    return *this;
}

Matrix4& Matrix4::setRows(const Vector4& row_1,
						  const Vector4& row_2,
						  const Vector4& row_3)
{
    a11 = row_1.x;
    a12 = row_1.y;
    a13 = row_1.z;
    a14 = row_1.w;
    a21 = row_2.x;
    a22 = row_2.y;
    a23 = row_2.z;
    a24 = row_2.w;
    a31 = row_3.x;
    a32 = row_3.y;
    a33 = row_3.z;
    a34 = row_3.w;

    return *this;
}

Matrix3 Matrix4::getLinearPart() const
{
	return Matrix3(a11, a12, a13,
				   a21, a22, a23,
				   a31, a32, a33);
}

imp_float Matrix4::getDeterminant() const
{
	return a11*(a22*a33 - a23*a32) +
		   a12*(a23*a31 - a21*a33) +
		   a13*(a21*a32 - a22*a31);
}

Matrix4& Matrix4::invert()
{
	*this = getInverse();
    return *this;
}

Matrix4 Matrix4::getInverse() const
{
	imp_float inverse_determinant = getDeterminant();

	assert(inverse_determinant != 0);

	inverse_determinant = 1/inverse_determinant;

	return Matrix4((a22*a33 - a23*a32)*inverse_determinant,
				   (a13*a32 - a12*a33)*inverse_determinant,
				   (a12*a23 - a13*a22)*inverse_determinant,
				   (a14*(a23*a32 - a22*a33) +
				    a13*(a22*a34 - a24*a32) +
				    a12*(a24*a33 - a23*a34))*inverse_determinant,
				   (a23*a31 - a21*a33)*inverse_determinant,
				   (a11*a33 - a13*a31)*inverse_determinant,
				   (a13*a21 - a11*a23)*inverse_determinant,
				   (a14*(a21*a33 - a23*a31) +
				    a13*(a24*a31 - a21*a34) +
				    a11*(a23*a34 - a24*a33))*inverse_determinant,
				   (a21*a32 - a22*a31)*inverse_determinant,
				   (a12*a31 - a11*a32)*inverse_determinant,
				   (a11*a22 - a12*a21)*inverse_determinant,
				   (a14*(a22*a31 - a21*a32) +
				    a12*(a21*a34 - a24*a31) +
				    a11*(a24*a32 - a22*a34))*inverse_determinant);
}

arma::Mat<imp_float> Matrix4::toArma4x4Matrix() const
{
	return arma::Mat<imp_float>({{a11, a12, a13, a14},
								 {a21, a22, a23, a24},
								 {a31, a32, a33, a34},
								 {  0,   0,	  0,   1}});
}

std::string Matrix4::toString() const
{
    std::ostringstream string_stream;
    string_stream << "|" << std::setw(7) << std::setprecision(4) << a11 << " " << std::setw(7) << std::setprecision(4) << a12 << " " << std::setw(7) << std::setprecision(4) << a13 << " " << std::setw(7) << std::setprecision(4) << a14 << "|\n"
				  << "|" << std::setw(7) << std::setprecision(4) << a21 << " " << std::setw(7) << std::setprecision(4) << a22 << " " << std::setw(7) << std::setprecision(4) << a23 << " " << std::setw(7) << std::setprecision(4) << a24 << "|\n"
				  << "|" << std::setw(7) << std::setprecision(4) << a31 << " " << std::setw(7) << std::setprecision(4) << a32 << " " << std::setw(7) << std::setprecision(4) << a33 << " " << std::setw(7) << std::setprecision(4) << a34 << "|\n"
				  << "|" << std::setw(7) << std::setprecision(4) <<   0 << " " << std::setw(7) << std::setprecision(4) <<   0 << " " << std::setw(7) << std::setprecision(4) <<   0 << " " << std::setw(7) << std::setprecision(4) <<   1 << "|";
    return string_stream.str();
}

} // Geometry3D
} // Impact
