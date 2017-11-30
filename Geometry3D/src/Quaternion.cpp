#include "Quaternion.hpp"
#include <cmath>

namespace Impact {
namespace Geometry3D {

Quaternion::Quaternion(imp_float new_w,
					   imp_float new_x,
					   imp_float new_y,
					   imp_float new_z)
	: w(new_w), x(new_x), y(new_y), z(new_z) {}

Quaternion::Quaternion(Vector axis, imp_float angle)
{
	imp_float half_angle = 0.5f*angle;
	imp_float sin_half_angle = sin(half_angle);

	axis.normalize();

	w = cos(half_angle);
	x = axis.x*sin_half_angle;
	y = axis.y*sin_half_angle;
	z = axis.z*sin_half_angle;
}

Quaternion Quaternion::operator+(const Quaternion& other) const
{
	return Quaternion(w + other.w,
					  x + other.x,
					  y + other.y,
					  z + other.z);
}

Quaternion Quaternion::operator-(const Quaternion& other) const
{
	return Quaternion(w - other.w,
					  x - other.x,
					  y - other.y,
					  z - other.z);
}

Quaternion Quaternion::operator*(const Quaternion& other) const
{
	imp_float new_w, new_x, new_y, new_z;

	new_w = w*other.w - x*other.x - y*other.y - z*other.z;
	new_x = w*other.x + x*other.w - y*other.z - z*other.y;
	new_y = w*other.y - x*other.z + y*other.w - z*other.x;
	new_z = w*other.z + x*other.y - y*other.x + z*other.w;

	return Quaternion(new_w, new_x, new_y, new_z);
}

Quaternion Quaternion::operator*(imp_float factor) const
{
	return Quaternion(w*factor,
					  x*factor,
					  y*factor,
					  z*factor);
}

Quaternion Quaternion::operator/(imp_float divisor) const
{
	assert(divisor != 0);
	
	imp_float factor = 1/divisor;

	return Quaternion(w*factor,
					  x*factor,
					  y*factor,
					  z*factor);
}

Quaternion& Quaternion::operator+=(const Quaternion& other)
{
	w += other.w;
	x += other.x;
	y += other.y;
	z += other.z;

	return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion& other)
{
	w -= other.w;
	x -= other.x;
	y -= other.y;
	z -= other.z;

	return *this;
}

Quaternion& Quaternion::operator*=(const Quaternion& other)
{
	imp_float new_w, new_x, new_y, new_z;

	new_w = w*other.w - x*other.x - y*other.y - z*other.z;
	new_x = w*other.x + x*other.w - y*other.z - z*other.y;
	new_y = w*other.y - x*other.z + y*other.w - z*other.x;
	new_z = w*other.z + x*other.y - y*other.x + z*other.w;

	w = new_w; x = new_x; y = new_y; z = new_z;

	return *this;
}

Quaternion& Quaternion::operator*=(imp_float factor)
{
	w *= factor;
	x *= factor;
	y *= factor;
	z *= factor;

	return *this;
}

Quaternion& Quaternion::operator/=(imp_float divisor)
{
	assert(divisor != 0);

	imp_float factor = 1/divisor;

	w *= factor;
	x *= factor;
	y *= factor;
	z *= factor;

	return *this;
}

Quaternion& Quaternion::rotate(const Vector& rotation_vector)
{
	return *this += 0.5f*(Quaternion(0, rotation_vector.x, rotation_vector.y, rotation_vector.z)*(*this));
}

Quaternion& Quaternion::spin(const Vector& angular_velocity, imp_float duration)
{
	return rotate(angular_velocity*duration);
}

Quaternion& Quaternion::normalize()
{
	imp_float length = sqrt(w*w + x*x + y*y + z*z);

	assert(length > 0);

	w /= length; x /= length; y /= length; z /= length;

	return *this;
}

Quaternion operator*(imp_float factor, const Quaternion& quaternion)
{
	return quaternion*factor;
}

} // Geometry3D
} // Impact
