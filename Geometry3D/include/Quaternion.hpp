#pragma once
#include "precision.hpp"
#include "Vector3.hpp"

namespace Impact {
namespace Geometry3D {

class Quaternion {

public:
    imp_float w, x, y, z;

	Quaternion(imp_float new_w,
			   imp_float new_x,
			   imp_float new_y,
			   imp_float new_z);

    Quaternion(Vector axis, imp_float angle);
	
    Quaternion  operator+ (const Quaternion& other) const;
    Quaternion  operator- (const Quaternion& other) const;
    Quaternion  operator* (const Quaternion& other) const;
    Quaternion  operator* (imp_float factor) const;
    Quaternion  operator/ (imp_float divisor) const;

    Quaternion& operator+=(const Quaternion& other);
    Quaternion& operator-=(const Quaternion& other);
    Quaternion& operator*=(const Quaternion& other);
    Quaternion& operator*=(imp_float factor);
    Quaternion& operator/=(imp_float divisor);

	Quaternion& rotate(const Vector& rotation_vector);
	Quaternion& spin(const Vector& angular_velocity, imp_float duration);

	Quaternion& normalize();
};

Quaternion operator*(imp_float factor, const Quaternion& quaternion);

} // Geometry3D
} // Impact
