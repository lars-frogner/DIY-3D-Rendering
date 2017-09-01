#pragma once
#include <math.h>
#include <assert.h>
#include <limits>
#include <string>
#include <sstream>
#include "Point.hpp"

namespace Geometry3D {

template <typename F>
class Vector {

public:
    F x, y, z;

    Vector<F>();
    Vector<F>(F x_new, F y_new, F z_new);

    static Vector<F> zero();
    static Vector<F> unitX();
    static Vector<F> unitY();
    static Vector<F> unitZ();

    Vector<F>  operator+ (const Vector<F>& other) const;
    Point<F>   operator+ (const Point<F>&  point) const;
    Vector<F>  operator- ()						  const;
    Vector<F>  operator- (const Vector<F>& other) const;
    Vector<F>& operator+=(const Vector<F>& other);
    Vector<F>& operator-=(const Vector<F>& other);
    Vector<F>  operator* (F factor)				  const;
    Vector<F>  operator/ (F divisor)			  const;
    Vector<F>& operator*=(F factor);
    Vector<F>& operator/=(F divisor);
    
    Vector4<F> operator+ (const Vector4<F>& vector4) const;
    Vector4<F> operator- (const Vector4<F>& vector4) const;

    F		  dot  (const Vector<F>& other) const;
    Vector<F> cross(const Vector<F>& other) const;

    Vector<F>& swap(Vector<F>& other);

    Vector<F>& setComponents(F x_new, F y_new, F z_new);
    Vector<F>& shiftComponents(F dx, F dy, F dz);

    Vector<F>& rotateFromXToY(F angle);
    Vector<F>& rotateFromYToZ(F angle);
    Vector<F>& rotateFromZToX(F angle);
    Vector<F> getRotatedFromXToY(F angle) const;
    Vector<F> getRotatedFromYToZ(F angle) const;
    Vector<F> getRotatedFromZToX(F angle) const;

    F getSmallestComponent() const;
    size_t getSmallestComponentIndex() const;
    F getLargestComponent() const;
    size_t getLargestComponentIndex() const;
    F getSmallestComponentNaNSafe() const;
    F getLargestComponentNaNSafe() const;

    static void sortComponentwise(Vector<F>& min_vector,
                                  Vector<F>& max_vector);

    F getLength() const;
    F getSquaredLength() const;
    Vector<F>& normalize();
    Vector<F> getNormalized() const;

    Vector<F> getUnitNormalWith	    (const Vector<F>& other)    const;
    Vector<F> getProjectedOn	    (const Vector<F>& other)    const;
    Vector<F> getProjectedOnNormalTo(const Vector<F>& vector_1,
                                     const Vector<F>& vector_2) const;

    Point<F> toPoint() const;
    Vector4<F> toVector4() const;
    std::string toString() const;
};

template <typename F>
Vector<F> operator*(F factor, const Vector<F>& vector);

template <typename F>
Vector<F> operator/(F factor, const Vector<F>& vector);

template <typename F>
Vector<F>::Vector()
    : x(0), y(0), z(0) {}

template <typename F>
Vector<F>::Vector(F x_new, F y_new, F z_new)
    : x(x_new), y(y_new), z(z_new) {}

template <typename F>
Vector<F> Vector<F>::zero()
{
    return Vector<F>(0, 0, 0);
}

template <typename F>
Vector<F> Vector<F>::unitX()
{
    return Vector<F>(1, 0, 0);
}

template <typename F>
Vector<F> Vector<F>::unitY()
{
    return Vector<F>(0, 1, 0);
}

template <typename F>
Vector<F> Vector<F>::unitZ()
{
    return Vector<F>(0, 0, 1);
}

template <typename F>
Vector<F> Vector<F>::operator+(const Vector<F>& other) const
{
    return Vector<F>(x + other.x, y + other.y, z + other.z);
}

template <typename F>
Point<F> Vector<F>::operator+(const Point<F>& point) const
{
    return Point<F>(x + point.x, y + point.y, z + point.z);
}

template <typename F>
Vector<F> Vector<F>::operator-() const
{
    return Vector<F>(-x, -y, -z);
}

template <typename F>
Vector<F> Vector<F>::operator-(const Vector<F>& other) const
{
    return *this + (-other);
}

template <typename F>
Vector<F>& Vector<F>::operator+=(const Vector<F>& other)
{
    return shiftComponents(other.x, other.y, other.z);
}

template <typename F>
Vector<F>& Vector<F>::operator-=(const Vector<F>& other)
{
    return *this += (-other);
}

template <typename F>
Vector<F> Vector<F>::operator*(F factor) const
{
    return Vector<F>(x*factor, y*factor, z*factor);
}

template <typename F>
Vector<F> Vector<F>::operator/(F divisor) const
{
    return (*this)*(1/divisor);
}

template <typename F>
Vector<F>& Vector<F>::operator*=(F factor)
{
    return setComponents(x*factor, y*factor, z*factor);
}

template <typename F>
Vector<F>& Vector<F>::operator/=(F divisor)
{
    return *this *= (1/divisor);
}

template <typename F>
Vector4<F> Vector<F>::operator+(const Vector4<F>& vector4) const
{
    return Vector4<F>(x + vector4.x, y + vector4.y, z + vector4.z, vector4.w);
}

template <typename F>
Vector4<F> Vector<F>::operator-(const Vector4<F>& vector4) const
{
    return Vector4<F>(x - vector4.x, y - vector4.y, z - vector4.z, -vector4.w);
}

template <typename F>
F Vector<F>::dot(const Vector<F>& other) const
{
    return x*other.x + y*other.y + z*other.z;
}

template <typename F>
Vector<F> Vector<F>::cross(const Vector<F>& other) const
{
    return Vector<F>(y*other.z - z*other.y,
                     z*other.x - x*other.z,
                     x*other.y - y*other.x);
}

template <typename F>
Vector<F>& Vector<F>::swap(Vector<F>& other)
{
    F temp = other.x;
    other.x = x;
    x = temp;

    temp = other.y;
    other.y = y;
    y = temp;

    temp = other.z;
    other.z = z;
    z = temp;

    return *this;
}

template <typename F>
Vector<F>& Vector<F>::setComponents(F x_new, F y_new, F z_new)
{
    x = x_new; y = y_new; z = z_new;
    return *this;
}

template <typename F>
Vector<F>& Vector<F>::shiftComponents(F dx, F dy, F dz)
{
    return setComponents(x + dx, y + dy, z + dz);
}

template <typename F>
Vector<F>& Vector<F>::rotateFromXToY(F angle)
{
    F cos_angle = cos(angle);
    F sin_angle = sin(angle);
    F new_x = x*cos_angle - y*sin_angle;
    y = x*sin_angle + y*cos_angle;
    x = new_x;
    return *this;
}

template <typename F>
Vector<F>& Vector<F>::rotateFromYToZ(F angle)
{
    F cos_angle = cos(angle);
    F sin_angle = sin(angle);
    F new_y = y*cos_angle - z*sin_angle;
    z = y*sin_angle + z*cos_angle;
    y = new_y;
    return *this;
}

template <typename F>
Vector<F>& Vector<F>::rotateFromZToX(F angle)
{
    F cos_angle = cos(angle);
    F sin_angle = sin(angle);
    F new_z = -x*sin_angle + z*cos_angle;
    x = x*cos_angle + z*sin_angle;
    z = new_z;
    return *this;
}

template <typename F>
Vector<F> Vector<F>::getRotatedFromXToY(F angle) const
{
    return Vector<F>(*this).rotateFromXToY(angle);
}

template <typename F>
Vector<F> Vector<F>::getRotatedFromYToZ(F angle) const
{
    return Vector<F>(*this).rotateFromYToZ(angle);
}

template <typename F>
Vector<F> Vector<F>::getRotatedFromZToX(F angle) const
{
    return Vector<F>(*this).rotateFromZToX(angle);
}

template <typename F>
F Vector<F>::getSmallestComponent() const
{
    F min_val = x;
    if (y < min_val) min_val = y;
    if (z < min_val) min_val = z;
    return min_val;
}

template <typename F>
size_t Vector<F>::getSmallestComponentIndex() const
{
    F min_val = x;
    size_t idx = 0;
    if (y < min_val) min_val = y; idx = 1;
    if (z < min_val) min_val = z; idx = 2;
    return idx;
}

template <typename F>
F Vector<F>::getLargestComponent() const
{
    F max_val = x;
    if (y > max_val) max_val = y;
    if (z > max_val) max_val = z;
    return max_val;
}

template <typename F>
size_t Vector<F>::getLargestComponentIndex() const
{
    F max_val = x;
    size_t idx = 0;
    if (y > max_val) max_val = y; idx = 1;
    if (z > max_val) max_val = z; idx = 2;
    return idx;
}

template <typename F>
F Vector<F>::getSmallestComponentNaNSafe() const
{
    F min_val = std::numeric_limits<F>::max();
    if (x < min_val) min_val = x;
    if (y < min_val) min_val = y;
    if (z < min_val) min_val = z;
    return min_val;
}

template <typename F>
F Vector<F>::getLargestComponentNaNSafe() const
{
    F max_val = std::numeric_limits<F>::min();
    if (x > max_val) max_val = x;
    if (y > max_val) max_val = y;
    if (z > max_val) max_val = z;
    return max_val;
}

template <typename F>
void Vector<F>::sortComponentwise(Vector<F>& min_vector,
                                  Vector<F>& max_vector)
{
    F temp;

    if (min_vector.x > max_vector.x)
    {
        temp = min_vector.x;
        min_vector.x = max_vector.x;
        max_vector.x = temp;
    }

    if (min_vector.y > max_vector.y)
    {
        temp = min_vector.y;
        min_vector.y = max_vector.y;
        max_vector.y = temp;
    }

    if (min_vector.z > max_vector.z)
    {
        temp = min_vector.z;
        min_vector.z = max_vector.z;
        max_vector.z = temp;
    }
}

template <typename F>
F Vector<F>::getLength() const
{
    return sqrt(getSquaredLength());
}

template <typename F>
F Vector<F>::getSquaredLength() const
{
    return x*x + y*y + z*z;
}

template <typename F>
Vector<F>& Vector<F>::normalize()
{
    F length = getLength();
    assert(length > 0);
    return *this /= length;
}

template <typename F>
Vector<F> Vector<F>::getNormalized() const
{
    return Vector<F>(*this).normalize();
}

template <typename F>
Vector<F> Vector<F>::getUnitNormalWith(const Vector<F>& other) const
{
    return cross(other).normalize();
}

template <typename F>
Vector<F> Vector<F>::getProjectedOn(const Vector<F>& other) const
{
    F other_squared_length = other.getSquaredLength();
    assert(other_squared_length > 0);
    return other*(dot(other)/other_squared_length);
}

template <typename F>
Vector<F> Vector<F>::getProjectedOnNormalTo(const Vector<F>& vector_1,
                                            const Vector<F>& vector_2) const
{
    return getProjectedOn(vector_1.cross(vector_2));
}

template <typename F>
Point<F> Vector<F>::toPoint() const
{
    return Point<F>(x, y, z);
}

template <typename F>
Vector4<F> Vector<F>::toVector4() const
{
    return Vector4<F>(x, y, z, 0);
}

template <typename F>
std::string Vector<F>::toString() const
{
    std::ostringstream string_stream;
    string_stream << "[" << x << ", " << y << ", " << z << "]";
    return string_stream.str();
}

template <typename F>
Vector<F> operator*(F factor, const Vector<F>& vector)
{
    return vector*factor;
}

template <typename F>
Vector<F> operator/(F factor, const Vector<F>& vector)
{
    return Vector<F>(factor/vector.x, factor/vector.y, factor/vector.z);
}

} // Geometry3D
