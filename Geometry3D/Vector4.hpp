#pragma once
#include <math.h>
#include <assert.h>
#include <limits>
#include <string>
#include <sstream>
#include "Point.hpp"
#include "Vector.hpp"

namespace Geometry3D {

template <typename F>
class Vector4 {

public:
    F x, y, z, w;

    Vector4<F>();
    Vector4<F>(F x_new, F y_new, F z_new, F new_w);

    Vector4<F>  operator+ (const Vector4<F>& other)  const;
    Vector4<F>  operator+ (const Point<F>&   point)  const;
    Vector4<F>  operator+ (const Vector<F>&  vector) const;

    Vector4<F>  operator- ()						 const;
    Vector4<F>  operator- (const Vector4<F>& other)  const;
    Vector4<F>  operator- (const Point<F>&   point)  const;
    Vector4<F>  operator- (const Vector<F>&  vector) const;

    Vector4<F>& operator+=(const Vector4<F>& other);
    Vector4<F>& operator+=(const Point<F>&   point);
    Vector4<F>& operator+=(const Vector<F>&  vector);
    
    Vector4<F>& operator-=(const Vector4<F>& other);
    Vector4<F>& operator-=(const Point<F>&   point);
    Vector4<F>& operator-=(const Vector<F>&  vector);

    Vector4<F>  operator* (F factor)		         const;
    Vector4<F>  operator/ (F divisor)			     const;
    Vector4<F>& operator*=(F factor);
    Vector4<F>& operator/=(F divisor);

    F dot  (const Vector4<F>& other) const;

    Vector4<F>& swap(Vector4<F>& other);

    Vector4<F>& setComponents(F x_new, F y_new, F z_new, F new_w);
    Vector4<F>& shiftComponents(F dx, F dy, F dz, F dw);

    F getLength() const;
    F getSquaredLength() const;
    Vector4<F>& normalize();
    Vector4<F> getNormalized() const;

    Vector4<F>& homogenize();
    Vector4<F> getHomogenized() const;

    Vector4<F> getProjectedOn(const Vector4<F>& other) const;
    
    Point<F> getXYZ() const;
    Point<F> toPoint() const;
    Vector<F> toVector() const;
    std::string toString() const;
};

template <typename F>
Vector4<F> operator*(F factor, const Vector4<F>& vector);

template <typename F>
Vector4<F>::Vector4()
    : x(0), y(0), z(0), w(0) {}

template <typename F>
Vector4<F>::Vector4(F x_new, F y_new, F z_new, F new_w)
    : x(x_new), y(y_new), z(z_new), w(new_w) {}

template <typename F>
Vector4<F> Vector4<F>::operator+(const Vector4<F>& other) const
{
    return Vector4<F>(x + other.x, y + other.y, z + other.z, w + other.w);
}

template <typename F>
Vector4<F> Vector4<F>::operator+(const Point<F>& point) const
{
    return Vector4<F>(x + point.x, y + point.y, z + point.z, w + 1);
}

template <typename F>
Vector4<F> Vector4<F>::operator+(const Vector<F>& vector) const
{
    return Vector4<F>(x + vector.x, y + vector.y, z + vector.z, w);
}

template <typename F>
Vector4<F> Vector4<F>::operator-() const
{
    return Vector4<F>(-x, -y, -z, -w);
}

template <typename F>
Vector4<F> Vector4<F>::operator-(const Vector4<F>& other) const
{
    return *this + (-other);
}

template <typename F>
Vector4<F> Vector4<F>::operator-(const Point<F>& point) const
{
    return Vector4<F>(x - point.x, y - point.y, z - point.z, w - 1);
}

template <typename F>
Vector4<F> Vector4<F>::operator-(const Vector<F>& vector) const
{
    return *this + (-vector);
}

template <typename F>
Vector4<F>& Vector4<F>::operator+=(const Vector4<F>& other)
{
    return shiftComponents(other.x, other.y, other.z, other.w);
}

template <typename F>
Vector4<F>& Vector4<F>::operator+=(const Point<F>& point)
{
    return shiftComponents(point.x, point.y, point.z, 1);
}

template <typename F>
Vector4<F>& Vector4<F>::operator+=(const Vector<F>& vector)
{
    return shiftComponents(vector.x, vector.y, vector.z, 0);
}

template <typename F>
Vector4<F>& Vector4<F>::operator-=(const Vector4<F>& other)
{
    return *this += (-other);
}

template <typename F>
Vector4<F>& Vector4<F>::operator-=(const Point<F>& point)
{
    return shiftComponents(-point.x, -point.y, -point.z, -1);
}

template <typename F>
Vector4<F>& Vector4<F>::operator-=(const Vector<F>& vector)
{
    return *this += (-vector);
}

template <typename F>
Vector4<F> Vector4<F>::operator*(F factor) const
{
    return Vector4<F>(x*factor, y*factor, z*factor, w*factor);
}

template <typename F>
Vector4<F> Vector4<F>::operator/(F divisor) const
{
    return (*this)*(1/divisor);
}

template <typename F>
Vector4<F>& Vector4<F>::operator*=(F factor)
{
    return setComponents(x*factor, y*factor, z*factor, w*factor);
}

template <typename F>
Vector4<F>& Vector4<F>::operator/=(F divisor)
{
    return *this *= (1/divisor);
}

template <typename F>
F Vector4<F>::dot(const Vector4<F>& other) const
{
    return x*other.x + y*other.y + z*other.z;
}

template <typename F>
Vector4<F>& Vector4<F>::swap(Vector4<F>& other)
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

    temp = other.w;
    other.w = w;
    w = temp;

    return *this;
}

template <typename F>
Vector4<F>& Vector4<F>::setComponents(F x_new, F y_new, F z_new, F new_w)
{
    x = x_new; y = y_new; z = z_new; w = new_w;
    return *this;
}

template <typename F>
Vector4<F>& Vector4<F>::shiftComponents(F dx, F dy, F dz, F dw)
{
    return setComponents(x + dx, y + dy, z + dz, w + dw);
}

template <typename F>
F Vector4<F>::getLength() const
{
    return sqrt(getSquaredLength());
}

template <typename F>
F Vector4<F>::getSquaredLength() const
{
    return x*x + y*y + z*z + w*w;
}

template <typename F>
Vector4<F>& Vector4<F>::normalize()
{
    F length = getLength();
    assert(length > 0);
    return *this /= length;
}

template <typename F>
Vector4<F> Vector4<F>::getNormalized() const
{
    return Vector4<F>(*this).normalize();
}

template <typename F>
Vector4<F>& Vector4<F>::homogenize()
{
    return *this /= w;
}

template <typename F>
Vector4<F> Vector4<F>::getHomogenized() const
{
    return Vector4<F>(*this).homogenize();
}

template <typename F>
Vector4<F> Vector4<F>::getProjectedOn(const Vector4<F>& other) const
{
    F other_squared_length = other.getSquaredLength();
    assert(other_squared_length > 0);
    return other*(dot(other)/other_squared_length);
}

template <typename F>
Point<F> Vector4<F>::getXYZ() const
{
    return Point<F>(x, y, z);
}

template <typename F>
Point<F> Vector4<F>::toPoint() const
{
    return Point<F>(x/w, y/w, z/w);
}

template <typename F>
Vector<F> Vector4<F>::toVector() const
{
    return Vector<F>(x, y, z);
}

template <typename F>
std::string Vector4<F>::toString() const
{
    std::ostringstream string_stream;
    string_stream << "{" << x << ", " << y << ", " << z << ", " << w << "}";
    return string_stream.str();
}

template <typename F>
Vector4<F> operator*(F factor, const Vector4<F>& vector)
{
    return vector*factor;
}

} // Geometry3D
