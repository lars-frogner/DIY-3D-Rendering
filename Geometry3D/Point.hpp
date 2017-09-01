#pragma once
#include <numeric>
#include <string>
#include <sstream>

namespace Geometry3D {

// Forward declaration of Vector class
template <typename F>
class Vector;

// Forward declaration of Vector4 class
template <typename F>
class Vector4;

template <typename F>
class Point {

public:
    F x, y, z;

    Point<F>();
    Point<F>(F x_new, F y_new, F z_new);

    static Point<F> origin();
    static Point<F> min();
    static Point<F> max();

    Vector<F> operator- (const Point<F>&  other)  const;
    Point<F>  operator+ (const Vector<F>& vector) const;
    Point<F>  operator- (const Vector<F>& vector) const;
    Point<F>& operator+=(const Vector<F>& vector);
    Point<F>& operator-=(const Vector<F>& vector);

    Point<F>  operator* (F factor)				  const;
    Point<F>  operator/ (F divisor)			      const;
    Point<F>& operator*=(F factor);
    Point<F>& operator/=(F divisor);
    
    Vector4<F> operator+ (const Vector4<F>& vector4) const;
    Vector4<F> operator- (const Vector4<F>& vector4) const;

    Point<F>& moveTo(F x_new, F y_new, F z_new);
    Point<F>& translate(F dx, F dy, F dz);

    Point<F>& swap(Point<F>& other);
    Point<F>& useSmallestCoordinates(const Point<F>& other);
    Point<F>& useLargestCoordinates(const Point<F>& other);

    Vector<F> toVector() const;
    Vector4<F> toVector4() const;
    std::string toString() const;
};

template <typename F>
Point<F> operator*(F factor, const Point<F>& vector);

template <typename F>
Point<F> operator/(F factor, const Point<F>& vector);

template <typename F>
Point<F>::Point()
    : x(0), y(0), z(0) {}

template <typename F>
Point<F>::Point(F x_new, F y_new, F z_new)
    : x(x_new), y(y_new), z(z_new) {}

template <typename F>
Point<F> Point<F>::origin()
{
    return Point<F>(0, 0, 0);
}

template <typename F>
Point<F> Point<F>::min()
{
    F min = -std::numeric_limits<F>::max();
    return Point<F>(min, min, min);
}

template <typename F>
Point<F> Point<F>::max()
{
    F max = std::numeric_limits<F>::max();
    return Point<F>(max, max, max);
}

template <typename F>
Vector<F> Point<F>::operator-(const Point<F>& other) const
{
    return Vector<F>(x - other.x, y - other.y, z - other.z);
}

template <typename F>
Point<F> Point<F>::operator+(const Vector<F>& vector) const
{
    return Point<F>(x + vector.x, y + vector.y, z + vector.z);
}

template <typename F>
Point<F> Point<F>::operator-(const Vector<F>& vector) const
{
    return *this + (-vector);
}

template <typename F>
Point<F>& Point<F>::operator+=(const Vector<F>& vector)
{
    return translate(vector.x, vector.y, vector.z);
}

template <typename F>
Point<F>& Point<F>::operator-=(const Vector<F>& vector)
{
    return *this += (-vector);
}

template <typename F>
Point<F> Point<F>::operator*(F factor) const
{
    return Point<F>(x*factor, y*factor, z*factor);
}

template <typename F>
Point<F> Point<F>::operator/(F divisor) const
{
    return (*this)*(1/divisor);
}

template <typename F>
Point<F>& Point<F>::operator*=(F factor)
{
    return moveTo(x*factor, y*factor, z*factor);
}

template <typename F>
Point<F>& Point<F>::operator/=(F divisor)
{
    return *this *= (1/divisor);
}

template <typename F>
Vector4<F> Point<F>::operator+(const Vector4<F>& vector4) const
{
    return Vector4<F>(x + vector4.x, y + vector4.y, z + vector4.z, 1 + vector4.w);
}

template <typename F>
Vector4<F> Point<F>::operator-(const Vector4<F>& vector4) const
{
    return Vector4<F>(x - vector4.x, y - vector4.y, z - vector4.z, 1 - vector4.w);
}

template <typename F>
Point<F>& Point<F>::moveTo(F x_new, F y_new, F z_new)
{
    x = x_new; y = y_new; z = z_new;
    return *this;
}

template <typename F>
Point<F>& Point<F>::translate(F dx, F dy, F dz)
{
    return moveTo(x + dx, y + dy, z + dz);
}

template <typename F>
Point<F>& Point<F>::swap(Point<F>& other)
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
Point<F>& Point<F>::useSmallestCoordinates(const Point<F>& other)
{
    if (other.x < x) x = other.x;
    if (other.y < y) y = other.y;
    if (other.z < z) z = other.z;
    return *this;
}

template <typename F>
Point<F>& Point<F>::useLargestCoordinates(const Point<F>& other)
{
    if (other.x > x) x = other.x;
    if (other.y > y) y = other.y;
    if (other.z > z) z = other.z;
    return *this;
}

template <typename F>
Vector<F> Point<F>::toVector() const
{
    return Vector<F>(x, y, z);
}

template <typename F>
Vector4<F> Point<F>::toVector4() const
{
    return Vector4<F>(x, y, z, 1);
}

template <typename F>
std::string Point<F>::toString() const
{
    std::ostringstream string_stream;
    string_stream << "(" << x << ", " << y << ", " << z << ")";
    return string_stream.str();
}

template <typename F>
Point<F> operator*(F factor, const Point<F>& point)
{
    return point*factor;
}

template <typename F>
Point<F> operator/(F factor, const Point<F>& point)
{
    return Point<F>(factor/point.x, factor/point.y, factor/point.z);
}

} // Geometry3D
