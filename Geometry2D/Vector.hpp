#pragma once
#include <math.h>
#include <assert.h>
#include <limits>
#include <string>
#include <sstream>
#include "Point.hpp"

namespace Geometry2D {

template <typename F>
class Vector {

public:
    F x, y;

    Vector<F>();
    Vector<F>(F x_new, F y_new);

    static Vector<F> zero();
    static Vector<F> unitX();
    static Vector<F> unitY();

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

    F		  dot  (const Vector<F>& other) const;
    Vector<F> cross() const;

    Vector<F>& swap(Vector<F>& other);

    Vector<F>& setComponents(F x_new, F y_new);
    Vector<F>& shiftComponents(F dx, F dy);

    Vector<F>& rotateFromXToY(F angle);
    Vector<F> getRotatedFromXToY(F angle) const;

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
    
    Vector<F> getUnitNormal() const;
    Vector<F> getProjectedOn(const Vector<F>& other) const;

    Point<F> toPoint() const;
    std::string toString() const;
};

template <typename F>
Vector<F> operator*(F factor, const Vector<F>& vector);

template <typename F>
Vector<F> operator/(F factor, const Vector<F>& vector);

template <typename F>
Vector<F>::Vector()
    : x(0), y(0) {}

template <typename F>
Vector<F>::Vector(F x_new, F y_new)
    : x(x_new), y(y_new) {}

template <typename F>
Vector<F> Vector<F>::zero()
{
    return Vector<F>(0, 0);
}

template <typename F>
Vector<F> Vector<F>::unitX()
{
    return Vector<F>(1, 0);
}

template <typename F>
Vector<F> Vector<F>::unitY()
{
    return Vector<F>(0, 1);
}

template <typename F>
Vector<F> Vector<F>::operator+(const Vector<F>& other) const
{
    return Vector<F>(x + other.x, y + other.y);
}

template <typename F>
Point<F> Vector<F>::operator+(const Point<F>& point) const
{
    return Point<F>(x + point.x, y + point.y);
}

template <typename F>
Vector<F> Vector<F>::operator-() const
{
    return Vector<F>(-x, -y);
}

template <typename F>
Vector<F> Vector<F>::operator-(const Vector<F>& other) const
{
    return *this + (-other);
}

template <typename F>
Vector<F>& Vector<F>::operator+=(const Vector<F>& other)
{
    return shiftComponents(other.x, other.y);
}

template <typename F>
Vector<F>& Vector<F>::operator-=(const Vector<F>& other)
{
    return *this += (-other);
}

template <typename F>
Vector<F> Vector<F>::operator*(F factor) const
{
    return Vector<F>(x*factor, y*factor);
}

template <typename F>
Vector<F> Vector<F>::operator/(F divisor) const
{
    return (*this)*(1/divisor);
}

template <typename F>
Vector<F>& Vector<F>::operator*=(F factor)
{
    return setComponents(x*factor, y*factor);
}

template <typename F>
Vector<F>& Vector<F>::operator/=(F divisor)
{
    return *this *= (1/divisor);
}

template <typename F>
F Vector<F>::dot(const Vector<F>& other) const
{
    return x*other.x + y*other.y;
}

template <typename F>
Vector<F> Vector<F>::cross() const
{
    return Vector<F>(y, -x);
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

    return *this;
}

template <typename F>
Vector<F>& Vector<F>::setComponents(F x_new, F y_new)
{
    x = x_new; y = y_new;
    return *this;
}

template <typename F>
Vector<F>& Vector<F>::shiftComponents(F dx, F dy)
{
    return setComponents(x + dx, y + dy);
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
Vector<F> Vector<F>::getRotatedFromXToY(F angle) const
{
    return Vector<F>(*this).rotateFromXToY(angle);
}

template <typename F>
F Vector<F>::getSmallestComponent() const
{
    return (x < y) ? x : y;
}

template <typename F>
size_t Vector<F>::getSmallestComponentIndex() const
{
    F min_val = x;
    size_t idx = 0;
    if (y < min_val) min_val = y; idx = 1;
    return idx;
}

template <typename F>
F Vector<F>::getLargestComponent() const
{
    return (x > y) ? x : y;
}

template <typename F>
size_t Vector<F>::getLargestComponentIndex() const
{
    F max_val = x;
    size_t idx = 0;
    if (y > max_val) max_val = y; idx = 1;
    return idx;
}

template <typename F>
F Vector<F>::getSmallestComponentNaNSafe() const
{
    F min_val = std::numeric_limits<F>::max();
    if (x < min_val) min_val = x;
    if (y < min_val) min_val = y;
    return min_val;
}

template <typename F>
F Vector<F>::getLargestComponentNaNSafe() const
{
    F max_val = std::numeric_limits<F>::min();
    if (x > max_val) max_val = x;
    if (y > max_val) max_val = y;
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
}

template <typename F>
F Vector<F>::getLength() const
{
    return sqrt(getSquaredLength());
}

template <typename F>
F Vector<F>::getSquaredLength() const
{
    return x*x + y*y;
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
Vector<F> Vector<F>::getUnitNormal() const
{
    return cross().normalize();
}

template <typename F>
Vector<F> Vector<F>::getProjectedOn(const Vector<F>& other) const
{
    F other_squared_length = other.getSquaredLength();
    assert(other_squared_length > 0);
    return other*(dot(other)/other_squared_length);
}

template <typename F>
Point<F> Vector<F>::toPoint() const
{
    return Point<F>(x, y);
}

template <typename F>
std::string Vector<F>::toString() const
{
    std::ostringstream string_stream;
    string_stream << "[" << x << ", " << y << "]";
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
    return Vector<F>(factor/vector.x, factor/vector.y);
}

} // Geometry2D
