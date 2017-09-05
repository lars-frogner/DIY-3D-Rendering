#pragma once
#include <numeric>
#include <string>
#include <sstream>

namespace Geometry2D {

// Forward declaration of Vector class
template <typename F>
class Vector;

template <typename F>
class Point {

public:
    F x, y;

    Point<F>();
    Point<F>(F x_new, F y_new);

    static Point<F> origin();
    static Point<F> min();
    static Point<F> max();

    Vector<F> operator- (const Point<F>&  other)  const;
    Point<F>  operator+ (const Vector<F>& vector) const;
    Point<F>  operator- (const Vector<F>& vector) const;
    Point<F>& operator+=(const Vector<F>& vector);
    Point<F>& operator-=(const Vector<F>& vector);

    Point<F>& moveTo(F x_new, F y_new);
    Point<F>& translate(F dx, F dy);

    Point<F>& swap(Point<F>& other);
    Point<F>& useSmallestCoordinates(const Point<F>& other);
    Point<F>& useLargestCoordinates(const Point<F>& other);

    Vector<F> toVector() const;
    std::string toString() const;
};

template <typename F>
Point<F>::Point()
    : x(0), y(0) {}

template <typename F>
Point<F>::Point(F x_new, F y_new)
    : x(x_new), y(y_new) {}

template <typename F>
Point<F> Point<F>::origin()
{
    return Point<F>(0, 0);
}

template <typename F>
Point<F> Point<F>::min()
{
    F min = -std::numeric_limits<F>::max();
    return Point<F>(min, min);
}

template <typename F>
Point<F> Point<F>::max()
{
    F max = std::numeric_limits<F>::max();
    return Point<F>(max, max);
}

template <typename F>
Vector<F> Point<F>::operator-(const Point<F>& other) const
{
    return Vector<F>(x - other.x, y - other.y);
}

template <typename F>
Point<F> Point<F>::operator+(const Vector<F>& vector) const
{
    return Point<F>(x + vector.x, y + vector.y);
}

template <typename F>
Point<F> Point<F>::operator-(const Vector<F>& vector) const
{
    return *this + (-vector);
}

template <typename F>
Point<F>& Point<F>::operator+=(const Vector<F>& vector)
{
    return translate(vector.x, vector.y);
}

template <typename F>
Point<F>& Point<F>::operator-=(const Vector<F>& vector)
{
    return *this += (-vector);
}

template <typename F>
Point<F>& Point<F>::moveTo(F x_new, F y_new)
{
    x = x_new; y = y_new;
    return *this;
}

template <typename F>
Point<F>& Point<F>::translate(F dx, F dy)
{
    return moveTo(x + dx, y + dy);
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

    return *this;
}

template <typename F>
Point<F>& Point<F>::useSmallestCoordinates(const Point<F>& other)
{
    if (other.x < x) x = other.x;
    if (other.y < y) y = other.y;
    return *this;
}

template <typename F>
Point<F>& Point<F>::useLargestCoordinates(const Point<F>& other)
{
    if (other.x > x) x = other.x;
    if (other.y > y) y = other.y;
    return *this;
}

template <typename F>
Vector<F> Point<F>::toVector() const
{
    return Vector<F>(x, y);
}

template <typename F>
std::string Point<F>::toString() const
{
    std::ostringstream string_stream;
    string_stream << "(" << x << ", " << y << ")";
    return string_stream.str();
}

} // Geometry2D
