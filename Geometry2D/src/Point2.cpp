#include "Point2.hpp"
#include "Vector2.hpp"
#include <sstream>

namespace Impact {
namespace Geometry2D {

Point::Point()
    : x(0), y(0) {}

Point::Point(imp_float x_new, imp_float y_new)
    : x(x_new), y(y_new) {}

Point Point::origin()
{
    return Point(0, 0);
}

Point Point::min()
{
    imp_float min = -IMP_FLOAT_MAX;
    return Point(min, min);
}

Point Point::max()
{
    imp_float max = IMP_FLOAT_MAX;
    return Point(max, max);
}

Vector Point::operator-(const Point& other) const
{
    return Vector(x - other.x, y - other.y);
}

Point Point::operator+(const Vector& vector) const
{
    return Point(x + vector.x, y + vector.y);
}

Point Point::operator-(const Vector& vector) const
{
    return *this + (-vector);
}

Point& Point::operator+=(const Vector& vector)
{
    return translate(vector.x, vector.y);
}

Point& Point::operator-=(const Vector& vector)
{
    return *this += (-vector);
}

Point& Point::moveTo(imp_float x_new, imp_float y_new)
{
    x = x_new; y = y_new;
    return *this;
}

Point& Point::translate(imp_float dx, imp_float dy)
{
    return moveTo(x + dx, y + dy);
}

Point& Point::swap(Point& other)
{
    imp_float temp = other.x;
    other.x = x;
    x = temp;

    temp = other.y;
    other.y = y;
    y = temp;

    return *this;
}

Point& Point::useSmallestCoordinates(const Point& other)
{
    if (other.x < x) x = other.x;
    if (other.y < y) y = other.y;
    return *this;
}

Point& Point::useLargestCoordinates(const Point& other)
{
    if (other.x > x) x = other.x;
    if (other.y > y) y = other.y;
    return *this;
}

Vector Point::toVector() const
{
    return Vector(x, y);
}

std::string Point::toString() const
{
    std::ostringstream string_stream;
    string_stream << "(" << x << ", " << y << ")";
    return string_stream.str();
}

} // Geometry2D
} // Impact
