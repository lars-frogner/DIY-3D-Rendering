#include "Vector2.hpp"
#include <cassert>
#include <cmath>
#include <string>
#include <sstream>

namespace Impact {
namespace Geometry2D {

Vector::Vector()
    : x(0), y(0) {}

Vector::Vector(imp_float x_new, imp_float y_new)
    : x(x_new), y(y_new) {}

Vector Vector::zero()
{
    return Vector(0, 0);
}

Vector Vector::unitX()
{
    return Vector(1, 0);
}

Vector Vector::unitY()
{
    return Vector(0, 1);
}

Vector Vector::operator+(const Vector& other) const
{
    return Vector(x + other.x, y + other.y);
}

Point Vector::operator+(const Point& point) const
{
    return Point(x + point.x, y + point.y);
}

Vector Vector::operator-() const
{
    return Vector(-x, -y);
}

Vector Vector::operator-(const Vector& other) const
{
    return *this + (-other);
}

Vector& Vector::operator+=(const Vector& other)
{
    return shiftComponents(other.x, other.y);
}

Vector& Vector::operator-=(const Vector& other)
{
    return *this += (-other);
}

Vector Vector::operator*(imp_float factor) const
{
    return Vector(x*factor, y*factor);
}

Vector Vector::operator/(imp_float divisor) const
{
	assert(divisor != 0);

    return (*this)*(1/divisor);
}

Vector& Vector::operator*=(imp_float factor)
{
    return setComponents(x*factor, y*factor);
}

Vector& Vector::operator/=(imp_float divisor)
{
	assert(divisor != 0);

    return *this *= (1/divisor);
}

imp_float Vector::dot(const Vector& other) const
{
    return x*other.x + y*other.y;
}

Vector Vector::cross() const
{
    return Vector(y, -x);
}

Vector& Vector::swap(Vector& other)
{
    imp_float temp = other.x;
    other.x = x;
    x = temp;

    temp = other.y;
    other.y = y;
    y = temp;

    return *this;
}

Vector& Vector::setComponents(imp_float x_new, imp_float y_new)
{
    x = x_new; y = y_new;
    return *this;
}

Vector& Vector::shiftComponents(imp_float dx, imp_float dy)
{
    return setComponents(x + dx, y + dy);
}

Vector& Vector::rotateFromXToY(imp_float angle)
{
    imp_float cos_angle = cos(angle);
    imp_float sin_angle = sin(angle);
    imp_float new_x = x*cos_angle - y*sin_angle;
    y = x*sin_angle + y*cos_angle;
    x = new_x;
    return *this;
}

Vector Vector::getRotatedFromXToY(imp_float angle) const
{
    return Vector(*this).rotateFromXToY(angle);
}

imp_float Vector::getSmallestComponent() const
{
    return (x < y) ? x : y;
}

imp_uint Vector::getSmallestComponentIndex() const
{
    imp_float min_val = x;
    imp_uint idx = 0;
    if (y < min_val) min_val = y; idx = 1;
    return idx;
}

imp_float Vector::getLargestComponent() const
{
    return (x > y) ? x : y;
}

imp_uint Vector::getLargestComponentIndex() const
{
    imp_float max_val = x;
    imp_uint idx = 0;
    if (y > max_val) max_val = y; idx = 1;
    return idx;
}

imp_float Vector::getSmallestComponentNaNSafe() const
{
    imp_float min_val = IMP_FLOAT_MAX;
    if (x < min_val) min_val = x;
    if (y < min_val) min_val = y;
    return min_val;
}

imp_float Vector::getLargestComponentNaNSafe() const
{
    imp_float max_val = IMP_FLOAT_MIN;
    if (x > max_val) max_val = x;
    if (y > max_val) max_val = y;
    return max_val;
}

void Vector::sortComponentwise(Vector& min_vector,
                                  Vector& max_vector)
{
    imp_float temp;

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

imp_float Vector::getLength() const
{
    return sqrt(getSquaredLength());
}

imp_float Vector::getSquaredLength() const
{
    return x*x + y*y;
}

Vector& Vector::normalize()
{
    imp_float length = getLength();
    assert(length > 0);
    return *this /= length;
}

Vector Vector::getNormalized() const
{
    return Vector(*this).normalize();
}

Vector Vector::getUnitNormal() const
{
    return cross().normalize();
}

Vector Vector::getProjectedOn(const Vector& other) const
{
    imp_float other_squared_length = other.getSquaredLength();
    assert(other_squared_length > 0);
    return other*(dot(other)/other_squared_length);
}

Point Vector::toPoint() const
{
    return Point(x, y);
}

std::string Vector::toString() const
{
    std::ostringstream string_stream;
    string_stream << "[" << x << ", " << y << "]";
    return string_stream.str();
}

Vector operator*(imp_float factor, const Vector& vector)
{
    return vector*factor;
}

Vector operator/(imp_float factor, const Vector& vector)
{
    return Vector(factor/vector.x, factor/vector.y);
}

} // Geometry2D
} // Impact
