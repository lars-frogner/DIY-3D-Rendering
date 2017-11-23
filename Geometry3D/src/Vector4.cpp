#include "Vector4.hpp"
#include <cassert>
#include <cmath>
#include <sstream>

namespace Impact {
namespace Geometry3D {

Vector4::Vector4()
    : x(0), y(0), z(0), w(0) {}

Vector4::Vector4(imp_float x_new, imp_float y_new, imp_float z_new, imp_float new_w)
    : x(x_new), y(y_new), z(z_new), w(new_w) {}

Vector4 Vector4::operator+(const Vector4& other) const
{
    return Vector4(x + other.x, y + other.y, z + other.z, w + other.w);
}

Vector4 Vector4::operator+(const Point& point) const
{
    return Vector4(x + point.x, y + point.y, z + point.z, w + 1);
}

Vector4 Vector4::operator+(const Vector& vector) const
{
    return Vector4(x + vector.x, y + vector.y, z + vector.z, w);
}

Vector4 Vector4::operator-() const
{
    return Vector4(-x, -y, -z, -w);
}

Vector4 Vector4::operator-(const Vector4& other) const
{
    return *this + (-other);
}

Vector4 Vector4::operator-(const Point& point) const
{
    return Vector4(x - point.x, y - point.y, z - point.z, w - 1);
}

Vector4 Vector4::operator-(const Vector& vector) const
{
    return *this + (-vector);
}

Vector4& Vector4::operator+=(const Vector4& other)
{
    return shiftComponents(other.x, other.y, other.z, other.w);
}

Vector4& Vector4::operator+=(const Point& point)
{
    return shiftComponents(point.x, point.y, point.z, 1);
}

Vector4& Vector4::operator+=(const Vector& vector)
{
    return shiftComponents(vector.x, vector.y, vector.z, 0);
}

Vector4& Vector4::operator-=(const Vector4& other)
{
    return *this += (-other);
}

Vector4& Vector4::operator-=(const Point& point)
{
    return shiftComponents(-point.x, -point.y, -point.z, -1);
}

Vector4& Vector4::operator-=(const Vector& vector)
{
    return *this += (-vector);
}

Vector4 Vector4::operator*(imp_float factor) const
{
    return Vector4(x*factor, y*factor, z*factor, w*factor);
}

Vector4 Vector4::operator/(imp_float divisor) const
{
    return (*this)*(1/divisor);
}

Vector4& Vector4::operator*=(imp_float factor)
{
    return setComponents(x*factor, y*factor, z*factor, w*factor);
}

Vector4& Vector4::operator/=(imp_float divisor)
{
    return *this *= (1/divisor);
}

imp_float Vector4::dot(const Vector4& other) const
{
    return x*other.x + y*other.y + z*other.z;
}

Vector4& Vector4::swap(Vector4& other)
{
    imp_float temp = other.x;
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

Vector4& Vector4::setComponents(imp_float x_new, imp_float y_new, imp_float z_new, imp_float new_w)
{
    x = x_new; y = y_new; z = z_new; w = new_w;
    return *this;
}

Vector4& Vector4::shiftComponents(imp_float dx, imp_float dy, imp_float dz, imp_float dw)
{
    return setComponents(x + dx, y + dy, z + dz, w + dw);
}

imp_float Vector4::getLength() const
{
    return sqrt(getSquaredLength());
}

imp_float Vector4::getSquaredLength() const
{
    return x*x + y*y + z*z + w*w;
}

Vector4& Vector4::normalize()
{
    imp_float length = getLength();
    assert(length > 0);
    return *this /= length;
}

Vector4 Vector4::getNormalized() const
{
    return Vector4(*this).normalize();
}

Vector4& Vector4::homogenize()
{
    return *this /= w;
}

Vector4 Vector4::getHomogenized() const
{
    return Vector4(*this).homogenize();
}

Vector4 Vector4::getProjectedOn(const Vector4& other) const
{
    imp_float other_squared_length = other.getSquaredLength();
    assert(other_squared_length > 0);
    return other*(dot(other)/other_squared_length);
}

Point Vector4::getXYZ() const
{
    return Point(x, y, z);
}

Point Vector4::toPoint() const
{
    return Point(x/w, y/w, z/w);
}

Vector Vector4::toVector() const
{
    return Vector(x, y, z);
}

std::string Vector4::toString() const
{
    std::ostringstream string_stream;
    string_stream << "{" << x << ", " << y << ", " << z << ", " << w << "}";
    return string_stream.str();
}

Vector4 operator*(imp_float factor, const Vector4& vector)
{
    return vector*factor;
}

} // Geometry3D
} //Impact
