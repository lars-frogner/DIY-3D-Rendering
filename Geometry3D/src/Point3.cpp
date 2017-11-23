#include "Point3.hpp"
#include "Vector3.hpp"
#include "Vector4.hpp"
#include <sstream>

namespace Impact {
namespace Geometry3D {

Point::Point()
    : x(0), y(0), z(0) {}

Point::Point(imp_float x_new, imp_float y_new, imp_float z_new)
    : x(x_new), y(y_new), z(z_new) {}

Point Point::origin()
{
    return Point(0, 0, 0);
}

Point Point::min()
{
    imp_float min = -IMP_FLOAT_MAX;
    return Point(min, min, min);
}

Point Point::max()
{
    imp_float max = IMP_FLOAT_MAX;
    return Point(max, max, max);
}

Vector Point::operator-(const Point& other) const
{
    return Vector(x - other.x, y - other.y, z - other.z);
}

Point Point::operator+(const Vector& vector) const
{
    return Point(x + vector.x, y + vector.y, z + vector.z);
}

Point Point::operator-(const Vector& vector) const
{
    return Point(x - vector.x, y - vector.y, z - vector.z);
}

Point& Point::operator+=(const Vector& vector)
{
	x += vector.x;
	y += vector.y;
	z += vector.z;
    return *this;
}

Point& Point::operator-=(const Vector& vector)
{
	x -= vector.x;
	y -= vector.y;
	z -= vector.z;
    return *this;
}

Point Point::operator*(imp_float factor) const
{
    return Point(x*factor, y*factor, z*factor);
}

Point Point::operator/(imp_float divisor) const
{
    imp_float factor = 1/divisor;
    return Point(x*factor, y*factor, z*factor);
}

Point& Point::operator*=(imp_float factor)
{
	x *= factor;
	y *= factor;
	z *= factor;
    return *this;
}

Point& Point::operator/=(imp_float divisor)
{
    imp_float factor = 1/divisor;
	x *= factor;
	y *= factor;
	z *= factor;
    return *this;
}

Vector4 Point::operator+(const Vector4& vector4) const
{
    return Vector4(x + vector4.x, y + vector4.y, z + vector4.z, 1 + vector4.w);
}

Vector4 Point::operator-(const Vector4& vector4) const
{
    return Vector4(x - vector4.x, y - vector4.y, z - vector4.z, 1 - vector4.w);
}

Point& Point::moveTo(imp_float x_new, imp_float y_new, imp_float z_new)
{
    x = x_new;
	y = y_new;
	z = z_new;
    return *this;
}

Point& Point::translate(imp_float dx, imp_float dy, imp_float dz)
{
	x += dx;
	y += dy;
	z += dz;
    return *this;
}

Point& Point::addScaledVector(const Vector& vector, imp_float scale)
{
	x += scale*vector.x;
	y += scale*vector.y;
	z += scale*vector.z;
	return *this;
}

Point& Point::swap(Point& other)
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

    return *this;
}

Point& Point::useSmallestCoordinates(const Point& other)
{
    if (other.x < x) x = other.x;
    if (other.y < y) y = other.y;
    if (other.z < z) z = other.z;
    return *this;
}

Point& Point::useLargestCoordinates(const Point& other)
{
    if (other.x > x) x = other.x;
    if (other.y > y) y = other.y;
    if (other.z > z) z = other.z;
    return *this;
}

Vector Point::toVector() const
{
    return Vector(x, y, z);
}

Vector4 Point::toVector4() const
{
    return Vector4(x, y, z, 1);
}

std::string Point::toString() const
{
    std::ostringstream string_stream;
    string_stream << "(" << x << ", " << y << ", " << z << ")";
    return string_stream.str();
}

Point operator*(imp_float factor, const Point& point)
{
    return point*factor;
}

Point operator/(imp_float factor, const Point& point)
{
    return Point(factor/point.x, factor/point.y, factor/point.z);
}

} // Geometry3D
} // Impact
