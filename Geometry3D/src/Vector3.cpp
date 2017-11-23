#include "Vector3.hpp"
#include "Vector4.hpp"
#include <cassert>
#include <cmath>
#include <sstream>

namespace Impact {
namespace Geometry3D {

Vector::Vector()
    : x(0), y(0), z(0) {}

Vector::Vector(imp_float x_new, imp_float y_new, imp_float z_new)
    : x(x_new), y(y_new), z(z_new) {}

Vector Vector::zero()
{
    return Vector(0, 0, 0);
}

Vector Vector::unitX()
{
    return Vector(1, 0, 0);
}

Vector Vector::unitY()
{
    return Vector(0, 1, 0);
}

Vector Vector::unitZ()
{
    return Vector(0, 0, 1);
}

void Vector::setToZero()
{
	x = 0;
	y = 0;
	z = 0;
}

Vector Vector::operator+(const Vector& other) const
{
    return Vector(x + other.x, y + other.y, z + other.z);
}

Point Vector::operator+(const Point& point) const
{
    return Point(x + point.x, y + point.y, z + point.z);
}

Vector Vector::operator-() const
{
    return Vector(-x, -y, -z);
}

Vector Vector::operator-(const Vector& other) const
{
    return Vector(x - other.x, y - other.y, z - other.z);
}

Vector& Vector::operator+=(const Vector& other)
{
	x += other.x;
	y += other.y;
	z += other.z;
    return *this;
}

Vector& Vector::operator-=(const Vector& other)
{
	x -= other.x;
	y -= other.y;
	z -= other.z;
    return *this;
}

Vector Vector::operator*(imp_float factor) const
{
    return Vector(x*factor, y*factor, z*factor);
}

Vector Vector::operator/(imp_float divisor) const
{
    imp_float factor = 1/divisor;
    return Vector(x*factor, y*factor, z*factor);
}

Vector& Vector::operator*=(imp_float factor)
{
	x *= factor;
	y *= factor;
	z *= factor;
    return *this;
}

Vector& Vector::operator/=(imp_float divisor)
{
    imp_float factor = 1/divisor;
	x *= factor;
	y *= factor;
	z *= factor;
    return *this;
}

Vector4 Vector::operator+(const Vector4& vector4) const
{
    return Vector4(x + vector4.x, y + vector4.y, z + vector4.z, vector4.w);
}

Vector4 Vector::operator-(const Vector4& vector4) const
{
    return Vector4(x - vector4.x, y - vector4.y, z - vector4.z, -vector4.w);
}

imp_float Vector::dot(const Vector& other) const
{
    return x*other.x + y*other.y + z*other.z;
}

Vector Vector::cross(const Vector& other) const
{
    return Vector(y*other.z - z*other.y,
                  z*other.x - x*other.z,
                  x*other.y - y*other.x);
}

Vector& Vector::swap(Vector& other)
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

Vector& Vector::setComponents(imp_float x_new, imp_float y_new, imp_float z_new)
{
    x = x_new;
	y = y_new;
	z = z_new;
    return *this;
}

Vector& Vector::shiftComponents(imp_float dx, imp_float dy, imp_float dz)
{
	x += dx;
	y += dy;
	z += dz;
    return *this;
}

Vector& Vector::addScaledVector(const Vector& vector, imp_float scale)
{
	x += scale*vector.x;
	y += scale*vector.y;
	z += scale*vector.z;
    return *this;
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

Vector& Vector::rotateFromYToZ(imp_float angle)
{
    imp_float cos_angle = cos(angle);
    imp_float sin_angle = sin(angle);
    imp_float new_y = y*cos_angle - z*sin_angle;
    z = y*sin_angle + z*cos_angle;
    y = new_y;
    return *this;
}

Vector& Vector::rotateFromZToX(imp_float angle)
{
    imp_float cos_angle = cos(angle);
    imp_float sin_angle = sin(angle);
    imp_float new_z = -x*sin_angle + z*cos_angle;
    x = x*cos_angle + z*sin_angle;
    z = new_z;
    return *this;
}

Vector Vector::getRotatedFromXToY(imp_float angle) const
{
    return Vector(*this).rotateFromXToY(angle);
}

Vector Vector::getRotatedFromYToZ(imp_float angle) const
{
    return Vector(*this).rotateFromYToZ(angle);
}

Vector Vector::getRotatedFromZToX(imp_float angle) const
{
    return Vector(*this).rotateFromZToX(angle);
}

imp_float Vector::getSmallestComponent() const
{
    imp_float min_val = x;
    if (y < min_val) min_val = y;
    if (z < min_val) min_val = z;
    return min_val;
}

imp_uint Vector::getSmallestComponentIndex() const
{
    imp_float min_val = x;
    imp_uint idx = 0;
    if (y < min_val) min_val = y; idx = 1;
    if (z < min_val) min_val = z; idx = 2;
    return idx;
}

imp_float Vector::getLargestComponent() const
{
    imp_float max_val = x;
    if (y > max_val) max_val = y;
    if (z > max_val) max_val = z;
    return max_val;
}

imp_uint Vector::getLargestComponentIndex() const
{
    imp_float max_val = x;
    imp_uint idx = 0;
    if (y > max_val) max_val = y; idx = 1;
    if (z > max_val) max_val = z; idx = 2;
    return idx;
}

imp_float Vector::getSmallestComponentNaNSafe() const
{
    imp_float min_val = IMP_FLOAT_MAX;
    if (x < min_val) min_val = x;
    if (y < min_val) min_val = y;
    if (z < min_val) min_val = z;
    return min_val;
}

imp_float Vector::getLargestComponentNaNSafe() const
{
    imp_float max_val = IMP_FLOAT_MIN;
    if (x > max_val) max_val = x;
    if (y > max_val) max_val = y;
    if (z > max_val) max_val = z;
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

    if (min_vector.z > max_vector.z)
    {
        temp = min_vector.z;
        min_vector.z = max_vector.z;
        max_vector.z = temp;
    }
}

imp_float Vector::getLength() const
{
    return sqrt(getSquaredLength());
}

imp_float Vector::getSquaredLength() const
{
    return x*x + y*y + z*z;
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

Vector Vector::getUnitNormalWith(const Vector& other) const
{
    return cross(other).normalize();
}

Vector Vector::getProjectedOn(const Vector& other) const
{
    imp_float other_squared_length = other.getSquaredLength();
    assert(other_squared_length > 0);
    return other*(dot(other)/other_squared_length);
}

Vector Vector::getProjectedOnNormalTo(const Vector& vector_1,
                                      const Vector& vector_2) const
{
    return getProjectedOn(vector_1.cross(vector_2));
}

Point Vector::toPoint() const
{
    return Point(x, y, z);
}

Vector4 Vector::toVector4() const
{
    return Vector4(x, y, z, 0);
}

std::string Vector::toString() const
{
    std::ostringstream string_stream;
    string_stream << "[" << x << ", " << y << ", " << z << "]";
    return string_stream.str();
}

Vector operator*(imp_float factor, const Vector& vector)
{
    return vector*factor;
}

Vector operator/(imp_float factor, const Vector& vector)
{
    return Vector(factor/vector.x, factor/vector.y, factor/vector.z);
}

} // Geometry3D
} // Impact
