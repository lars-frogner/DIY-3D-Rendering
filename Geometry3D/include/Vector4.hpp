#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include <string>

namespace Impact {
namespace Geometry3D {

class Vector4 {

public:
    imp_float x, y, z, w;

    Vector4();
    Vector4(imp_float x_new, imp_float y_new, imp_float z_new, imp_float new_w);

    Vector4  operator+ (const Vector4& other)  const;
    Vector4  operator+ (const Point&   point)  const;
    Vector4  operator+ (const Vector&  vector) const;

    Vector4  operator- ()					   const;
    Vector4  operator- (const Vector4& other)  const;
    Vector4  operator- (const Point&   point)  const;
    Vector4  operator- (const Vector&  vector) const;

    Vector4& operator+=(const Vector4& other);
    Vector4& operator+=(const Point&   point);
    Vector4& operator+=(const Vector&  vector);
    
    Vector4& operator-=(const Vector4& other);
    Vector4& operator-=(const Point&   point);
    Vector4& operator-=(const Vector&  vector);

    Vector4  operator* (imp_float factor)  const;
    Vector4  operator/ (imp_float divisor) const;
    Vector4& operator*=(imp_float factor);
    Vector4& operator/=(imp_float divisor);

    imp_float dot  (const Vector4& other) const;

    Vector4& swap(Vector4& other);

    Vector4& setComponents(imp_float x_new, imp_float y_new, imp_float z_new, imp_float new_w);
    Vector4& shiftComponents(imp_float dx, imp_float dy, imp_float dz, imp_float dw);

    imp_float getLength() const;
    imp_float getSquaredLength() const;
    Vector4& normalize();
    Vector4 getNormalized() const;

    Vector4& homogenize();
    Vector4 getHomogenized() const;

    Vector4 getProjectedOn(const Vector4& other) const;
    
    Point getXYZ() const;
    Point toPoint() const;
    Vector toVector() const;
    std::string toString() const;
};

Vector4 operator*(imp_float factor, const Vector4& vector);

} // Geometry3D
} //Impact
