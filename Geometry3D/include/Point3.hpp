#pragma once
#include "precision.hpp"
#include <string>

namespace Impact {
namespace Geometry3D {

// Forward declaration of Vector class
class Vector;

// Forward declaration of Vector4 class
class Vector4;

class Point {

private:
	imp_float _pad; // Extra variable to achieve alignment in memory

public:
    imp_float x, y, z;

    Point();
    Point(imp_float x_new, imp_float y_new, imp_float z_new);

    static Point origin();
    static Point min();
    static Point max();

    Vector operator- (const Point&  other)   const;
    Point  operator+ (const Vector& vector)  const;
    Point  operator- (const Vector& vector)  const;
    Point& operator+=(const Vector& vector);
    Point& operator-=(const Vector& vector);
    Point  operator* (imp_float factor)		 const;
    Point  operator/ (imp_float divisor)	 const;
    Point& operator*=(imp_float factor);
    Point& operator/=(imp_float divisor);
  
    Vector4 operator+ (const Vector4& vector4) const;
    Vector4 operator- (const Vector4& vector4) const;

    Point& moveTo(imp_float x_new, imp_float y_new, imp_float z_new);
    Point& translate(imp_float dx, imp_float dy, imp_float dz);
	
    Point& addScaledVector(const Vector& vector, imp_float scale);

    Point& swap(Point& other);
    Point& useSmallestCoordinates(const Point& other);
    Point& useLargestCoordinates(const Point& other);

    Vector toVector() const;
    Vector4 toVector4() const;
    std::string toString() const;
};

Point operator*(imp_float factor, const Point& vector);

Point operator/(imp_float factor, const Point& vector);

} // Geometry3D
} // Impact
