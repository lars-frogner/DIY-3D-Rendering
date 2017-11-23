#pragma once
#include "precision.hpp"
#include <string>

namespace Impact {
namespace Geometry2D {

// Forward declaration of Vector class
class Vector;

class Point {

public:
    imp_float x, y;

    Point();
    Point(imp_float x_new, imp_float y_new);

    static Point origin();
    static Point min();
    static Point max();

    Vector operator- (const Point&  other)  const;
    Point  operator+ (const Vector& vector) const;
    Point  operator- (const Vector& vector) const;
    Point& operator+=(const Vector& vector);
    Point& operator-=(const Vector& vector);

    Point& moveTo(imp_float x_new, imp_float y_new);
    Point& translate(imp_float dx, imp_float dy);

    Point& swap(Point& other);
    Point& useSmallestCoordinates(const Point& other);
    Point& useLargestCoordinates(const Point& other);

    Vector toVector() const;
    std::string toString() const;
};

} // Geometry2D
} // Impact
