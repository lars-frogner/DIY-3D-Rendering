#pragma once
#include "precision.hpp"
#include "Point2.hpp"

namespace Impact {
namespace Geometry2D {

class Vector {

public:
    imp_float x, y;

    Vector();
    Vector(imp_float x_new, imp_float y_new);

    static Vector zero();
    static Vector unitX();
    static Vector unitY();

    Vector  operator+ (const Vector& other)  const;
    Point   operator+ (const Point&  point)  const;
    Vector  operator- ()					 const;
    Vector  operator- (const Vector& other)  const;
    Vector& operator+=(const Vector& other);
    Vector& operator-=(const Vector& other);
    Vector  operator* (imp_float factor)		 const;
    Vector  operator/ (imp_float divisor)	 const;
    Vector& operator*=(imp_float factor);
    Vector& operator/=(imp_float divisor);

    imp_float dot (const Vector& other) const;
    Vector cross() const;

    Vector& swap(Vector& other);

    Vector& setComponents(imp_float x_new, imp_float y_new);
    Vector& shiftComponents(imp_float dx, imp_float dy);

    Vector& rotateFromXToY(imp_float angle);
    Vector getRotatedFromXToY(imp_float angle) const;

    imp_float getSmallestComponent()		   const;
    imp_uint  getSmallestComponentIndex()   const;
    imp_float getLargestComponent()		   const;
    imp_uint  getLargestComponentIndex()    const;
    imp_float getSmallestComponentNaNSafe() const;
    imp_float getLargestComponentNaNSafe()  const;

    static void sortComponentwise(Vector& min_vector,
                                  Vector& max_vector);

    imp_float getLength() const;
    imp_float getSquaredLength() const;
    Vector& normalize();
    Vector getNormalized() const;
    
    Vector getUnitNormal() const;
    Vector getProjectedOn(const Vector& other) const;

    Point toPoint() const;
    std::string toString() const;
};

Vector operator*(imp_float factor, const Vector& vector);

Vector operator/(imp_float factor, const Vector& vector);

} // Geometry2D
} // Impact
