#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include <string>
#include <cassert>
#include <cmath>
#include <sstream>

namespace Impact {
namespace Geometry3D {

class Vector {

private:
	imp_float _pad; // Extra variable to achieve alignment in memory

public:
    imp_float x, y, z;

    Vector();
    Vector(imp_float x_new, imp_float y_new, imp_float z_new);

    static Vector zero();
    static Vector unitX();
    static Vector unitY();
    static Vector unitZ();
	
	static Vector randomDirectionOnHemisphere(const Vector& normal);
	static Vector randomCosineWeightedDirectionOnHemisphere(const Vector& normal);

	void setToZero();

	bool nonZero() const;

    Vector  operator+ (const Vector& other)  const;
    Point   operator+ (const Point&  point)  const;
    Vector  operator- ()					 const;
    Vector  operator- (const Vector& other)  const;
    Vector& operator+=(const Vector& other);
    Vector& operator-=(const Vector& other);
    Vector  operator* (imp_float factor)     const;
    Vector  operator/ (imp_float divisor)	 const;
    Vector& operator*=(imp_float factor);
    Vector& operator/=(imp_float divisor);
    
    Vector4 operator+ (const Vector4& vector4) const;
    Vector4 operator- (const Vector4& vector4) const;

    imp_float dot (const Vector& other) const;
    Vector cross(const Vector& other) const;

    Vector& swap(Vector& other);

    Vector& setComponents(imp_float x_new, imp_float y_new, imp_float z_new);
    Vector& shiftComponents(imp_float dx, imp_float dy, imp_float dz);

	Vector& addScaledVector(const Vector& vector, imp_float scale);
    
	Vector& rotateFromXToY(imp_float angle);
    Vector& rotateFromYToZ(imp_float angle);
    Vector& rotateFromZToX(imp_float angle);
    Vector getRotatedFromXToY(imp_float angle) const;
    Vector getRotatedFromYToZ(imp_float angle) const;
    Vector getRotatedFromZToX(imp_float angle) const;

	Vector getReflectedAbout(const Vector& direction) const;

	Vector getSnellRefracted(const Vector& surface_normal,
							 imp_float refractive_index_incoming,
							 imp_float refractive_index_outgoing) const;
	
	Vector getSnellRefracted(const Vector& surface_normal,
							 imp_float cos_incoming_angle,
							 imp_float refractive_index_incoming,
							 imp_float refractive_index_outgoing) const;

    imp_float getSmallestComponent() const;
    imp_uint getSmallestComponentIndex() const;
    imp_float getLargestComponent() const;
    imp_uint getLargestComponentIndex() const;
    imp_float getSmallestComponentNaNSafe() const;
    imp_float getLargestComponentNaNSafe() const;

    static void sortComponentwise(Vector& min_vector,
                                  Vector& max_vector);

    imp_float getLength() const;
    imp_float getSquaredLength() const;
    Vector& normalize();
    Vector getNormalized() const;

    Vector getUnitNormalWith(const Vector& other) const;
    Vector getProjectedOn(const Vector& other) const;
    Vector getProjectedOnNormalTo(const Vector& vector_1,
                                  const Vector& vector_2) const;

    Point toPoint() const;
    Vector4 toVector4() const;
    std::string toString() const;
};

Vector operator*(imp_float factor, const Vector& vector);

Vector operator/(imp_float factor, const Vector& vector);

Vector zeroAllowedDivision(imp_float factor, const Vector& vector);

} // Geometry3D
} // Impact
