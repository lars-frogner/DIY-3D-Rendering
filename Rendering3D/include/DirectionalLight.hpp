#pragma once
#include "precision.hpp"
#include "Light.hpp"
#include "Color.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Vector4.hpp"
#include "CoordinateFrame.hpp"
#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"

namespace Impact {
namespace Rendering3D {

class DirectionalLight : public Light {
    
private:
    typedef Geometry3D::Point Point;
    typedef Geometry3D::Vector Vector;
    typedef Geometry3D::Vector4 Vector4;
    typedef Geometry3D::CoordinateFrame CoordinateFrame;
    typedef Geometry3D::LinearTransformation LinearTransformation;
    typedef Geometry3D::AffineTransformation AffineTransformation;
    
    Vector _direction;
    Biradiance _biradiance;

public:
    DirectionalLight(const Vector& new_direction,
                     const Biradiance& new_biradiance);

    CoordinateFrame getCoordinateFrame() const;
    imp_float getSurfaceArea() const;
    imp_uint getNumberOfSamples() const;

    Vector4 getRandomPoint() const;
    Biradiance getBiradiance(const Vector4& source_point,
                             const Point& surface_point) const;

    Power getTotalPower() const;

	void setCoordinateFrame(const CoordinateFrame& cframe);

    void applyTransformation(const LinearTransformation& transformation);
    void applyTransformation(const AffineTransformation& transformation);
};

} // Rendering3D
} // Impact
