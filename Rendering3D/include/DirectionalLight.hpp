#pragma once
#include "precision.hpp"
#include "Color.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "CoordinateFrame.hpp"
#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"

namespace Impact {
namespace Rendering3D {

class DirectionalLight {
    
private:
    typedef Geometry3D::Point Point;
    typedef Geometry3D::Vector Vector;
    typedef Geometry3D::CoordinateFrame CoordinateFrame;
    typedef Geometry3D::LinearTransformation LinearTransformation;
    typedef Geometry3D::AffineTransformation AffineTransformation;
    
    Vector _direction;
    Biradiance _biradiance;

public:
	bool creates_shadows = true;
	Radiance ambient_radiance = Radiance::black();

    DirectionalLight(const Vector& new_direction,
                     const Biradiance& new_biradiance);

    CoordinateFrame getCoordinateFrame() const;

    const Vector& getDirection() const;
    const Biradiance& getBiradiance() const;

	void setCoordinateFrame(const CoordinateFrame& cframe);

    void applyTransformation(const LinearTransformation& transformation);
    void applyTransformation(const AffineTransformation& transformation);
};

} // Rendering3D
} // Impact
