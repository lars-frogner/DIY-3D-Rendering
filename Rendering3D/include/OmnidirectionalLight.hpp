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

class OmnidirectionalLight {
    
private:
    typedef Geometry3D::Point Point;
    typedef Geometry3D::Vector Vector;
    typedef Geometry3D::CoordinateFrame CoordinateFrame;
    typedef Geometry3D::LinearTransformation LinearTransformation;
    typedef Geometry3D::AffineTransformation AffineTransformation;
    
    Point _origin;
    Power _power;

public:
	bool creates_shadows = true;
	Radiance ambient_radiance = Radiance::black();

    OmnidirectionalLight(const Point& new_origin,
                         const Power& new_power);

    CoordinateFrame getCoordinateFrame() const;

    Point getPosition() const;

    Power getTotalPower() const;

	void setCoordinateFrame(const CoordinateFrame& cframe);

    void applyTransformation(const LinearTransformation& transformation);
    void applyTransformation(const AffineTransformation& transformation);
};

} // Rendering3D
} // Impact
