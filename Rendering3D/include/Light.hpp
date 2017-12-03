#pragma once
#include "Color.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Vector4.hpp"
#include "CoordinateFrame.hpp"
#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"

namespace Impact {
namespace Rendering3D {

class Light {

private:
    typedef Geometry3D::Point Point;
    typedef Geometry3D::Vector Vector;
    typedef Geometry3D::Vector4 Vector4;
    typedef Geometry3D::CoordinateFrame CoordinateFrame;
    typedef Geometry3D::LinearTransformation LinearTransformation;
    typedef Geometry3D::AffineTransformation AffineTransformation;

public:
	bool creates_shadows = true;
	Radiance ambient_radiance = Radiance::black();

    virtual CoordinateFrame getCoordinateFrame() const = 0;
    virtual imp_float getSurfaceArea() const = 0;
    virtual imp_uint getNumberOfSamples() const = 0;

    virtual Vector4 getRandomPoint() const = 0;
    virtual Biradiance getBiradiance(const Vector4& source_point,
                                     const Point& surface_point,
									 imp_float distance) const = 0;

    virtual Power getTotalPower() const = 0;
    //virtual Color getEmittedPhoton(Point& photon_origin,
    //                               Vector& photon_direction) const = 0;

	virtual void setCoordinateFrame(const CoordinateFrame& cframe) = 0;

    virtual void applyTransformation(const LinearTransformation& transformation) = 0;
    virtual void applyTransformation(const AffineTransformation& transformation) = 0;
};

} // Rendering3D
} // Impact
