#pragma once
#include "precision.hpp"
#include "AreaLight.hpp"
#include "Color.hpp"
#include "SurfaceElement.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "CoordinateFrame.hpp"
#include "TriangleMesh.hpp"
#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"

namespace Impact {
namespace Rendering3D {

class HemisphereAreaLight : public AreaLight {
    
private:
    typedef Geometry3D::Point Point;
    typedef Geometry3D::Vector Vector;
    typedef Geometry3D::CoordinateFrame CoordinateFrame;
    typedef Geometry3D::TriangleMesh TriangleMesh;
    typedef Geometry3D::LinearTransformation LinearTransformation;
    typedef Geometry3D::AffineTransformation AffineTransformation;
    
    Point _center;
    Vector _direction;
    imp_float _radius;
    Power _power;
    imp_uint _n_samples;
    LinearTransformation _rotation;

public:
    HemisphereAreaLight(const Point& new_center,
                        const Vector& new_direction,
                        imp_float new_radius,
                        const Power& new_power,
                        imp_uint new_n_samples);

    CoordinateFrame getCoordinateFrame() const;
    imp_float getSurfaceArea() const;
    imp_uint getNumberOfSamples() const;
	virtual TriangleMesh getMesh() const = 0;

    Point getRandomPoint() const;

    SurfaceElement getRandomSurfaceElement() const;

    Power getTotalPower() const;

	void setCoordinateFrame(const CoordinateFrame& cframe);

    void applyTransformation(const LinearTransformation& transformation);
    void applyTransformation(const AffineTransformation& transformation);
};

} // Rendering3D
} // Impact
