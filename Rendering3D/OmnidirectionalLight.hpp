#pragma once
#define _USE_MATH_DEFINES
#include <assert.h>
#include "../Geometry3D/Point.hpp"
#include "../Geometry3D/Vector.hpp"
#include "../Geometry3D/Vector4.hpp"
#include "../Geometry3D/CoordinateFrame.hpp"
#include "../Transformations3D/LinearTransformation.hpp"
#include "../Transformations3D/AffineTransformation.hpp"
#include "Color.hpp"
#include "Light.hpp"

namespace Rendering3D {

template <typename F>
class OmnidirectionalLight : public Light<F> {
    
private:
    typedef Geometry3D::Point<F> Point;
    typedef Geometry3D::Vector<F> Vector;
    typedef Geometry3D::Vector4<F> Vector4;
    typedef Geometry3D::CoordinateFrame<F> CoordinateFrame;
    typedef Transformations3D::LinearTransformation<F> LinearTransformation;
    typedef Transformations3D::AffineTransformation<F> AffineTransformation;
    
    const float _FOUR_PI = 4*static_cast<float>(M_PI);
    
    Point _origin;
    Power _power;

public:
    OmnidirectionalLight<F>(const Point& new_origin,
                            const Power& new_power);

    CoordinateFrame getCoordinateFrame() const;
    F getSurfaceArea() const;
    size_t getNumberOfSamples() const;

    Vector4 getRandomPoint() const;
    Biradiance getBiradiance(const Vector4& source_point,
                             const Point& surface_point) const;

    Power getTotalPower() const;

    void applyTransformation(const LinearTransformation& transformation);
    void applyTransformation(const AffineTransformation& transformation);
};

template <typename F>
OmnidirectionalLight<F>::OmnidirectionalLight(const Point& new_origin,
                                              const Power& new_power)
    : _origin(new_origin),
      _power(new_power) {}

template <typename F>
Geometry3D::CoordinateFrame<F> OmnidirectionalLight<F>::getCoordinateFrame() const
{
    return CoordinateFrame(_origin,
                           Vector::unitX(),
                           Vector::unitY(),
                           Vector::unitZ());
}

template <typename F>
F OmnidirectionalLight<F>::getSurfaceArea() const
{
    return 0;
}

template <typename F>
size_t OmnidirectionalLight<F>::getNumberOfSamples() const
{
    return 1;
}

template <typename F>
Geometry3D::Vector4<F> OmnidirectionalLight<F>::getRandomPoint() const
{
    return _origin.toVector4();
}

template <typename F>
Biradiance OmnidirectionalLight<F>::getBiradiance(const Vector4& source_point,
                                                  const Point& surface_point) const
{
    assert(source_point.w == 1);
    return _power/(_FOUR_PI*(surface_point - source_point.getXYZ()).getSquaredLength());
}

template <typename F>
Power OmnidirectionalLight<F>::getTotalPower() const
{
    return _power;
}

template <typename F>
void OmnidirectionalLight<F>::applyTransformation(const LinearTransformation& transformation)
{
    _origin = transformation*_origin;
}

template <typename F>
void OmnidirectionalLight<F>::applyTransformation(const AffineTransformation& transformation)
{
    _origin = transformation*_origin;
}

} //Rendering3D
