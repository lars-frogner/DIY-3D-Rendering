#pragma once
#define _USE_MATH_DEFINES
#include <assert.h>
#include <limits>
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
class DirectionalLight : public Light<F> {
    
private:
    typedef Geometry3D::Point<F> Point;
    typedef Geometry3D::Vector<F> Vector;
    typedef Geometry3D::Vector4<F> Vector4;
    typedef Geometry3D::CoordinateFrame<F> CoordinateFrame;
    typedef Transformations3D::LinearTransformation<F> LinearTransformation;
    typedef Transformations3D::AffineTransformation<F> AffineTransformation;
    
    const float _FOUR_PI = 4*static_cast<float>(M_PI);
    const F _INFINITY = std::numeric_limits<F>::infinity();
    
    Vector _direction;
    Biradiance _biradiance;

public:
    DirectionalLight<F>(const Vector& new_direction,
                        const Biradiance& new_biradiance);

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
DirectionalLight<F>::DirectionalLight(const Vector& new_direction,
                                      const Biradiance& new_biradiance)
    : _direction(new_direction.getNormalized()),
      _biradiance(new_biradiance) {}

template <typename F>
Geometry3D::CoordinateFrame<F> DirectionalLight<F>::getCoordinateFrame() const
{
    return CoordinateFrame(Point::origin(),
                           Vector::zero(),
                           Vector::zero(),
                           Vector::zero());
}

template <typename F>
F DirectionalLight<F>::getSurfaceArea() const
{
    return 0;
}

template <typename F>
size_t DirectionalLight<F>::getNumberOfSamples() const
{
    return 1;
}

template <typename F>
Geometry3D::Vector4<F> DirectionalLight<F>::getRandomPoint() const
{
    return -_direction.toVector4();
}

template <typename F>
Biradiance DirectionalLight<F>::getBiradiance(const Vector4& source_point,
                                              const Point& surface_point) const
{
    assert(source_point.w == 0);
    return _biradiance;
}

template <typename F>
Power DirectionalLight<F>::getTotalPower() const
{
    return Power(_INFINITY, _INFINITY, _INFINITY);
}

template <typename F>
void DirectionalLight<F>::applyTransformation(const LinearTransformation& transformation)
{
    _direction = (transformation*_direction).normalize();
}

template <typename F>
void DirectionalLight<F>::applyTransformation(const AffineTransformation& transformation)
{
    _direction = (transformation*_direction).normalize();
}

} //Rendering3D
