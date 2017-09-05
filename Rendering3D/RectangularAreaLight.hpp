#pragma once
#define _USE_MATH_DEFINES
#include <algorithm>
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
class RectangularAreaLight : public Light<F> {
    
private:
    typedef Geometry3D::Point<F> Point;
    typedef Geometry3D::Vector<F> Vector;
    typedef Geometry3D::Vector4<F> Vector4;
    typedef Geometry3D::CoordinateFrame<F> CoordinateFrame;
    typedef Transformations3D::LinearTransformation<F> LinearTransformation;
    typedef Transformations3D::AffineTransformation<F> AffineTransformation;
    
    const float _PI = static_cast<float>(M_PI);
    float const _RAND_NORM = 1/static_cast<float>(RAND_MAX);
    
    Point _origin;
    Vector _width_vector, _height_vector, _direction;
    Power _power;
    F _width, _height;
    size_t _n_samples;

public:
    RectangularAreaLight<F>(const Point& new_center,
                            const Vector& direction,
                            const Vector& new_width_vector,
                            F height,
                            const Power& new_power,
                            size_t new_n_samples);
    
    RectangularAreaLight<F>(const Point& new_center,
                            const Point& point_in_direction,
                            const Vector& new_width_vector,
                            F height,
                            const Power& new_power,
                            size_t new_n_samples);

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
RectangularAreaLight<F>::RectangularAreaLight(const Point& new_center,
                                              const Vector& new_direction,
                                              const Vector& new_width_vector,
                                              F height,
                                              const Power& new_power,
                                              size_t new_n_samples)
    : _width_vector(new_width_vector),
      _direction(new_direction.getNormalized()),
      _height_vector(height*new_direction.getUnitNormalWith(new_width_vector)),
      _width(new_width_vector.getLength()),
      _height(height),
      _power(new_power),
      _n_samples(new_n_samples)
{
    _origin = new_center - (_width_vector + _height_vector)/2;
}

template <typename F>
RectangularAreaLight<F>::RectangularAreaLight(const Point& new_center,
                                              const Point& point_in_direction,
                                              const Vector& new_width_vector,
                                              F height,
                                              const Power& new_power,
                                              size_t new_n_samples)
    : _width_vector(new_width_vector),
      _direction((point_in_direction - new_center).getNormalized()),
      _height_vector(height*(point_in_direction - new_center).getNormalized().getUnitNormalWith(new_width_vector)),
      _width(new_width_vector.getLength()),
      _height(height),
      _power(new_power),
      _n_samples(new_n_samples)
{
    _origin = new_center - (_width_vector + _height_vector)/2;
}

template <typename F>
Geometry3D::CoordinateFrame<F> RectangularAreaLight<F>::getCoordinateFrame() const
{
    return CoordinateFrame(_origin,
                           _width_vector,
                           _height_vector,
                           _direction);
}

template <typename F>
F RectangularAreaLight<F>::getSurfaceArea() const
{
    return _width*_height;
}

template <typename F>
size_t RectangularAreaLight<F>::getNumberOfSamples() const
{
    return _n_samples;
}

template <typename F>
Geometry3D::Vector4<F> RectangularAreaLight<F>::getRandomPoint() const
{
    return (_origin +
            (_RAND_NORM*rand())*_width_vector +
            (_RAND_NORM*rand())*_height_vector).toVector4();
}

template <typename F>
Biradiance RectangularAreaLight<F>::getBiradiance(const Vector4& source_point,
                                                  const Point& surface_point) const
{
    assert(source_point.w == 1);
    const Vector& ray_distance_vector = surface_point - source_point.getXYZ();
    F squared_distance = ray_distance_vector.getSquaredLength();

    return _power*std::max<F>(_direction.dot(ray_distance_vector/sqrt(squared_distance)), 0)
           /(_PI*squared_distance);
}

template <typename F>
Power RectangularAreaLight<F>::getTotalPower() const
{
    return _power;
}

template <typename F>
void RectangularAreaLight<F>::applyTransformation(const LinearTransformation& transformation)
{
    _origin = transformation*_origin;
    _width_vector = transformation*_width_vector;
    _height_vector = transformation*_height_vector;
    _direction = (transformation*_direction).normalize();
    _width = _width_vector.getLength();
    _height = _height_vector.getLength();
}

template <typename F>
void RectangularAreaLight<F>::applyTransformation(const AffineTransformation& transformation)
{
    _origin = transformation*_origin;
    _width_vector = transformation*_width_vector;
    _height_vector = transformation*_height_vector;
    _direction = (transformation*_direction).normalize();
    _width = _width_vector.getLength();
    _height = _height_vector.getLength();
}

} //Rendering3D
