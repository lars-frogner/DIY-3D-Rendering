#include "RectangularAreaLight.hpp"
#include <cassert>
#include <algorithm>

namespace Impact {
namespace Rendering3D {

RectangularAreaLight::RectangularAreaLight(const Point& new_center,
                                           const Vector& new_direction,
                                           const Vector& new_width_vector,
                                           imp_float height,
                                           const Power& new_power,
                                           imp_uint new_n_samples)
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

RectangularAreaLight::RectangularAreaLight(const Point& new_center,
                                           const Point& point_in_direction,
                                           const Vector& new_width_vector,
                                           imp_float height,
                                           const Power& new_power,
                                           imp_uint new_n_samples)
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

Geometry3D::CoordinateFrame RectangularAreaLight::getCoordinateFrame() const
{
    return CoordinateFrame(_origin,
                           _width_vector,
                           _height_vector,
                           _direction);
}

imp_float RectangularAreaLight::getSurfaceArea() const
{
    return _width*_height;
}

imp_uint RectangularAreaLight::getNumberOfSamples() const
{
    return _n_samples;
}

Geometry3D::Vector4 RectangularAreaLight::getRandomPoint() const
{
    return (_origin +
            (IMP_RAND_NORM*rand())*_width_vector +
            (IMP_RAND_NORM*rand())*_height_vector).toVector4();
}

Biradiance RectangularAreaLight::getBiradiance(const Vector4& source_point,
                                                  const Point& surface_point) const
{
    assert(source_point.w == 1);
    const Vector& ray_distance_vector = surface_point - source_point.getXYZ();
    imp_float squared_distance = ray_distance_vector.getSquaredLength();

    return _power*std::max<imp_float>(_direction.dot(ray_distance_vector/sqrt(squared_distance)), 0)
           /(IMP_PI*squared_distance);
}

Power RectangularAreaLight::getTotalPower() const
{
    return _power;
}

void RectangularAreaLight::applyTransformation(const LinearTransformation& transformation)
{
    _origin = transformation*_origin;
    _width_vector = transformation*_width_vector;
    _height_vector = transformation*_height_vector;
    _direction = (transformation*_direction).normalize();
    _width = _width_vector.getLength();
    _height = _height_vector.getLength();
}

void RectangularAreaLight::applyTransformation(const AffineTransformation& transformation)
{
    _origin = transformation*_origin;
    _width_vector = transformation*_width_vector;
    _height_vector = transformation*_height_vector;
    _direction = (transformation*_direction).normalize();
    _width = _width_vector.getLength();
    _height = _height_vector.getLength();
}

} // Rendering3D
} // Impact
