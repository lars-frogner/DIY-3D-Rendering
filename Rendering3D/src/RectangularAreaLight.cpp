#include "RectangularAreaLight.hpp"
#include "math_util.hpp"
#include <cassert>

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

Geometry3D::TriangleMesh RectangularAreaLight::getMesh() const
{
    return TriangleMesh::sheet(_origin + (_width_vector + _height_vector)/2, _direction, _width_vector, _height);
}

Geometry3D::Point RectangularAreaLight::getRandomPoint() const
{
    return (_origin +
            math_util::random()*_width_vector +
            math_util::random()*_height_vector);
}

SurfaceElement RectangularAreaLight::getRandomSurfaceElement() const
{
	SurfaceElement surface_element;

	surface_element.geometric.position = getRandomPoint();
	surface_element.geometric.normal = _direction;

	return surface_element;
}

Power RectangularAreaLight::getTotalPower() const
{
    return _power;
}

void RectangularAreaLight::setCoordinateFrame(const CoordinateFrame& cframe)
{
	_origin = cframe.origin;
	_width_vector = cframe.basis_1;
	_height_vector = cframe.basis_2;
	_direction = cframe.basis_3;
}

void RectangularAreaLight::applyTransformation(const LinearTransformation& transformation)
{
    _origin = transformation(_origin);
    _width_vector = transformation(_width_vector);
    _height_vector = transformation(_height_vector);
    _direction = transformation(_direction).normalize();
    _width = _width_vector.getLength();
    _height = _height_vector.getLength();
}

void RectangularAreaLight::applyTransformation(const AffineTransformation& transformation)
{
    _origin = transformation(_origin);
    _width_vector = transformation(_width_vector);
    _height_vector = transformation(_height_vector);
    _direction = transformation(_direction).normalize();
    _width = _width_vector.getLength();
    _height = _height_vector.getLength();
}

} // Rendering3D
} // Impact
