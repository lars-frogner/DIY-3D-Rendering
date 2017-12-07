#include "DirectionalLight.hpp"
#include <cassert>

namespace Impact {
namespace Rendering3D {

DirectionalLight::DirectionalLight(const Vector& new_direction,
                                   const Biradiance& new_biradiance)
    : _direction(new_direction.getNormalized()),
      _biradiance(new_biradiance) {}

Geometry3D::CoordinateFrame DirectionalLight::getCoordinateFrame() const
{
    return CoordinateFrame(Point::origin(),
                           _direction,
                           Vector::zero(),
                           Vector::zero());
}

const Geometry3D::Vector& DirectionalLight::getDirection() const
{
    return _direction;
}

const Biradiance& DirectionalLight::getBiradiance() const
{
    return _biradiance;
}

void DirectionalLight::setCoordinateFrame(const CoordinateFrame& cframe)
{
	_direction = cframe.basis_1;
}

void DirectionalLight::applyTransformation(const LinearTransformation& transformation)
{
    _direction = transformation(_direction).normalize();
}

void DirectionalLight::applyTransformation(const AffineTransformation& transformation)
{
    _direction = transformation(_direction).normalize();
}

} // Rendering3D
} // Impact
