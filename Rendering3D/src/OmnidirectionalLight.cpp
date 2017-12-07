#include "OmnidirectionalLight.hpp"
#include <cassert>

namespace Impact {
namespace Rendering3D {

OmnidirectionalLight::OmnidirectionalLight(const Point& new_origin,
                                           const Power& new_power)
    : _origin(new_origin),
      _power(new_power) {}

Geometry3D::CoordinateFrame OmnidirectionalLight::getCoordinateFrame() const
{
    return CoordinateFrame(_origin,
                           Vector::unitX(),
                           Vector::unitY(),
                           Vector::unitZ());
}

Geometry3D::Point OmnidirectionalLight::getPosition() const
{
    return _origin;
}

Power OmnidirectionalLight::getTotalPower() const
{
    return _power;
}

void OmnidirectionalLight::setCoordinateFrame(const CoordinateFrame& cframe)
{
	_origin = cframe.origin;
}

void OmnidirectionalLight::applyTransformation(const LinearTransformation& transformation)
{
    _origin = transformation(_origin);
}

void OmnidirectionalLight::applyTransformation(const AffineTransformation& transformation)
{
    _origin = transformation(_origin);
}

} // Rendering3D
} // Impact
