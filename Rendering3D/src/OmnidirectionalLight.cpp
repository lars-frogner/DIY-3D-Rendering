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

imp_float OmnidirectionalLight::getSurfaceArea() const
{
    return 0;
}

imp_uint OmnidirectionalLight::getNumberOfSamples() const
{
    return 1;
}

Geometry3D::Vector4 OmnidirectionalLight::getRandomPoint() const
{
    return _origin.toVector4();
}

Biradiance OmnidirectionalLight::getBiradiance(const Vector4& source_point,
                                               const Point& surface_point) const
{
    assert(source_point.w == 1);
    return _power/(IMP_FOUR_PI*(surface_point - source_point.getXYZ()).getSquaredLength());
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
    _origin = transformation*_origin;
}

void OmnidirectionalLight::applyTransformation(const AffineTransformation& transformation)
{
    _origin = transformation*_origin;
}

} // Rendering3D
} // Impact
