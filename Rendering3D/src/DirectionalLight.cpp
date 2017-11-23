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
                           Vector::zero(),
                           Vector::zero(),
                           Vector::zero());
}

imp_float DirectionalLight::getSurfaceArea() const
{
    return 0;
}

imp_uint DirectionalLight::getNumberOfSamples() const
{
    return 1;
}

Geometry3D::Vector4 DirectionalLight::getRandomPoint() const
{
    return -_direction.toVector4();
}

Biradiance DirectionalLight::getBiradiance(const Vector4& source_point,
                                           const Point& surface_point) const
{
    assert(source_point.w == 0);
    return _biradiance;
}

Power DirectionalLight::getTotalPower() const
{
    return Power(IMP_FLOAT_INF, IMP_FLOAT_INF, IMP_FLOAT_INF);
}

void DirectionalLight::applyTransformation(const LinearTransformation& transformation)
{
    _direction = (transformation*_direction).normalize();
}

void DirectionalLight::applyTransformation(const AffineTransformation& transformation)
{
    _direction = (transformation*_direction).normalize();
}

} // Rendering3D
} // Impact
