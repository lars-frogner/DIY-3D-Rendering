#include "HemisphereAreaLight.hpp"
#include "math_util.hpp"
#include <cassert>

namespace Impact {
namespace Rendering3D {

HemisphereAreaLight::HemisphereAreaLight(const Point& new_center,
                                         const Vector& new_direction,
                                         imp_float new_radius,
                                         const Power& new_power,
                                         imp_uint new_n_samples)
    : _center(new_center),
      _direction(new_direction.getNormalized()),
      _radius(new_radius),
      _power(new_power),
      _n_samples(new_n_samples),
      _rotation(LinearTransformation::rotationFromVectorToVector(Vector::unitZ(), -new_direction)) {}

Geometry3D::CoordinateFrame HemisphereAreaLight::getCoordinateFrame() const
{
    const Vector& basis_3 = -_direction;
    const Vector& basis_1 = Vector::unitY().getUnitNormalWith(basis_3);
    const Vector& basis_2 = basis_3.getUnitNormalWith(basis_1);

    return CoordinateFrame(_center, basis_1, basis_2, basis_3);
}

imp_float HemisphereAreaLight::getSurfaceArea() const
{
    return 2*IMP_PI*_radius*_radius;
}

imp_uint HemisphereAreaLight::getNumberOfSamples() const
{
    return _n_samples;
}

Geometry3D::Point HemisphereAreaLight::getRandomPoint() const
{
    imp_float num = 1 - math_util::random();

    imp_float r_cos_latitude = _radius*sqrt(1 - num*num);
    imp_float longitude = math_util::random()*IMP_TWO_PI;

    Point point(_center.x + r_cos_latitude*cos(longitude),
                _center.y + r_cos_latitude*sin(longitude),
                _center.z + _radius*num);

    return _rotation(point);
}

SurfaceElement HemisphereAreaLight::getRandomSurfaceElement() const
{
	SurfaceElement surface_element;

	surface_element.geometric.position = getRandomPoint();
	surface_element.geometric.normal = (surface_element.geometric.position - _center).getNormalized();

	return surface_element;
}

Power HemisphereAreaLight::getTotalPower() const
{
    return _power;
}

void HemisphereAreaLight::setCoordinateFrame(const CoordinateFrame& cframe)
{
	_center = cframe.origin;
	_direction = -cframe.basis_1;
}

void HemisphereAreaLight::applyTransformation(const LinearTransformation& transformation)
{
    _center = transformation(_center);
    _direction = transformation(_direction).normalize();
    _rotation = LinearTransformation::rotationFromVectorToVector(Vector::unitZ(), -_direction);
}

void HemisphereAreaLight::applyTransformation(const AffineTransformation& transformation)
{
    _center = transformation(_center);
    _direction = transformation(_direction).normalize();
    _rotation = LinearTransformation::rotationFromVectorToVector(Vector::unitZ(), -_direction);
}

} // Rendering3D
} // Impact
