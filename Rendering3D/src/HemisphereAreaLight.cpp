#include "HemisphereAreaLight.hpp"
#include <cassert>
#include <algorithm>

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

Geometry3D::Vector4 HemisphereAreaLight::getRandomPoint() const
{
    imp_float num = 1 - IMP_RAND_NORM*rand();

    imp_float r_cos_latitude = _radius*sqrt(1 - num*num);
    imp_float longitude = 2*IMP_PI*IMP_RAND_NORM*rand();

    Point point(_center.x + r_cos_latitude*cos(longitude),
                _center.y + r_cos_latitude*sin(longitude),
                _center.z + _radius*num);

    return (_rotation*point).toVector4();
}

Biradiance HemisphereAreaLight::getBiradiance(const Vector4& source_point,
                                                 const Point& surface_point) const
{
    const Vector& ray_direction = (_center - source_point.getXYZ())/_radius;
    assert(source_point.w == 1 && abs(ray_direction.getLength() - 1.0) < 1.0e-5);
    const Vector& ray_distance_vector = surface_point - source_point.getXYZ();
    imp_float squared_distance = ray_distance_vector.getSquaredLength();

    return _power*std::max<imp_float>(ray_direction.dot(ray_distance_vector/sqrt(squared_distance)), 0)
           /(IMP_PI*squared_distance);
}

Power HemisphereAreaLight::getTotalPower() const
{
    return _power;
}

void HemisphereAreaLight::applyTransformation(const LinearTransformation& transformation)
{
    _center = transformation*_center;
    _direction = (transformation*_direction).normalize();
    _rotation = LinearTransformation::rotationFromVectorToVector(Vector::unitZ(), -_direction);
}

void HemisphereAreaLight::applyTransformation(const AffineTransformation& transformation)
{
    _center = transformation*_center;
    _direction = (transformation*_direction).normalize();
    _rotation = LinearTransformation::rotationFromVectorToVector(Vector::unitZ(), -_direction);
}

} // Rendering3D
} // Impact
