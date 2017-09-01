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
class HemisphereAreaLight : public Light<F> {
    
private:
    typedef Geometry3D::Point<F> Point;
    typedef Geometry3D::Vector<F> Vector;
    typedef Geometry3D::Vector4<F> Vector4;
    typedef Geometry3D::CoordinateFrame<F> CoordinateFrame;
    typedef Transformations3D::LinearTransformation<F> LinearTransformation;
    typedef Transformations3D::AffineTransformation<F> AffineTransformation;
    
    const float _PI = static_cast<float>(M_PI);
    float const _RAND_NORM = 1/static_cast<float>(RAND_MAX);
    
    Point _center;
    Vector _direction;
    F _radius;
    Power _power;
    size_t _n_samples;
    LinearTransformation _rotation;

public:
    HemisphereAreaLight<F>(const Point& new_center,
                           const Vector& new_direction,
                           F new_radius,
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
HemisphereAreaLight<F>::HemisphereAreaLight(const Point& new_center,
                                            const Vector& new_direction,
                                            F new_radius,
                                            const Power& new_power,
                                            size_t new_n_samples)
    : _center(new_center),
      _direction(new_direction.getNormalized()),
      _radius(new_radius),
      _power(new_power),
      _n_samples(new_n_samples),
      _rotation(LinearTransformation::rotationFromVectorToVector(Vector::unitZ(), -new_direction)) {}

template <typename F>
Geometry3D::CoordinateFrame<F> HemisphereAreaLight<F>::getCoordinateFrame() const
{
    const Vector& basis_3 = -_direction;
    const Vector& basis_1 = Vector::unitY().getUnitNormalWith(basis_3);
    const Vector& basis_2 = basis_3.getUnitNormalWith(basis_1);

    return CoordinateFrame(_center, basis_1, basis_2, basis_3);
}

template <typename F>
F HemisphereAreaLight<F>::getSurfaceArea() const
{
    return 2*_PI*_radius*_radius;
}

template <typename F>
size_t HemisphereAreaLight<F>::getNumberOfSamples() const
{
    return _n_samples;
}

template <typename F>
Geometry3D::Vector4<F> HemisphereAreaLight<F>::getRandomPoint() const
{
    F num = 1 - _RAND_NORM*rand();

    F r_cos_latitude = _radius*sqrt(1 - num*num);
    F longitude = 2*_PI*_RAND_NORM*rand();

    Point point(_center.x + r_cos_latitude*cos(longitude),
                _center.y + r_cos_latitude*sin(longitude),
                _center.z + _radius*num);

    return (_rotation*point).toVector4();
}

template <typename F>
Biradiance HemisphereAreaLight<F>::getBiradiance(const Vector4& source_point,
                                                 const Point& surface_point) const
{
    const Vector& ray_direction = (_center - source_point.getXYZ())/_radius;
    assert(source_point.w == 1 && abs(ray_direction.getLength() - 1.0) < 1.0e-5);
    const Vector& ray_distance_vector = surface_point - source_point.getXYZ();
    F squared_distance = ray_distance_vector.getSquaredLength();

    return _power*std::max<F>(ray_direction.dot(ray_distance_vector/sqrt(squared_distance)), 0)
           /(_PI*squared_distance);
}

template <typename F>
Power HemisphereAreaLight<F>::getTotalPower() const
{
    return _power;
}

template <typename F>
void HemisphereAreaLight<F>::applyTransformation(const LinearTransformation& transformation)
{
    _center = transformation*_center;
    _direction = (transformation*_direction).normalize();
    _rotation = LinearTransformation::rotationFromVectorToVector(Vector::unitZ(), -_direction);
}

template <typename F>
void HemisphereAreaLight<F>::applyTransformation(const AffineTransformation& transformation)
{
    _center = transformation*_center;
    _direction = (transformation*_direction).normalize();
    _rotation = LinearTransformation::rotationFromVectorToVector(Vector::unitZ(), -_direction);
}

} //Rendering3D
