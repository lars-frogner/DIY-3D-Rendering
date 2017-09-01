#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include "Color.hpp"
#include "../Geometry3D/Vector.hpp"
#include "Material.hpp"

namespace Rendering3D {

template <typename F>
class BlinnPhongMaterial : public Material<F> {
    
private:
    typedef Geometry3D::Vector<F> Vector;

    const float _PI = static_cast<float>(M_PI);

    Reflectance _diffuse_reflectance;
    Reflectance _glossy_reflectance;
    float _shininess;

    Color _diffuse_scattering_density;
    Color _glossy_scattering_density_factor;
    float _smoothness;

    void _initialize();

public:
    
    BlinnPhongMaterial<F>();
    BlinnPhongMaterial<F>(const Reflectance& new_diffuse_reflectance,
                          const Reflectance& new_glossy_reflectance,
                          float new_shininess);

    const Reflectance& getDiffuseReflectance() const;
    const Reflectance& getGlossyReflectance() const;
    float getShininess() const;

    BlinnPhongMaterial<F>& setDiffuseReflectance(const Reflectance& new_diffuse_reflectance);
    BlinnPhongMaterial<F>& setGlossyReflectance(const Reflectance& new_glossy_reflectance);
    BlinnPhongMaterial<F>& setShininess(float new_shininess);

    Color getScatteringDensity(const Vector& surface_normal,
                               const Vector& direction_to_source,
                               const Vector& scatter_direction) const;
};

template <typename F>
BlinnPhongMaterial<F>::BlinnPhongMaterial()
    : _diffuse_reflectance(Color::black()),
      _glossy_reflectance(Color::black()),
      _shininess(0)
{
    _initialize();
}

template <typename F>
BlinnPhongMaterial<F>::BlinnPhongMaterial(const Reflectance& new_diffuse_reflectance,
                                          const Reflectance& new_glossy_reflectance,
                                          float new_shininess)
    : _diffuse_reflectance(new_diffuse_reflectance),
      _glossy_reflectance(new_glossy_reflectance),
      _shininess(new_shininess)
{
    _initialize();
}

template <typename F>
const Color& BlinnPhongMaterial<F>::getDiffuseReflectance() const
{
    return _diffuse_reflectance;
}

template <typename F>
const Color& BlinnPhongMaterial<F>::getGlossyReflectance() const
{
    return _glossy_reflectance;
}

template <typename F>
float BlinnPhongMaterial<F>::getShininess() const
{
    return _shininess;
}

template <typename F>
BlinnPhongMaterial<F>& BlinnPhongMaterial<F>::setDiffuseReflectance(const Reflectance& new_diffuse_reflectance)
{
    _diffuse_reflectance = new_diffuse_reflectance;
    _initialize();
    return *this;
}

template <typename F>
BlinnPhongMaterial<F>& BlinnPhongMaterial<F>::setGlossyReflectance(const Reflectance& new_glossy_reflectance)
{
    _glossy_reflectance = new_glossy_reflectance;
    _initialize();
    return *this;
}

template <typename F>
BlinnPhongMaterial<F>& BlinnPhongMaterial<F>::setShininess(float new_shininess)
{
    _shininess = new_shininess;
    _initialize();
    return *this;
}

template <typename F>
Color BlinnPhongMaterial<F>::getScatteringDensity(const Vector& surface_normal,
                                                  const Vector& direction_to_source,
                                                  const Vector& scatter_direction) const
{
    if (direction_to_source.dot(surface_normal) <= 0 || scatter_direction.dot(surface_normal) <= 0)
    {
        // Light source is behind surface or reflected ray points into the surface
        return Color::black();
    }

    F specular_alignment = (direction_to_source + scatter_direction).normalize().dot(surface_normal);

    if (specular_alignment <= 0)
    {
        return _diffuse_scattering_density;
    }
    else
    {
        return _diffuse_scattering_density +
               _glossy_scattering_density_factor*pow(specular_alignment, _smoothness);
    }
}

template <typename F>
void BlinnPhongMaterial<F>::_initialize()
{
    const Reflectance& total_reflectance = _diffuse_reflectance + _glossy_reflectance;
    float max_total_reflectance = std::max(total_reflectance.r, std::max(total_reflectance.g, total_reflectance.b));

    // Energy conservation: sum of reflectances cannot exceed one
    if (max_total_reflectance > 1.0f)
    {
        _diffuse_reflectance /= max_total_reflectance;
        _glossy_reflectance /= max_total_reflectance;
    }
    
     _smoothness = static_cast<float>(pow(8192.0, _shininess));

    _diffuse_scattering_density = _diffuse_reflectance/_PI;
    _glossy_scattering_density_factor = _glossy_reflectance*(8 + _smoothness)/(8*_PI);
}

} // Rendering3D
