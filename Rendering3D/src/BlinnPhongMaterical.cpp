#include "BlinnPhongMaterial.hpp"
#include <cmath>
#include <algorithm>

namespace Impact {
namespace Rendering3D {

BlinnPhongMaterial::BlinnPhongMaterial()
    : _diffuse_reflectance(Color::black()),
      _glossy_reflectance(Color::black()),
      _smoothness(0),
      _name()
{
    _initialize();
}

BlinnPhongMaterial::BlinnPhongMaterial(const Reflectance& new_diffuse_reflectance,
                                       const Reflectance& new_glossy_reflectance,
                                       imp_float new_smoothness,
                                       const std::string& new_name)
    : _diffuse_reflectance(new_diffuse_reflectance),
      _glossy_reflectance(new_glossy_reflectance),
      _smoothness(new_smoothness),
      _name(new_name)
{
    _initialize();
}

BlinnPhongMaterial::BlinnPhongMaterial(const Color& color,
                                       imp_float reflectance,
                                       imp_float glossiness,
                                       imp_float new_shininess,
                                       const std::string& new_name)
    : _smoothness(_shininessToSmoothness(new_shininess)),
      _name(new_name)
{
    assert(reflectance >= 0 && reflectance <= 1);
    assert(glossiness >= 0 && glossiness <= 1);

    const Color& normalized_color = reflectance*color/color.getMax();

    _diffuse_reflectance = normalized_color*(1 - glossiness);
    _glossy_reflectance = normalized_color*glossiness;

    _initialize();
}

BlinnPhongMaterial::BlinnPhongMaterial(const BlinnPhongMaterial& other)
    : _diffuse_reflectance(other._diffuse_reflectance),
      _glossy_reflectance(other._glossy_reflectance),
      _smoothness(other._smoothness),
      _name(other._name)
{
    _initialize();
}

const Color& BlinnPhongMaterial::getDiffuseReflectance() const
{
    return _diffuse_reflectance;
}

const Color& BlinnPhongMaterial::getGlossyReflectance() const
{
    return _glossy_reflectance;
}

float BlinnPhongMaterial::getShininess() const
{
    return _smoothnessToShininess(_smoothness);
}

const std::string& BlinnPhongMaterial::getName() const
{
    return _name;
}

void BlinnPhongMaterial::setName(const std::string& name)
{
    _name = name;
}

const Color& BlinnPhongMaterial::getBaseColor() const
{
	return _diffuse_reflectance;
}

BlinnPhongMaterial& BlinnPhongMaterial::setDiffuseReflectance(const Reflectance& new_diffuse_reflectance)
{
    _diffuse_reflectance = new_diffuse_reflectance;
    _initialize();
    return *this;
}

BlinnPhongMaterial& BlinnPhongMaterial::setGlossyReflectance(const Reflectance& new_glossy_reflectance)
{
    _glossy_reflectance = new_glossy_reflectance;
    _initialize();
    return *this;
}

BlinnPhongMaterial& BlinnPhongMaterial::setSmoothness(imp_float new_smoothness)
{
    _smoothness = new_smoothness;
    _initialize();
    return *this;
}

BlinnPhongMaterial& BlinnPhongMaterial::setShininess(imp_float new_shininess)
{
    _smoothness = _shininessToSmoothness(new_shininess);
    _initialize();
    return *this;
}

Color BlinnPhongMaterial::getScatteringDensity(const Vector& surface_normal,
                                               const Vector& direction_to_source,
                                               const Vector& scatter_direction) const
{
    if (direction_to_source.dot(surface_normal) <= 0 || scatter_direction.dot(surface_normal) <= 0)
    {
        // Light source is behind surface or reflected ray points into the surface
        return Color::black();
    }

    imp_float specular_alignment = (direction_to_source + scatter_direction).normalize().dot(surface_normal);

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

imp_float BlinnPhongMaterial::_shininessToSmoothness(imp_float shininess)
{
    return pow<imp_float>(8192, shininess);
}

imp_float BlinnPhongMaterial::_smoothnessToShininess(imp_float smoothness)
{
    return log(smoothness)/static_cast<imp_float>(log(8192));
}

void BlinnPhongMaterial::_initialize()
{
    const Reflectance& total_reflectance = _diffuse_reflectance + _glossy_reflectance;
    imp_float max_total_reflectance = std::max<imp_float>(total_reflectance.r, std::max(total_reflectance.g, total_reflectance.b));

    // Energy conservation: sum of reflectances cannot exceed one
    if (max_total_reflectance > 1.0f)
    {
        _diffuse_reflectance /= max_total_reflectance;
        _glossy_reflectance /= max_total_reflectance;
    }

    _diffuse_scattering_density = _diffuse_reflectance/IMP_PI;
    _glossy_scattering_density_factor = _glossy_reflectance*(8 + _smoothness)/(8*IMP_PI);
}

} // Rendering3D
} // Impact
