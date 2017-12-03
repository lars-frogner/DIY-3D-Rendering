#pragma once
#include "precision.hpp"
#include "Material.hpp"
#include "Color.hpp"
#include "Vector3.hpp"

namespace Impact {
namespace Rendering3D {

class BlinnPhongMaterial : public Material {
    
private:
    typedef Geometry3D::Vector Vector;

    Reflectance _diffuse_reflectance;
    Reflectance _glossy_reflectance;
    float _smoothness;
    std::string _name;

    Color _diffuse_scattering_density;
    Color _glossy_scattering_density_factor;

    static imp_float _shininessToSmoothness(imp_float shininess);
    static imp_float _smoothnessToShininess(imp_float smoothness);
    void _initialize();

public:
    
    BlinnPhongMaterial();
    BlinnPhongMaterial(const Reflectance& new_diffuse_reflectance,
                       const Reflectance& new_glossy_reflectance,
                       imp_float new_smoothness,
                       const std::string& new_name = std::string());
    BlinnPhongMaterial(const Color& color,
                       imp_float reflectance,
                       imp_float glossiness,
                       imp_float new_shininess,
                       const std::string& new_name = std::string());
    BlinnPhongMaterial(const BlinnPhongMaterial& other);

    const Reflectance& getDiffuseReflectance() const;
    const Reflectance& getGlossyReflectance() const;
    float getShininess() const;
    const std::string& getName() const;

    void setName(const std::string& name);

	const Color& getBaseColor() const;

    BlinnPhongMaterial& setDiffuseReflectance(const Reflectance& new_diffuse_reflectance);
    BlinnPhongMaterial& setGlossyReflectance(const Reflectance& new_glossy_reflectance);
    BlinnPhongMaterial& setSmoothness(imp_float new_smoothness);
    BlinnPhongMaterial& setShininess(imp_float new_shininess);

    Color getScatteringDensity(const Vector& surface_normal,
                               const Vector& direction_to_source,
                               const Vector& scatter_direction) const;
};

} // Rendering3D
} // Impact
