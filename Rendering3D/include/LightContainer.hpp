#pragma once
#include "Light.hpp"
#include "RectangularAreaLight.hpp"
#include "HemisphereAreaLight.hpp"
#include "OmnidirectionalLight.hpp"
#include "DirectionalLight.hpp"
#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"
#include <vector>

namespace Impact {
namespace Rendering3D {

class LightContainer {

private:
    typedef Geometry3D::LinearTransformation LinearTransformation;
    typedef Geometry3D::AffineTransformation AffineTransformation;

    std::vector<Light*> _lights;
    
    static void _swap(LightContainer& first, LightContainer& second);

public:
    LightContainer();

    LightContainer(const LightContainer& other);
    ~LightContainer();
    LightContainer& operator=(LightContainer other);

    LightContainer& addLight(const RectangularAreaLight& light);
    LightContainer& addLight(const HemisphereAreaLight& light);
    LightContainer& addLight(const OmnidirectionalLight& light);
    LightContainer& addLight(const DirectionalLight& light);

    imp_uint getNumberOfLights() const;
    const Light* getLight(imp_uint idx) const;
    const std::vector<Light*>& getLights() const;

    void applyTransformation(const LinearTransformation& transformation);
    void applyTransformation(const AffineTransformation& transformation);
};

} // Rendering3D
} // Impact
