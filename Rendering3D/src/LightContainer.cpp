#include "LightContainer.hpp"

namespace Impact {
namespace Rendering3D {

LightContainer::LightContainer() {}

LightContainer::LightContainer(const LightContainer& other)
{
    for (imp_uint i = 0; i < other._lights.size(); i++)
    {
        if (RectangularAreaLight* light_ptr = dynamic_cast<RectangularAreaLight*>(other._lights[i]))
            _lights.push_back(new RectangularAreaLight(*light_ptr));
        else if (HemisphereAreaLight* light_ptr = dynamic_cast<HemisphereAreaLight*>(other._lights[i]))
            _lights.push_back(new HemisphereAreaLight(*light_ptr));
        else if (OmnidirectionalLight* light_ptr = dynamic_cast<OmnidirectionalLight*>(other._lights[i]))
            _lights.push_back(new OmnidirectionalLight(*light_ptr));
        else if (DirectionalLight* light_ptr = dynamic_cast<DirectionalLight*>(other._lights[i]))
            _lights.push_back(new DirectionalLight(*light_ptr));
    }
}

LightContainer::~LightContainer()
{
    while (!_lights.empty())
    {
        delete _lights.back();
        _lights.pop_back();
    }
}

LightContainer& LightContainer::operator=(LightContainer other)
{
	LightContainer::_swap(*this, other);
    return *this;
}

void LightContainer::_swap(LightContainer& first, LightContainer& second)
{
    std::swap(first._lights, second._lights);
}

LightContainer& LightContainer::addLight(const RectangularAreaLight& light)
{
    _lights.push_back(new RectangularAreaLight(light));
    return *this;
}

LightContainer& LightContainer::addLight(const HemisphereAreaLight& light)
{
    _lights.push_back(new HemisphereAreaLight(light));
    return *this;
}

LightContainer& LightContainer::addLight(const OmnidirectionalLight& light)
{
    _lights.push_back(new OmnidirectionalLight(light));
    return *this;
}

LightContainer& LightContainer::addLight(const DirectionalLight& light)
{
    _lights.push_back(new DirectionalLight(light));
    return *this;
}

imp_uint LightContainer::getNumberOfLights() const
{
    return static_cast<imp_uint>(_lights.size());
}

const Light* LightContainer::getLight(imp_uint idx) const
{
    return _lights[idx];
}

const std::vector<Light*>& LightContainer::getLights() const
{
    return _lights;
}

void LightContainer::applyTransformation(const LinearTransformation& transformation)
{
    imp_uint n_lights = getNumberOfLights();
    for (imp_uint i = 0; i < n_lights; i++)
    {
        _lights[i]->applyTransformation(transformation);
    }
}

void LightContainer::applyTransformation(const AffineTransformation& transformation)
{
    imp_uint n_lights = getNumberOfLights();
    for (imp_uint i = 0; i < n_lights; i++)
    {
        _lights[i]->applyTransformation(transformation);
    }
}

} // Rendering3D
} // Impact
