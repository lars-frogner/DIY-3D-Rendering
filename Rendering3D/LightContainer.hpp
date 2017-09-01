#pragma once
#include <vector>
#include "../Transformations3D/LinearTransformation.hpp"
#include "../Transformations3D/AffineTransformation.hpp"
#include "Light.hpp"
#include "RectangularAreaLight.hpp"
#include "HemisphereAreaLight.hpp"
#include "OmnidirectionalLight.hpp"
#include "DirectionalLight.hpp"

namespace Rendering3D {

template <typename F>
class LightContainer {

private:
    typedef Transformations3D::LinearTransformation<F> LinearTransformation;
    typedef Transformations3D::AffineTransformation<F> AffineTransformation;

    std::vector<Light<F>*> _lights;
    
    friend void _swap(LightContainer<F>& first, LightContainer<F>& second);

public:
    LightContainer<F>();

    LightContainer<F>(const LightContainer<F>& other);
    ~LightContainer<F>();
    LightContainer<F>& operator=(LightContainer<F> other);

    LightContainer<F>& addLight(const RectangularAreaLight<F>& light);
    LightContainer<F>& addLight(const HemisphereAreaLight<F>& light);
    LightContainer<F>& addLight(const OmnidirectionalLight<F>& light);
    LightContainer<F>& addLight(const DirectionalLight<F>& light);

    size_t getNumberOfLights() const;
    const Light<F>* getLight(size_t idx) const;
    const std::vector<Light<F>*>& getLights() const;

    void applyTransformation(const LinearTransformation& transformation);
    void applyTransformation(const AffineTransformation& transformation);
};

template <typename F>
LightContainer<F>::LightContainer() {}

template <typename F>
LightContainer<F>::LightContainer(const LightContainer<F>& other)
{
    for (size_t i = 0; i < other._lights.size(); i++)
    {
        if (RectangularAreaLight<F>* light_ptr = dynamic_cast<RectangularAreaLight<F>*>(other._lights[i]))
            _lights.push_back(new RectangularAreaLight<F>(*light_ptr));
        else if (HemisphereAreaLight<F>* light_ptr = dynamic_cast<HemisphereAreaLight<F>*>(other._lights[i]))
            _lights.push_back(new HemisphereAreaLight<F>(*light_ptr));
        else if (OmnidirectionalLight<F>* light_ptr = dynamic_cast<OmnidirectionalLight<F>*>(other._lights[i]))
            _lights.push_back(new OmnidirectionalLight<F>(*light_ptr));
        else if (DirectionalLight<F>* light_ptr = dynamic_cast<DirectionalLight<F>*>(other._lights[i]))
            _lights.push_back(new DirectionalLight<F>(*light_ptr));
    }
}

template <typename F>
LightContainer<F>::~LightContainer()
{
    while (!_lights.empty())
    {
        delete _lights.back();
        _lights.pop_back();
    }
}

template <typename F>
LightContainer<F>& LightContainer<F>::operator=(LightContainer<F> other)
{
    _swap(*this, other);
    return *this;
}

template <typename F>
void _swap(LightContainer<F>& first, LightContainer<F>& second)
{
    std::swap(first._lights, second._lights);
}

template <typename F>
LightContainer<F>& LightContainer<F>::addLight(const RectangularAreaLight<F>& light)
{
    _lights.push_back(new RectangularAreaLight<F>(light));
    return *this;
}

template <typename F>
LightContainer<F>& LightContainer<F>::addLight(const HemisphereAreaLight<F>& light)
{
    _lights.push_back(new HemisphereAreaLight<F>(light));
    return *this;
}

template <typename F>
LightContainer<F>& LightContainer<F>::addLight(const OmnidirectionalLight<F>& light)
{
    _lights.push_back(new OmnidirectionalLight<F>(light));
    return *this;
}

template <typename F>
LightContainer<F>& LightContainer<F>::addLight(const DirectionalLight<F>& light)
{
    _lights.push_back(new DirectionalLight<F>(light));
    return *this;
}

template <typename F>
size_t LightContainer<F>::getNumberOfLights() const
{
    return _lights.size();
}

template <typename F>
const Light<F>* LightContainer<F>::getLight(size_t idx) const
{
    return _lights[idx];
}

template <typename F>
const std::vector<Light<F>*>& LightContainer<F>::getLights() const
{
    return _lights;
}

template <typename F>
void LightContainer<F>::applyTransformation(const LinearTransformation& transformation)
{
    size_t n_lights = getNumberOfLights();
    for (size_t i = 0; i < n_lights; i++)
    {
        _lights[i]->applyTransformation(transformation);
    }
}

template <typename F>
void LightContainer<F>::applyTransformation(const AffineTransformation& transformation)
{
    size_t n_lights = getNumberOfLights();
    for (size_t i = 0; i < n_lights; i++)
    {
        _lights[i]->applyTransformation(transformation);
    }
}

} // Rendering3D
