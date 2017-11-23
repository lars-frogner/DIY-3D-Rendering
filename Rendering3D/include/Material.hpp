#pragma once
#include "Color.hpp"
#include "Vector3.hpp"
#include <string>

namespace Impact {
namespace Rendering3D {

class Material {
    
private:
    typedef Geometry3D::Vector Vector;

public:
    virtual const std::string& getName() const = 0;
    virtual void setName(const std::string& name) = 0;

    virtual Color getScatteringDensity(const Vector& surface_normal,
                                       const Vector& direction_to_source,
                                       const Vector& scatter_direction) const = 0;
};

} // Rendering3D
} // Impact
