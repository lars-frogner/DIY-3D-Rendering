#pragma once
#include "Color.hpp"
#include "../Geometry3D/Vector.hpp"

namespace Rendering3D {

template <typename F>
class Material {
    
private:
    typedef Geometry3D::Vector<F> Vector;

public:
    virtual Color getScatteringDensity(const Vector& surface_normal,
                                       const Vector& direction_to_source,
                                       const Vector& scatter_direction) const = 0;
};

} // Rendering3D
