#pragma once
#include "../Rendering3D/Color.hpp"

namespace Geometry3D {

template <typename F>
class Vertex {

private:
    typedef Rendering3D::Color Color;

public:
    F x, y, z;
    Color color;

    Vertex<F>();
    Vertex<F>(F new_x, F new_y, F new_z);
    Vertex<F>(F new_x, F new_y, F new_z,
              const Color& new_color);
    Vertex<F>(F new_x, F new_y, F new_z,
              float red, float green, float blue);
};

template <typename F>
Vertex<F>::Vertex() : x(0), y(0), z(0), color() {}

template <typename F>
Vertex<F>::Vertex(F new_x, F new_y, F new_z)
    : x(new_x), y(new_y), z(new_z), color() {}

template <typename F>
Vertex<F>::Vertex(F new_x, F new_y, F new_z,
                  const Color& new_color)
    : x(new_x), y(new_y), z(new_z), color(new_color) {}

template <typename F>
Vertex<F>::Vertex(F new_x, F new_y, F new_z,
                  float red, float green, float blue)
    : x(new_x), y(new_y), z(new_z), color(red, green, blue) {}

} // Geometry3D
