#pragma once
#include "../Rendering3D/Color.hpp"

namespace Geometry2D {

template <typename F>
class Vertex {

private:
    typedef Rendering3D::Color Color;

public:
    F x, y;
    Color color;

    Vertex<F>();
    Vertex<F>(F new_x, F new_y);
    Vertex<F>(F new_x, F new_y,
              const Color& new_color);
    Vertex<F>(F new_x, F new_y,
              float red, float green, float blue);
};

template <typename F>
Vertex<F>::Vertex() : x(0), y(0), color() {}

template <typename F>
Vertex<F>::Vertex(F new_x, F new_y)
    : x(new_x), y(new_y), color() {}

template <typename F>
Vertex<F>::Vertex(F new_x, F new_y,
                  const Color& new_color)
    : x(new_x), y(new_y), color(new_color) {}

template <typename F>
Vertex<F>::Vertex(F new_x, F new_y,
                  float red, float green, float blue)
    : x(new_x), y(new_y), color(red, green, blue) {}

} // Geometry2D
