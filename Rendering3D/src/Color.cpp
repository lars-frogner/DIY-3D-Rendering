#include "Color.hpp"
#include <cassert>
#include <algorithm>
#include <sstream>

namespace Impact {
namespace Rendering3D {

Color::Color() : r(0), g(0), b(0) {}

Color::Color(imp_float new_r, imp_float new_g, imp_float new_b)
    : r(new_r), g(new_g), b(new_b) {}

Color::Color(imp_int hex_value)
    : r(((hex_value >> 16) & 0xFF)/255.0f),
      g(((hex_value >> 8) & 0xFF)/255.0f),
      b(((hex_value) & 0xFF)/255.0f) {}

Color Color::black()
{
    return grey(0);
}

Color Color::white()
{
    return grey(1);
}

Color Color::grey(imp_float luminance)
{
    return Color(luminance, luminance, luminance);
}

Color Color::red()
{
    return Color(1, 0, 0);
}

Color Color::green()
{
    return Color(0, 1, 0);
}

Color Color::blue()
{
    return Color(0, 0, 1);
}

Color Color::yellow()
{
    return Color(1, 1, 0);
}

Color Color::cyan()
{
    return Color(0, 1, 1);
}

Color Color::magenta()
{
    return Color(1, 0, 1);
}

Color Color::orange()
{
    return Color(1, 0.647f, 0);
}

Color Color::maroon()
{
    return Color(0.5f, 0, 0);
}

Color Color::pink()
{
    return Color(1, 0.753f, 0.796f);
}

Color Color::purple()
{
    return Color(0.5f, 0, 0.5f);
}

Color Color::gold()
{
    return Color(1, 0.843f, 0);
}

Color Color::operator+(const Color& other) const
{
    return Color(r + other.r, g + other.g, b + other.b);
}

Color Color::operator-(const Color& other) const
{
    return Color(r - other.r, g - other.g, b - other.b);
}

Color& Color::operator+=(const Color& other)
{
    r += other.r;
    g += other.g;
    b += other.b;

    return *this;
}

Color& Color::operator-=(const Color& other)
{
    r -= other.r;
    g -= other.g;
    b -= other.b;

    return *this;
}

Color Color::operator*(imp_float factor) const
{
    return Color(r*factor, g*factor, b*factor);
}

Color Color::operator/(imp_float divisor) const
{
    return (*this)*(1/divisor);
}

Color Color::operator*(const Color& other) const
{
    return Color(r*other.r, g*other.g, b*other.b);
}

Color Color::operator/(const Color& other) const
{
    assert(other.r != 0 && other.g != 0 && other.b != 0);
    return Color(r/other.r, g/other.g, b/other.b);
}

Color& Color::operator*=(imp_float factor)
{
    r *= factor;
    g *= factor;
    b *= factor;

    return *this;
}

Color& Color::operator/=(imp_float divisor)
{
    return *this *= (1/divisor);
}

Color& Color::operator*=(const Color& other)
{
    r *= other.r;
    g *= other.g;
    b *= other.b;

    return *this;
}

Color& Color::operator/=(const Color& other)
{
    assert(other.r != 0 && other.g != 0 && other.b != 0);

    r /= other.r;
    g /= other.g;
    b /= other.b;

    return *this;
}

Color& Color::clamp()
{
    if      (r > 1) r = 1;
    else if (r < 0) r = 0;
    if      (g > 1) g = 1;
    else if (g < 0) g = 0;
    if      (b > 1) b = 1;
    else if (b < 0) b = 0;

    return *this;
}

float Color::getMin() const
{
    return std::min(r, std::min(g, b));
}

float Color::getMax() const
{
    return std::max(r, std::max(g, b));
}

Color operator*(imp_float factor, const Color& color)
{
    return color*factor;
}

std::string Color::toString() const
{
    std::ostringstream string_stream;
    string_stream << "(" << r << ", " << g << ", " << b << ")";
    return string_stream.str();
}

} // Rendering3D
} // Impact
