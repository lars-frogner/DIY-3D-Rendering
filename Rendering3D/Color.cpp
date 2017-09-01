#include "Color.hpp"
#include <assert.h>
#include <algorithm>
#include <string>
#include <sstream>

namespace Rendering3D {

Color::Color() : r(0), g(0), b(0) {}

Color::Color(float new_r, float new_g, float new_b)
    : r(new_r), g(new_g), b(new_b) {}

Color::Color(int hex_value)
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

Color Color::grey(float luminance)
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

Color Color::operator*(float factor) const
{
    return Color(r*factor, g*factor, b*factor);
}

Color Color::operator/(float divisor) const
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

Color& Color::operator*=(float factor)
{
    r *= factor;
    g *= factor;
    b *= factor;

    return *this;
}

Color& Color::operator/=(float divisor)
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

Color operator*(float factor, const Color& color)
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