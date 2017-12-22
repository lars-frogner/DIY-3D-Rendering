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

Color Color::crimson()
{
    return Color(220/255.0f, 20/255.0f, 60/255.0f);
}

Color Color::forestgreen()
{
    return Color(34/255.0f, 139/255.0f, 34/255.0f);
}

Color Color::navy()
{
    return Color(0, 0, 128/255.0f);
}

Color Color::operator+(const Color& other) const
{
    return Color(r + other.r, g + other.g, b + other.b);
}

Color Color::operator+(imp_float value) const
{
    return Color(r + value, g + value, b + value);
}

Color Color::operator-(const Color& other) const
{
    return Color(r - other.r, g - other.g, b - other.b);
}

Color Color::operator-(imp_float value) const
{
    return Color(r - value, g - value, b - value);
}

Color Color::operator-() const
{
    return Color(-r, -g, -b);
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
	assert(divisor != 0);

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
	assert(divisor != 0);

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

Color& Color::normalize()
{
	assert(r != 0 || g != 0 || b != 0);

	imp_float norm = 1/(r*r + g*g + b*b);

	r *= norm;
	g *= norm;
	b *= norm;

    return *this;
}

imp_float Color::getComponent(imp_uint component) const
{
	return (component == 0)? r : ((component == 1)? g : b);
}

imp_float Color::getMin() const
{
    return std::min(r, std::min(g, b));
}

imp_float Color::getMax() const
{
    return std::max(r, std::max(g, b));
}

imp_float Color::getMax(imp_uint& max_component) const
{
    if (r > g)
	{
		if (r > b)
		{
			max_component = 0;
			return r;
		}
		else
		{
			max_component = 2;
			return b;
		}
	}
	else
	{
		if (g > b)
		{
			max_component = 1;
			return g;
		}
		else
		{
			max_component = 2;
			return b;
		}
	}
}

imp_float Color::getMean() const
{
    return (r + g + b)/3;
}

imp_float Color::getTotal() const
{
    return r + g + b;
}

Color Color::getAbsolute() const
{
    return Color(abs(r), abs(g), abs(b));
}

bool Color::nonZero() const
{
    return r != 0 || g != 0 || b != 0;
}

Color operator*(imp_float factor, const Color& color)
{
    return color*factor;
}

Color operator+(imp_float value, const Color& color)
{
	return Color(value + color.r, value + color.g, value + color.b);
}

Color operator-(imp_float value, const Color& color)
{
	return Color(value - color.r, value - color.g, value - color.b);
}

std::string Color::toString() const
{
    std::ostringstream string_stream;
    string_stream << "(" << r << ", " << g << ", " << b << ")";
    return string_stream.str();
}

} // Rendering3D
} // Impact
