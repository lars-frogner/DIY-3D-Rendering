#pragma once
#include "precision.hpp"
#include <string>

namespace Impact {
namespace Rendering3D {

class Color {

private:
	imp_float _pad; // Extra variable to achieve alignment in memory

public:
    imp_float r, g, b;

    Color();
    Color(imp_float new_r, imp_float new_g, imp_float new_b);
    Color(imp_int hex_value);
    
    static Color black();
    static Color white();
    static Color grey(imp_float luminance);
    static Color red();
    static Color green();
    static Color blue();
    static Color yellow();
    static Color cyan();
    static Color magenta();
    static Color orange();
    static Color maroon();
    static Color pink();
    static Color purple();
    static Color gold();
	static Color crimson();
	static Color forestgreen();
	static Color navy();
    
    Color  operator+ (const Color& other) const;
    Color  operator+ (imp_float value) const;
    Color  operator- () const;
    Color  operator- (const Color& other) const;
    Color  operator- (imp_float value) const;
    Color& operator+= (const Color& other);
    Color& operator-= (const Color& other);
    Color  operator* (imp_float factor) const;
    Color  operator/ (imp_float divisor) const;
    Color  operator* (const Color& other) const;
    Color  operator/ (const Color& other) const;
    Color& operator*= (imp_float factor);
    Color& operator/= (imp_float divisor);
    Color& operator*= (const Color& other);
    Color& operator/= (const Color& other);

    Color& clamp();

	Color& normalize();
    
	imp_float getComponent(imp_uint component) const;
    imp_float getMin() const;
    imp_float getMax() const;
    imp_float getMax(imp_uint& max_component) const;
	imp_float getMean() const;
	imp_float getTotal() const;
	Color getAbsolute() const;

	bool nonZero() const;
    
    std::string toString() const;
};

Color operator*(imp_float factor, const Color& color);
Color operator+(imp_float term, const Color& color);
Color operator-(imp_float term, const Color& color);

typedef Color Power;
typedef Color Radiance;
typedef Color Biradiance;
typedef Color Reflectance;

} // Rendering3D
} // Impact
