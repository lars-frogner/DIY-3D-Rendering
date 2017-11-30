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
    
    Color  operator+ (const Color& other) const;
    Color  operator- (const Color& other) const;
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
    
    imp_float getMin() const;
    imp_float getMax() const;
    
    std::string toString() const;
};

Color operator*(imp_float factor, const Color& color);

typedef Color Power;
typedef Color Radiance;
typedef Color Biradiance;
typedef Color Reflectance;

} // Rendering3D
} // Impact
