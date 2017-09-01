#pragma once
#include <string>

namespace Rendering3D {

class Color {

public:
    float r, g, b;

    Color();
    Color(float new_r, float new_g, float new_b);
    Color(int hex_value);
    
    static Color black();
    static Color white();
    static Color grey(float luminance);
    static Color red();
    static Color green();
    static Color blue();
    
    Color  operator+ (const Color& other) const;
    Color  operator- (const Color& other) const;
    Color& operator+= (const Color& other);
    Color& operator-= (const Color& other);
    Color  operator* (float factor) const;
    Color  operator/ (float divisor) const;
    Color  operator* (const Color& other) const;
    Color  operator/ (const Color& other) const;
    Color& operator*= (float factor);
    Color& operator/= (float divisor);
    Color& operator*= (const Color& other);
    Color& operator/= (const Color& other);

    Color& clamp();
    
    std::string toString() const;
};

typedef Color Power;
typedef Color Radiance;
typedef Color Biradiance;
typedef Color Reflectance;

Color operator*(float factor, const Color& color);

} // Rendering3D
