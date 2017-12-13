#include "material_assets.hpp"

namespace Impact {
namespace Rendering3D {

extern const Reflectance IMP_DIFFUSE_GLOSSY_REFLECTANCE = Reflectance::grey(0.2f);
extern const imp_float IMP_DIFFUSE_SMOOTHNESS = 40.0f;

extern const Reflectance IMP_SHINY_GLOSSY_REFLECTANCE = Reflectance::grey(0.35f);
extern const imp_float IMP_SHINY_SMOOTHNESS = 400.0f;

extern const Color IMP_REFRACTIVE_INDEX = Color::grey(1.03f);
extern const Color IMP_ATTENUATION = Color::grey(0.4f);
extern const Color IMP_EXTINCTION_COEFF = Color::black();

extern const BlinnPhongMaterial IMP_DIFFUSE_BROWN = BlinnPhongMaterial(Reflectance(0x3E211B), IMP_DIFFUSE_GLOSSY_REFLECTANCE, IMP_DIFFUSE_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_DIFFUSE_SLATEGREY = BlinnPhongMaterial(Reflectance(112/255.0f, 128/255.0f, 144/255.0f), IMP_DIFFUSE_GLOSSY_REFLECTANCE, IMP_DIFFUSE_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_DIFFUSE_DARKSLATEGREY = BlinnPhongMaterial(Reflectance(47/255.0f, 79/255.0f, 79/255.0f), IMP_DIFFUSE_GLOSSY_REFLECTANCE, IMP_DIFFUSE_SMOOTHNESS);

extern const BlinnPhongMaterial IMP_SHINY_WHITE = BlinnPhongMaterial(Reflectance::white(), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_BLACK = BlinnPhongMaterial(Reflectance::black(), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_GREY = BlinnPhongMaterial(Reflectance::grey(0.5f), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_RED = BlinnPhongMaterial(Reflectance::red(), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_GREEN = BlinnPhongMaterial(Reflectance::green(), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_BLUE = BlinnPhongMaterial(Reflectance::blue(), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_YELLOW = BlinnPhongMaterial(Reflectance::yellow(), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_CYAN = BlinnPhongMaterial(Reflectance::cyan(), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_MAGENTA = BlinnPhongMaterial(Reflectance::magenta(), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_ORANGE = BlinnPhongMaterial(Reflectance::orange(), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_MAROON = BlinnPhongMaterial(Reflectance::maroon(), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_PINK = BlinnPhongMaterial(Reflectance::pink(), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_PURPLE = BlinnPhongMaterial(Reflectance::purple(), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_GOLD = BlinnPhongMaterial(Reflectance::gold(), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_CRIMSON = BlinnPhongMaterial(Reflectance::crimson(), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_FORESTGREEN = BlinnPhongMaterial(Reflectance::forestgreen(), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_NAVY = BlinnPhongMaterial(Reflectance::navy(), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);

extern const BlinnPhongMaterial IMP_TRANSPARENT_CRIMSON = BlinnPhongMaterial(Color::black(), IMP_REFRACTIVE_INDEX, IMP_EXTINCTION_COEFF, Reflectance::crimson(), IMP_ATTENUATION, IMP_FLOAT_INF);
extern const BlinnPhongMaterial IMP_TRANSPARENT_FORESTGREEN = BlinnPhongMaterial(Color::black(), IMP_REFRACTIVE_INDEX, IMP_EXTINCTION_COEFF, Reflectance::forestgreen(), IMP_ATTENUATION, IMP_FLOAT_INF);
extern const BlinnPhongMaterial IMP_TRANSPARENT_NAVY = BlinnPhongMaterial(Color::black(), IMP_REFRACTIVE_INDEX, IMP_EXTINCTION_COEFF, Reflectance::navy(), IMP_ATTENUATION, IMP_FLOAT_INF);

} // Rendering3D
} // Impact