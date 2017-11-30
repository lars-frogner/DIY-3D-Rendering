#include "material_assets.hpp"

namespace Impact {
namespace Rendering3D {

extern const Reflectance IMP_DIFFUSE_GLOSSY_REFLECTANCE = Reflectance::grey(0.2f);
extern const imp_float IMP_DIFFUSE_SMOOTHNESS = 40.0f;

extern const Reflectance IMP_SHINY_GLOSSY_REFLECTANCE = Reflectance::grey(0.35f);
extern const imp_float IMP_SHINY_SMOOTHNESS = 400.0f;

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
extern const BlinnPhongMaterial IMP_SHINY_NAVY = BlinnPhongMaterial(Reflectance(0, 0, 128/255.0f), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_FORESTGREEN = BlinnPhongMaterial(Reflectance(34/255.0f, 139/255.0f, 34/255.0f), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);
extern const BlinnPhongMaterial IMP_SHINY_CRIMSON = BlinnPhongMaterial(Reflectance(220/255.0f, 20/255.0f, 60/255.0f), IMP_SHINY_GLOSSY_REFLECTANCE, IMP_SHINY_SMOOTHNESS);

} // Rendering3D
} // Impact