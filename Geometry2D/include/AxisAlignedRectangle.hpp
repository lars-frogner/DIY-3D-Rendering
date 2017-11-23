#pragma once
#include "precision.hpp"
#include "Point2.hpp"
#include "Vector2.hpp"
#include <vector>

namespace Impact {
namespace Geometry2D {

class AxisAlignedRectangle {

public:
    Point lower_corner, upper_corner;

    AxisAlignedRectangle();
    AxisAlignedRectangle(const Point& new_lower_corner,
                         const Point& new_upper_corner);

    Vector   getSpan()   const;
    Point    getCenter() const;
    imp_float getWidth()  const;
    imp_float getHeight() const;

    std::vector<AxisAlignedRectangle> getQuadrants() const;

    AxisAlignedRectangle& setSpan(const Vector& new_span);
    AxisAlignedRectangle& setCenter(const Point& center);
    AxisAlignedRectangle& setWidth(imp_float new_width);
    AxisAlignedRectangle& setHeight(imp_float new_height);
    AxisAlignedRectangle& setDimensions(imp_float new_width, imp_float new_height);

    bool containsInclusive(const Point& point) const;
    bool containsUpperExclusive(const Point& point) const;

    AxisAlignedRectangle& translate(imp_float dx, imp_float dy);
    AxisAlignedRectangle& translate(const Vector& displacement);
    AxisAlignedRectangle getTranslated(imp_float dx, imp_float dy) const;
    AxisAlignedRectangle getTranslated(const Vector& displacement) const;
    
    AxisAlignedRectangle& merge(const AxisAlignedRectangle& other);

    AxisAlignedRectangle& validateOrientation();
};

struct AABRContainer
{
    AxisAlignedRectangle aabr;
    Point centroid;
    imp_uint id;
};

} // Geometry2D
} // Impact
