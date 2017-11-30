#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Ray.hpp"
#include <vector>

namespace Impact {
namespace Geometry3D {

// Forward declaration of Sphere class
class Sphere;

class AxisAlignedBox {

public:
    Point lower_corner, upper_corner;

    AxisAlignedBox();

    AxisAlignedBox(const Point& new_lower_corner,
                   const Point& new_upper_corner);
    
    AxisAlignedBox(const Point&  new_lower_corner,
                   const Vector& new_span);
    
    AxisAlignedBox(const Point&  new_lower_corner,
                   imp_float width, imp_float height, imp_float depth);

    static AxisAlignedBox merged(const AxisAlignedBox& aab_1, const AxisAlignedBox& aab_2);

    Vector getSpan() const;
    Point getCenter() const;
    imp_float getWidth() const;
    imp_float getHeight() const;
    imp_float getDepth() const;

    std::vector<AxisAlignedBox> getOctants() const;

    AxisAlignedBox& setSpan(const Vector& new_span);
    AxisAlignedBox& setCenter(const Point& center);
    AxisAlignedBox& setWidth(imp_float new_width);
    AxisAlignedBox& setHeight(imp_float new_height);
    AxisAlignedBox& setDepth(imp_float new_depth);
    AxisAlignedBox& setDimensions(imp_float new_width, imp_float new_height, imp_float new_depth);

    AxisAlignedBox& translate(imp_float dx, imp_float dy, imp_float dz);
    AxisAlignedBox& translate(const Vector& displacement);
    AxisAlignedBox getTranslated(imp_float dx, imp_float dy, imp_float dz) const;
    AxisAlignedBox getTranslated(const Vector& displacement) const;
    
    AxisAlignedBox& merge(const AxisAlignedBox& other);

    imp_float evaluateRayIntersection(const Ray& ray) const;
	bool intersects(const AxisAlignedBox& other) const;
	bool encloses(const AxisAlignedBox& other) const;

	bool containsInclusive(const Point& point) const;
    bool containsUpperExclusive(const Point& point) const;

    Sphere getBoundingSphere() const;

    AxisAlignedBox& validateOrientation();
};

struct AABBContainer
{
    AxisAlignedBox aabb;
    Point centroid;
    imp_uint id;
};

} // Geometry3D
} // Impact
