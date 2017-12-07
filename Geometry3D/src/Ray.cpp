#include "Ray.hpp"

namespace Impact {
namespace Geometry3D {

Ray::Ray(const Point&  new_origin,
         const Vector& new_direction)
    : origin(new_origin),
      direction(new_direction),
      inverse_direction(zeroAllowedDivision(1, new_direction)),
      max_distance(IMP_FLOAT_INF) {}

Ray::Ray(const Point&  new_origin,
         const Vector& new_direction,
         imp_float new_max_distance)
    : origin(new_origin),
      direction(new_direction),
      inverse_direction(zeroAllowedDivision(1, new_direction)),
      max_distance(new_max_distance) {}

Point Ray::operator()(imp_float distance) const
{
    return origin + direction*distance;
}

Ray& Ray::alignWith(const Vector& other_direction)
{
    direction = other_direction.getNormalized();
    return *this;
}

Ray& Ray::pointAt(const Point& point)
{
    return alignWith(point - origin);
}

Ray& Ray::nudge(imp_float nudge_distance)
{
	origin += nudge_distance*direction;
	return *this;
}

Ray& Ray::nudge(imp_float nudge_distance, const Vector& nudge_direction)
{
	origin += nudge_distance*nudge_direction;
	return *this;
}

} // Geometry3D
} // Impact
