#pragma once
#include "precision.hpp"
#include "Point2.hpp"
#include "AxisAlignedRectangle.hpp"
#include "BoundingAreaHierarchy.hpp"
#include <vector>
#include <memory>

namespace Impact {
namespace Geometry2D {

class BAHNode {
    
friend BoundingAreaHierarchy;

private:
    typedef std::unique_ptr<BAHNode> node_ptr;

protected:
    AxisAlignedRectangle _aabr;
    std::vector<AxisAlignedRectangle> _quadrants;
    AABRContainer _object;
    std::vector<node_ptr> _child_nodes;
    bool _has_children = false;

    BAHNode(const AxisAlignedRectangle& new_bounding_area,
			const AABRContainer& new_object);

    void _insertObject(const AABRContainer& object);
    const AxisAlignedRectangle& _computeBoundingAreas();

    std::vector<imp_uint> _getIntersectedObjectIDs(const Point& point) const;
};

} // Geometry2D
} // Impact
