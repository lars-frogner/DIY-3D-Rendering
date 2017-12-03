#pragma once
#include "precision.hpp"
#include "Point2.hpp"
#include "AxisAlignedRectangle.hpp"
#include <vector>
#include <memory>

namespace Impact {
namespace Geometry2D {

class BAHNode;

class BoundingAreaHierarchy {

private:
    typedef std::shared_ptr<BAHNode> node_ptr;

    node_ptr _root_node;

public:
    BoundingAreaHierarchy();
    BoundingAreaHierarchy(const AxisAlignedRectangle& bounding_area,
						  const std::vector<AABRContainer>& objects);
	BoundingAreaHierarchy(const BoundingAreaHierarchy& other);
	BoundingAreaHierarchy& operator=(const BoundingAreaHierarchy& other);

    std::vector<imp_uint> getIntersectedObjectIDs(const Point& point) const;
};

} // Geometry2D
} // Impact
