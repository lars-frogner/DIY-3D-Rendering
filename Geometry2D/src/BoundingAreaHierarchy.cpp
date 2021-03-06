#include "BoundingAreaHierarchy.hpp"
#include "BAHNode.hpp"
#include <cassert>

namespace Impact {
namespace Geometry2D {

BoundingAreaHierarchy::BoundingAreaHierarchy() {}

BoundingAreaHierarchy::BoundingAreaHierarchy(const BoundingAreaHierarchy& other)
	: _root_node(other._root_node) {}

BoundingAreaHierarchy& BoundingAreaHierarchy::operator=(const BoundingAreaHierarchy& other)
{
	_root_node = other._root_node;
	return *this;
}

BoundingAreaHierarchy::BoundingAreaHierarchy(const AxisAlignedRectangle& bounding_area,
											 const std::vector<AABRContainer>& objects)
{
    imp_uint n_objects = static_cast<imp_uint>(objects.size());
    assert(n_objects > 0);

    _root_node = node_ptr(new BAHNode(bounding_area, objects[0]));

	for (imp_uint i = 1; i < n_objects; i++)
	{
		_root_node->_insertObject(objects[i]);
	}

    _root_node->_computeBoundingAreas();
}

void BoundingAreaHierarchy::addPotentiallyIntersectedObjectIDs(const Point& point, std::list<imp_uint>& object_ids) const
{
    assert(_root_node);
    _root_node->_addPotentiallyIntersectedObjectIDs(point, object_ids);
}

} // Geometry2D
} // Impact
