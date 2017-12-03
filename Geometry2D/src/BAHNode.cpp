#include "BAHNode.hpp"
#include <cassert>
#include <iostream>
#include <algorithm>

namespace Impact {
namespace Geometry2D {

BAHNode::BAHNode(const AxisAlignedRectangle& new_bounding_area,
				 const AABRContainer& new_object)
    : _quadrants(new_bounding_area.getQuadrants()),
      _object(new_object),
      _child_nodes(4) {}

BAHNode::BAHNode(const BAHNode& other)
	: _quadrants(other._quadrants),
      _object(other._object),
	  _has_children(other._has_children),
	  _child_nodes(other._child_nodes) {}

BAHNode& BAHNode::operator=(const BAHNode& other)
{
	_quadrants = other._quadrants;
	_object = other._object;
	_has_children = other._has_children;
	_child_nodes = other._child_nodes;
	return *this;
}

void BAHNode::_insertObject(const AABRContainer& object)
{
    bool inserted = false;

    for (imp_uint i = 0; i < 4; i++)
    {
        if (_quadrants[i].containsInclusive(object.centroid))
        {
			if (_child_nodes[i])
			{
				_child_nodes[i]->_insertObject(object);
			}
			else
			{
				_child_nodes[i] = node_ptr(new BAHNode(_quadrants[i], object));
				_has_children = true;
			}

            inserted = true;
            break;
        }
    }

    if (!inserted)
    {
        std::cerr << "Error: centroid " << object.centroid.toString() << " of object with id " << object.id << " is outside the bounding area" << std::endl;
        throw;
    }
}

const AxisAlignedRectangle& BAHNode::_computeBoundingAreas()
{
    _aabr = _object.aabr;

    _child_nodes.erase(std::remove(_child_nodes.begin(), _child_nodes.end(), nullptr),
                       _child_nodes.end());

    for (std::vector<node_ptr>::const_iterator iter = _child_nodes.begin(); iter != _child_nodes.end(); ++iter)
    {
        _aabr.merge((*iter)->_computeBoundingAreas());
    }

    _quadrants.clear();

    return _aabr;
}

std::vector<imp_uint> BAHNode::_getIntersectedObjectIDs(const Point& point) const
{
    std::vector<imp_uint> object_ids;

    if (_has_children && _aabr.containsInclusive(point))
    {
        for (std::vector<node_ptr>::const_iterator iter = _child_nodes.begin(); iter != _child_nodes.end(); iter++)
        {
            const std::vector<imp_uint>& child_object_ids = (*iter)->_getIntersectedObjectIDs(point);
            object_ids.insert(object_ids.end(), child_object_ids.begin(), child_object_ids.end());
        }
    }

	if (_object.aabr.containsInclusive(point))
	{
		object_ids.push_back(_object.id);
	}
    
    return object_ids;
}

} // Geometry2D
} // Impact
