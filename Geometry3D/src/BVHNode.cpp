#include "BVHNode.hpp"
#include <cassert>
#include <iostream>

namespace Impact {
namespace Geometry3D {

BVHNode::BVHNode(const AxisAlignedBox& new_bounding_volume, const AABBContainer& new_object)
    : _octants(new_bounding_volume.getOctants()),
      _object(new_object),
      _child_nodes(8) {}

/*BVHNode::~BVHNode()
{
	_child_nodes.clear();
}

BVHNode::BVHNode(const BVHNode& other)
	: _octants(other._octants),
      _object(other._object),
	  _has_children(other._has_children)
{
	_child_nodes.reserve(other._child_nodes.size());

	for (std::vector<node_ptr>::const_iterator iter = other._child_nodes.begin(); iter != other._child_nodes.end(); iter++)
	{
		_child_nodes.push_back(node_ptr(new BVHNode(*(*iter))));
	}
}

BVHNode& BVHNode::operator=(const BVHNode& other)
{
	_octants = other._octants;
	_object = other._object;
	_has_children = other._has_children;

	_child_nodes.reserve(other._child_nodes.size());

	for (std::vector<node_ptr>::const_iterator iter = other._child_nodes.begin(); iter != other._child_nodes.end(); iter++)
	{
		_child_nodes.push_back(node_ptr(new BVHNode(*(*iter))));
	}

	return *this;
}*/

void BVHNode::_insertObject(const AABBContainer& object)
{
    bool inserted = false;

    for (imp_uint i = 0; i < 8; i++)
    {
        if (_octants[i].containsInclusive(object.centroid))
        {
			if (_child_nodes[i])
			{
				_child_nodes[i]->_insertObject(object);
			}
			else
			{
				_child_nodes[i] = node_ptr(new BVHNode(_octants[i], object));
				_has_children = true;
			}

            inserted = true;
            break;
        }
    }

    if (!inserted)
    {
        std::cerr << "Error: centroid " << object.centroid.toString() << " of object with id " << object.id << " is outside the bounding volume" << std::endl;
        throw;
    }
}

const AxisAlignedBox& BVHNode::_computeBoundingVolumes()
{
    _aabb = _object.aabb;

    _child_nodes.erase(std::remove(_child_nodes.begin(), _child_nodes.end(), nullptr),
                       _child_nodes.end());

    for (std::vector<node_ptr>::const_iterator iter = _child_nodes.begin(); iter != _child_nodes.end(); ++iter)
    {
        _aabb.merge((*iter)->_computeBoundingVolumes());
    }

    _octants.clear();

    return _aabb;
}

std::vector<imp_uint> BVHNode::_getIntersectedObjectIDs(const Ray& ray) const
{
    std::vector<imp_uint> object_ids;

    if (_has_children && _aabb.evaluateRayIntersection(ray) < IMP_FLOAT_INF)
    {
        for (std::vector<node_ptr>::const_iterator iter = _child_nodes.begin(); iter != _child_nodes.end(); ++iter)
        {
            const std::vector<imp_uint>& child_object_ids = (*iter)->_getIntersectedObjectIDs(ray);
            object_ids.insert(object_ids.end(), child_object_ids.begin(), child_object_ids.end());
        }
    }

	if (_object.aabb.evaluateRayIntersection(ray) < IMP_FLOAT_INF)
	{
		object_ids.push_back(_object.id);
	}
    
    return object_ids;
}

} // Geometry3D
} // Impact
