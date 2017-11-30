#pragma once
#include "BoundingVolumeHierarchy.hpp"
#include "BVHNode.hpp"
#include "TriangleMesh.hpp"
#include <cassert>
#include <queue>

namespace Impact {
namespace Geometry3D {

BoundingVolumeHierarchy::BoundingVolumeHierarchy() {}

/*BoundingVolumeHierarchy::BoundingVolumeHierarchy(const BoundingVolumeHierarchy& other)
{
	if (other._root_node)
		_root_node = node_ptr(new BVHNode(*other._root_node));
}

BoundingVolumeHierarchy::~BoundingVolumeHierarchy()
{
	_root_node.reset();
}

BoundingVolumeHierarchy& BoundingVolumeHierarchy::operator=(const BoundingVolumeHierarchy& other)
{
	if (other._root_node)
		_root_node = node_ptr(new BVHNode(*other._root_node));

	return *this;
}*/

BoundingVolumeHierarchy::BoundingVolumeHierarchy(const AxisAlignedBox& bounding_volume, const std::vector< AABBContainer >& objects)
{
    imp_uint n_objects = static_cast<imp_uint>(objects.size());
    assert(n_objects > 0);

    _root_node = node_ptr(new BVHNode(bounding_volume, objects[0]));

	for (imp_uint i = 1; i < n_objects; i++)
	{
		_root_node->_insertObject(objects[i]);
	}

    _root_node->_computeBoundingVolumes();
}

imp_float BoundingVolumeHierarchy::evaluateRayIntersection(const TriangleMesh& mesh, const Ray& ray) const
{
    assert(_root_node);

    // Find intersection distance with root bounding box
    imp_float distance = _root_node->_aabb.evaluateRayIntersection(ray);

    // Return immediately if the ray doesn't intersect the root bounding box
	if (distance == IMP_FLOAT_INF)
	{
		return distance;
	}
    
    imp_float smallest_true_intersection_distance = IMP_FLOAT_INF;
    std::priority_queue<BVHQueueElement> priority_queue;
    std::vector<node_ptr>::const_iterator iter;
    node_ptr current_node;

    priority_queue.push(BVHQueueElement(_root_node, distance));

    // Repeat while there are still nodes in the queue and there are AABB
    // intersection distances smaller than the smallest true intersection distance
    while (!priority_queue.empty() &&
           priority_queue.top().distance < smallest_true_intersection_distance)
    {
        // Extract node with smallest AABB intersection distance
        current_node = priority_queue.top().node;
        priority_queue.pop();
        
        // Find true intersection distance of the object belonging to the node
        distance = mesh.evaluateRayFaceIntersectionDistanceOnly(ray, current_node->_object.id);

        // If it is smaller than all the previous ones, store it as new smallest intersection distance
		if (distance < smallest_true_intersection_distance)
		{
			smallest_true_intersection_distance = distance;
		}

        // Loop through child nodes
        for (iter = (current_node->_child_nodes).begin(); iter != (current_node->_child_nodes).end(); ++iter)
        {
            // Find AABB intersection distance of child node
            distance = (*iter)->_aabb.evaluateRayIntersection(ray);

            // If there was an itersection, add the child node to the priority list
			if (distance < IMP_FLOAT_INF)
			{
				priority_queue.push(BVHQueueElement(*iter, distance));
			}
        }
    }

    return smallest_true_intersection_distance;
}

std::vector<imp_uint> BoundingVolumeHierarchy::getIntersectedObjectIDs(const Ray& ray) const
{
    assert(_root_node);
    return _root_node->_getIntersectedObjectIDs(ray);
}

} // Geometry3D
} // Impact
