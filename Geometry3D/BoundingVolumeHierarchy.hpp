#pragma once
#include <memory>
#include <vector>
#include <queue>
#include <assert.h>
#include "AxisAlignedBox.hpp"
#include "Ray.hpp"

namespace Geometry3D {
    
template <typename F>
class TriangleMesh;
    
template <typename F>
class BoundingVolumeHierarchy;

template <typename F>
class BVHNode {
    
friend BoundingVolumeHierarchy<F>;

private:
    typedef std::shared_ptr< BVHNode<F> > node_ptr;
    
    const F _INFINITY = std::numeric_limits<F>::infinity();

protected:
    AxisAlignedBox<F> _aabb;
    std::vector< AxisAlignedBox<F> > _octants;
    AABBContainer<F> _object;
    std::vector<node_ptr> _child_nodes;
    bool _has_children = false;

    BVHNode<F>(const AxisAlignedBox<F>& new_bounding_volume, const AABBContainer<F>& new_object);

    void _insertObject(const AABBContainer<F>& object);
    const AxisAlignedBox<F>& _computeBoundingVolumes();

    std::vector<size_t> _getIntersectedObjectIDs(const Ray<F>& ray) const;
};

template <typename F>
struct BVHQueueElement 
{ 
    typedef std::shared_ptr< BVHNode<F> > node_ptr;

    node_ptr node;
    F distance;

    BVHQueueElement(node_ptr new_node, F new_distance)
        : node(new_node), distance(new_distance) {} 

    friend bool operator<(const BVHQueueElement& first, const BVHQueueElement& second)
    {
        return first.distance > second.distance;
    } 
};

template <typename F>
class BoundingVolumeHierarchy {

private:
    typedef std::shared_ptr< BVHNode<F> > node_ptr;

    F _INFINITY = std::numeric_limits<F>::infinity();

    node_ptr _root_node;

public:
    BoundingVolumeHierarchy<F>();
    BoundingVolumeHierarchy<F>(const AxisAlignedBox<F>& bounding_volume, const std::vector< AABBContainer<F> >& objects);

    F evaluateRayIntersection(const TriangleMesh<F>& mesh, const Ray<F>& ray) const;
    std::vector<size_t> getIntersectedObjectIDs(const Ray<F>& ray) const;
};

template <typename F>
BVHNode<F>::BVHNode(const AxisAlignedBox<F>& new_bounding_volume, const AABBContainer<F>& new_object)
    : _octants(new_bounding_volume.getOctants()),
      _object(new_object),
      _child_nodes(8) {}

template <typename F>
void BVHNode<F>::_insertObject(const AABBContainer<F>& object)
{
    bool inserted = false;

    for (size_t i = 0; i < 8; i++)
    {
        if (_octants[i].containsInclusive(object.centroid))
        {
			if (_child_nodes[i])
			{
				_child_nodes[i]->_insertObject(object);
			}
			else
			{
				_child_nodes[i] = node_ptr(new BVHNode<F>(_octants[i], object));
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

template <typename F>
const AxisAlignedBox<F>& BVHNode<F>::_computeBoundingVolumes()
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

template <typename F>
std::vector<size_t> BVHNode<F>::_getIntersectedObjectIDs(const Ray<F>& ray) const
{
    std::vector<size_t> object_ids;

    if (_has_children && _aabb.evaluateRayIntersection(ray) < _INFINITY)
    {
        for (std::vector<node_ptr>::const_iterator iter = _child_nodes.begin(); iter != _child_nodes.end(); ++iter)
        {
            const std::vector<size_t>& child_object_ids = (*iter)->_getIntersectedObjectIDs(ray);
            object_ids.insert(object_ids.end(), child_object_ids.begin(), child_object_ids.end());
        }
    }

	if (_object.aabb.evaluateRayIntersection(ray) < _INFINITY)
	{
		object_ids.push_back(_object.id);
	}
    
    return object_ids;
}

template <typename F>
BoundingVolumeHierarchy<F>::BoundingVolumeHierarchy() {}

template <typename F>
BoundingVolumeHierarchy<F>::BoundingVolumeHierarchy(const AxisAlignedBox<F>& bounding_volume, const std::vector< AABBContainer<F> >& objects)
{
    size_t n_objects = objects.size();
    assert(n_objects > 0);

    _root_node = node_ptr(new BVHNode<F>(bounding_volume, objects[0]));

	for (size_t i = 1; i < n_objects; i++)
	{
		_root_node->_insertObject(objects[i]);
	}

    _root_node->_computeBoundingVolumes();
}

template <typename F>
F BoundingVolumeHierarchy<F>::evaluateRayIntersection(const TriangleMesh<F>& mesh, const Ray<F>& ray) const
{
    assert(_root_node);

    // Find intersection distance with root bounding box
    F distance = _root_node->_aabb.evaluateRayIntersection(ray);

    // Return immediately if the ray doesn't intersect the root bounding box
	if (distance == _INFINITY)
	{
		return distance;
	}
    
    F smallest_true_intersection_distance = _INFINITY;
    std::priority_queue< BVHQueueElement<F> > priority_queue;
    std::vector<node_ptr>::const_iterator iter;
    node_ptr current_node;

    priority_queue.push(BVHQueueElement<F>(_root_node, distance));

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
			if (distance < _INFINITY)
			{
				priority_queue.push(BVHQueueElement<F>(*iter, distance));
			}
        }
    }

    return smallest_true_intersection_distance;
}

template <typename F>
std::vector<size_t> BoundingVolumeHierarchy<F>::getIntersectedObjectIDs(const Ray<F>& ray) const
{
    assert(_root_node);
    return _root_node->_getIntersectedObjectIDs(ray);
}

} // Geometry3D
