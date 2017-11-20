#pragma once
#include <memory>
#include <vector>
#include <assert.h>
#include "AxisAlignedRectangle.hpp"
#include "../Rendering3D/Image.hpp"

namespace Geometry2D {
    
template <typename F>
class BoundingAreaHierarchy;

template <typename F>
class BAHNode {
    
friend BoundingAreaHierarchy<F>;

private:
    typedef std::shared_ptr< BAHNode<F> > node_ptr;
    typedef Rendering3D::Image<F> Image;

protected:
    AxisAlignedRectangle<F> _aabr;
    std::vector< AxisAlignedRectangle<F> > _quadrants;
    AABRContainer<F> _object;
    std::vector<node_ptr> _child_nodes;
    bool _has_children = false;

    BAHNode<F>(const AxisAlignedRectangle<F>& new_bounding_area, const AABRContainer<F>& new_object);

    void _insertObject(const AABRContainer<F>& object);
    const AxisAlignedRectangle<F>& _computeBoundingAreas();

    std::vector<size_t> _getIntersectedObjectIDs(const Point<F>& point) const;
    
    void _draw(Image& image, F luminance) const;
};

template <typename F>
class BoundingAreaHierarchy {

private:
    typedef std::shared_ptr< BAHNode<F> > node_ptr;
    typedef Rendering3D::Image<F> Image;

    node_ptr _root_node;

public:
    BoundingAreaHierarchy<F>();
    BoundingAreaHierarchy<F>(const AxisAlignedRectangle<F>& bounding_area, const std::vector< AABRContainer<F> >& objects);

    std::vector<size_t> getIntersectedObjectIDs(const Point<F>& point) const;

    void draw(Image& image, F luminance) const;
};

template <typename F>
BAHNode<F>::BAHNode(const AxisAlignedRectangle<F>& new_bounding_area, const AABRContainer<F>& new_object)
    : _quadrants(new_bounding_area.getQuadrants()),
      _object(new_object),
      _child_nodes(4) {}

template <typename F>
void BAHNode<F>::_insertObject(const AABRContainer<F>& object)
{
    bool inserted = false;

    for (size_t i = 0; i < 4; i++)
    {
        if (_quadrants[i].containsInclusive(object.centroid))
        {
			if (_child_nodes[i])
			{
				_child_nodes[i]->_insertObject(object);
			}
			else
			{
				_child_nodes[i] = node_ptr(new BAHNode<F>(_quadrants[i], object));
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

template <typename F>
const AxisAlignedRectangle<F>& BAHNode<F>::_computeBoundingAreas()
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

template <typename F>
std::vector<size_t> BAHNode<F>::_getIntersectedObjectIDs(const Point<F>& point) const
{
    std::vector<size_t> object_ids;

    if (_has_children && _aabr.containsInclusive(point))
    {
        for (std::vector<node_ptr>::const_iterator iter = _child_nodes.begin(); iter != _child_nodes.end(); ++iter)
        {
            const std::vector<size_t>& child_object_ids = (*iter)->_getIntersectedObjectIDs(point);
            object_ids.insert(object_ids.end(), child_object_ids.begin(), child_object_ids.end());
        }
    }

	if (_object.aabr.containsInclusive(point))
	{
		object_ids.push_back(_object.id);
	}
    
    return object_ids;
}

template <typename F>
void BAHNode<F>::_draw(Image& image, F luminance) const
{
    for (std::vector<node_ptr>::const_iterator iter = _child_nodes.begin(); iter != _child_nodes.end(); ++iter)
    {
        (*iter)->_draw(image, luminance);
    }

    const Point<F>& lower_corner = _aabr.lower_corner;
    const Point<F>& upper_corner = _aabr.upper_corner;

    image.drawLine(lower_corner.x, lower_corner.y, upper_corner.x, lower_corner.y, luminance);
    image.drawLine(upper_corner.x, lower_corner.y, upper_corner.x, upper_corner.y, luminance);
    image.drawLine(lower_corner.x, upper_corner.y, upper_corner.x, upper_corner.y, luminance);
    image.drawLine(lower_corner.x, lower_corner.y, lower_corner.x, upper_corner.y, luminance);
}

template <typename F>
BoundingAreaHierarchy<F>::BoundingAreaHierarchy() {}

template <typename F>
BoundingAreaHierarchy<F>::BoundingAreaHierarchy(const AxisAlignedRectangle<F>& bounding_area, const std::vector< AABRContainer<F> >& objects)
{
    size_t n_objects = objects.size();
    assert(n_objects > 0);

    _root_node = node_ptr(new BAHNode<F>(bounding_area, objects[0]));

	for (size_t i = 1; i < n_objects; i++)
	{
		_root_node->_insertObject(objects[i]);
	}

    _root_node->_computeBoundingAreas();
}

template <typename F>
std::vector<size_t> BoundingAreaHierarchy<F>::getIntersectedObjectIDs(const Point<F>& point) const
{
    assert(_root_node);
    return _root_node->_getIntersectedObjectIDs(point);
}

template <typename F>
void BoundingAreaHierarchy<F>::draw(Image& image, F luminance) const
{
    assert(_root_node);
    _root_node->_draw(image, luminance);
}

} // Geometry2D
