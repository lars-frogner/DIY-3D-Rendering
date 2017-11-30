#include "SceneGraph.hpp"
#include <cassert>
#include <iostream>

namespace Impact {
namespace Rendering3D {

SceneGraph::SceneGraph(const TriangleMesh& root_object)
    : _object_ptr(new TriangleMesh(root_object)),
     _is_transformation_node(false) {}

SceneGraph::SceneGraph(const LinearTransformation& root_transformation)
    : _transformation_ptr(new LinearTransformation(root_transformation)),
     _is_transformation_node(true) {}

SceneGraph::SceneGraph(const AffineTransformation& root_transformation)
    : _transformation_ptr(new AffineTransformation(root_transformation)),
     _is_transformation_node(true) {}

SceneGraph::SceneGraph(SceneGraph* new_parent_ptr,
                       TriangleMesh* new_object_ptr)
    : _parent_ptr(new_parent_ptr),
      _object_ptr(new_object_ptr),
      _is_transformation_node(false) {}

SceneGraph::SceneGraph(SceneGraph* new_parent_ptr,
                       Transformation* new_transformation_ptr)
    : _parent_ptr(new_parent_ptr),
      _transformation_ptr(new_transformation_ptr),
      _is_transformation_node(true) {}

SceneGraph* SceneGraph::addObject(const TriangleMesh& new_object)
{
    _child_ptrs.push_back(new SceneGraph(this, new TriangleMesh(new_object)));
    return _child_ptrs.back();
}

SceneGraph* SceneGraph::addTransformation(const LinearTransformation& new_transformation)
{
    _child_ptrs.push_back(new SceneGraph(this, new LinearTransformation(new_transformation)));
    return _child_ptrs.back();
}

SceneGraph* SceneGraph::addTransformation(const AffineTransformation& new_transformation)
{
    _child_ptrs.push_back(new SceneGraph(this, new AffineTransformation(new_transformation)));
    return _child_ptrs.back();
}

SceneGraph::~SceneGraph()
{
    if (_is_transformation_node)
        delete _transformation_ptr;
    else
        delete _object_ptr;

    while (!_child_ptrs.empty())
    {
        delete _child_ptrs.back();
        _child_ptrs.pop_back();
    }
}

std::vector<Geometry3D::TriangleMesh> SceneGraph::getTransformedObjects(SceneGraph& root)
{
    assert(!root._parent_ptr);
    
    std::vector<TriangleMesh> object_stack;
    std::vector<Transformation*> transformation_stack;

    if (root._is_transformation_node)
    {
        transformation_stack.push_back(root._transformation_ptr);
    }
    else
    {
        object_stack.push_back(TriangleMesh(*(root._object_ptr)));
        transformation_stack.push_back(new LinearTransformation()); // Identity transformation
    }

    for (imp_uint i = 0; i < root._child_ptrs.size(); i++)
    {
        _processNode(*(root._child_ptrs[i]), transformation_stack, object_stack);
    }

    delete transformation_stack.back();

    return object_stack;
}

void SceneGraph::_processNode(SceneGraph& node,
                              std::vector<Transformation*>& transformation_stack,
                              std::vector<TriangleMesh>& object_stack)
{
    Transformation* previous = transformation_stack.back();
    
    if (node._is_transformation_node)
    {
        Transformation* current = node._transformation_ptr;
        Transformation* combined = NULL;

        if (LinearTransformation* new_current = dynamic_cast<LinearTransformation*>(current))
        {
            if (LinearTransformation* new_previous = dynamic_cast<LinearTransformation*>(previous))
            {
                combined = new LinearTransformation((*new_previous)*(*new_current));
            }
            else if (AffineTransformation* new_previous = dynamic_cast<AffineTransformation*>(previous))
            {
                combined = new AffineTransformation((*new_previous)*(*new_current));
            }
        }
        else if (AffineTransformation* new_current = dynamic_cast<AffineTransformation*>(current))
        {
            if (LinearTransformation* new_previous = dynamic_cast<LinearTransformation*>(previous))
            {
                combined = new AffineTransformation((*new_previous)*(*new_current));
            }
            else if (AffineTransformation* new_previous = dynamic_cast<AffineTransformation*>(previous))
            {
                combined = new AffineTransformation((*new_previous)*(*new_current));
            }
        }
        transformation_stack.push_back(combined);
    }
    else
    {
        if (LinearTransformation* new_previous = dynamic_cast<LinearTransformation*>(previous))
        {
			node._object_ptr->applyTransformation(*new_previous);
            object_stack.push_back(*(node._object_ptr));
        }
        else if (AffineTransformation* new_previous = dynamic_cast<AffineTransformation*>(previous))
        {
			node._object_ptr->applyTransformation(*new_previous);
            object_stack.push_back(*(node._object_ptr));
        }
    }

    for (imp_uint i = 0; i < node._child_ptrs.size(); i++)
    {
        _processNode(*(node._child_ptrs[i]), transformation_stack, object_stack);
    }

    if (node._is_transformation_node)
    {
        delete transformation_stack.back();
        transformation_stack.pop_back();
    }
}

} // Rendering3D
} // Impact
