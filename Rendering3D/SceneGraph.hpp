#pragma once
#include <vector>
#include <iostream>
#include "../Geometry3D/TriangleMesh.hpp"
#include "../Transformations3D/Transformation.hpp"
#include "../Transformations3D/LinearTransformation.hpp"
#include "../Transformations3D/AffineTransformation.hpp"
#include "Image.hpp"

namespace Rendering3D {

template <typename F>
class SceneGraph {

private:
    typedef Geometry3D::TriangleMesh<F> TriangleMesh;
    typedef Transformations3D::Transformation<F> Transformation;
    typedef Transformations3D::LinearTransformation<F> LinearTransformation;
    typedef Transformations3D::AffineTransformation<F> AffineTransformation;

    SceneGraph<F>* _parent_ptr = NULL;
    std::vector<SceneGraph<F>*> _child_ptrs;
    
    TriangleMesh*   _object_ptr         = NULL;
    Transformation* _transformation_ptr = NULL;

    bool _is_transformation_node;

    SceneGraph(SceneGraph<F>* new_parent_ptr,
               TriangleMesh*     new_object_ptr);

    SceneGraph(SceneGraph<F>* new_parent_ptr,
               Transformation*   new_transformation_ptr);

    static void _processNode(SceneGraph<F>&             node,
                             std::vector<Transformation*>& transformation_stack,
                             std::vector<TriangleMesh>&    object_stack);

public:
    SceneGraph(const TriangleMesh& root_object);
    SceneGraph(const LinearTransformation& root_transformation);
    SceneGraph(const AffineTransformation& root_transformation);

    SceneGraph(const SceneGraph<F>& other) = delete;
    ~SceneGraph();
    SceneGraph& operator=(const SceneGraph<F>& other) = delete;
    
    SceneGraph<F>* addObject(const TriangleMesh& new_object);
    SceneGraph<F>* addTransformation(const LinearTransformation& new_transformation);
    SceneGraph<F>* addTransformation(const AffineTransformation& new_transformation);
    
    static std::vector<TriangleMesh> getTransformedObjects(SceneGraph<F>& root);
};

template <typename F>
SceneGraph<F>::SceneGraph(const TriangleMesh& root_object)
    : _object_ptr(new TriangleMesh(root_object)),
     _is_transformation_node(false) {}

template <typename F>
SceneGraph<F>::SceneGraph(const LinearTransformation& root_transformation)
    : _transformation_ptr(new LinearTransformation(root_transformation)),
     _is_transformation_node(true) {}

template <typename F>
SceneGraph<F>::SceneGraph(const AffineTransformation& root_transformation)
    : _transformation_ptr(new AffineTransformation(root_transformation)),
     _is_transformation_node(true) {}

template <typename F>
SceneGraph<F>::SceneGraph(SceneGraph<F>* new_parent_ptr,
                          TriangleMesh*     new_object_ptr)
    : _parent_ptr(new_parent_ptr),
      _object_ptr(new_object_ptr),
      _is_transformation_node(false) {}

template <typename F>
SceneGraph<F>::SceneGraph(SceneGraph<F>* new_parent_ptr,
                             Transformation*   new_transformation_ptr)
    : _parent_ptr(new_parent_ptr),
      _transformation_ptr(new_transformation_ptr),
      _is_transformation_node(true) {}

template <typename F>
SceneGraph<F>* SceneGraph<F>::addObject(const TriangleMesh& new_object)
{
    _child_ptrs.push_back(new SceneGraph<F>(this, new TriangleMesh(new_object)));
    return _child_ptrs.back();
}

template <typename F>
SceneGraph<F>* SceneGraph<F>::addTransformation(const LinearTransformation& new_transformation)
{
    _child_ptrs.push_back(new SceneGraph<F>(this, new LinearTransformation(new_transformation)));
    return _child_ptrs.back();
}

template <typename F>
SceneGraph<F>* SceneGraph<F>::addTransformation(const AffineTransformation& new_transformation)
{
    _child_ptrs.push_back(new SceneGraph<F>(this, new AffineTransformation(new_transformation)));
    return _child_ptrs.back();
}

template <typename F>
SceneGraph<F>::~SceneGraph()
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

template <typename F>
std::vector< Geometry3D::TriangleMesh<F> > SceneGraph<F>::getTransformedObjects(SceneGraph<F>& root)
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

    for (size_t i = 0; i < root._child_ptrs.size(); i++)
    {
        _processNode(*(root._child_ptrs[i]), transformation_stack, object_stack);
    }

    delete transformation_stack.back();

    return object_stack;
}

template <typename F>
void SceneGraph<F>::_processNode(SceneGraph<F>&             node,
                                 std::vector<Transformation*>& transformation_stack,
                                 std::vector<TriangleMesh>&    object_stack)
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
            object_stack.push_back(node._object_ptr->applyTransformation(*new_previous));
        }
        else if (AffineTransformation* new_previous = dynamic_cast<AffineTransformation*>(previous))
        {
            object_stack.push_back(node._object_ptr->applyTransformation(*new_previous));
        }
    }

    for (size_t i = 0; i < node._child_ptrs.size(); i++)
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
