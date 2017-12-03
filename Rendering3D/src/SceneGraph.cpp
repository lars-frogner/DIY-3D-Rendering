#include "SceneGraph.hpp"
#include <cassert>
#include <iostream>

namespace Impact {
namespace Rendering3D {

SceneGraph::SceneGraph(const Model& root_model)
    : _model(root_model),
      _is_transformation_node(false) {}

SceneGraph::SceneGraph(const AffineTransformation& root_transformation)
    : _model(nullptr, nullptr, root_transformation),
      _is_transformation_node(true) {}

SceneGraph::SceneGraph(SceneGraph* new_parent_ptr,
                       const Model& new_model)
    : _parent_ptr(new_parent_ptr),
      _model(new_model),
      _is_transformation_node(false) {}

SceneGraph::SceneGraph(SceneGraph* new_parent_ptr,
                       const AffineTransformation& new_transformation)
    : _parent_ptr(new_parent_ptr),
	  _model(nullptr, nullptr, new_transformation),
      _is_transformation_node(true) {}

SceneGraph::~SceneGraph()
{
    while (!_child_ptrs.empty())
    {
        delete _child_ptrs.back();
        _child_ptrs.pop_back();
    }
}

SceneGraph* SceneGraph::addModel(const Model& new_model)
{
    _child_ptrs.push_back(new SceneGraph(this, new_model));
    return _child_ptrs.back();
}

SceneGraph* SceneGraph::addTransformation(const AffineTransformation& new_transformation)
{
    _child_ptrs.push_back(new SceneGraph(this, new_transformation));
    return _child_ptrs.back();
}

std::vector<Model> SceneGraph::getTransformedModels(SceneGraph& root)
{
    assert(!root._parent_ptr);
    
    std::vector<Model> model_stack;
    std::vector<AffineTransformation> transformation_stack;

    if (root._is_transformation_node)
    {
        transformation_stack.push_back(root._model.getTransformation());
    }
    else
    {
        model_stack.push_back(root._model);
        transformation_stack.push_back(AffineTransformation());
    }

    for (imp_uint i = 0; i < root._child_ptrs.size(); i++)
    {
        _processNode(*(root._child_ptrs[i]), transformation_stack, model_stack);
    }

    transformation_stack.clear();

    return model_stack;
}

void SceneGraph::_processNode(SceneGraph& node,
                              std::vector<AffineTransformation>& transformation_stack,
                              std::vector<Model>& model_stack)
{
    const AffineTransformation& previous = transformation_stack.back();
    
    if (node._is_transformation_node)
    {
        transformation_stack.push_back(previous(node._model.getTransformation()));
    }
    else
    {
        node._model.applyTransformation(previous);
        model_stack.push_back(node._model);
    }

    for (imp_uint i = 0; i < node._child_ptrs.size(); i++)
    {
        _processNode(*(node._child_ptrs[i]), transformation_stack, model_stack);
    }

    if (node._is_transformation_node)
    {
        transformation_stack.pop_back();
    }
}

} // Rendering3D
} // Impact
