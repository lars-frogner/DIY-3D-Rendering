#pragma once
#include "precision.hpp"
#include "AffineTransformation.hpp"
#include "Model.hpp"
#include <vector>

namespace Impact {
namespace Rendering3D {

class SceneGraph {

private:
    typedef Geometry3D::AffineTransformation AffineTransformation;

    SceneGraph* _parent_ptr = nullptr;
    std::vector<SceneGraph*> _child_ptrs;
    
    Model _model;

    bool _is_transformation_node;

    SceneGraph(SceneGraph* new_parent_ptr,
               const Model& new_model);

    SceneGraph(SceneGraph* new_parent_ptr,
               const AffineTransformation& new_transformation);

    static void _processNode(SceneGraph& node,
                             std::vector<AffineTransformation>& transformation_stack,
                             std::vector<Model>& object_stack);

public:
    SceneGraph(const Model& root_model);
    SceneGraph(const AffineTransformation& root_transformation);

    SceneGraph(const SceneGraph& other) = delete;
    ~SceneGraph();
    SceneGraph& operator=(const SceneGraph& other) = delete;
    
    SceneGraph* addModel(const Model& new_model);
    SceneGraph* addTransformation(const AffineTransformation& new_transformation);
    
    static std::vector<Model> getTransformedModels(SceneGraph& root);
};

} // Rendering3D
} // Impact
