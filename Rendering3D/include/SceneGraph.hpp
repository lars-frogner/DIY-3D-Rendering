#pragma once
#include "precision.hpp"
#include "Image.hpp"
#include "RenderableTriangleMesh.hpp"
#include "Transformation.hpp"
#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"
#include <vector>

namespace Impact {
namespace Rendering3D {

class SceneGraph {

private:
    typedef Geometry3D::Transformation Transformation;
    typedef Geometry3D::LinearTransformation LinearTransformation;
    typedef Geometry3D::AffineTransformation AffineTransformation;

    SceneGraph* _parent_ptr = NULL;
    std::vector<SceneGraph*> _child_ptrs;
    
    RenderableTriangleMesh* _object_ptr = NULL;
    Transformation* _transformation_ptr = NULL;

    bool _is_transformation_node;

    SceneGraph(SceneGraph* new_parent_ptr,
               RenderableTriangleMesh* new_object_ptr);

    SceneGraph(SceneGraph* new_parent_ptr,
               Transformation* new_transformation_ptr);

    static void _processNode(SceneGraph& node,
                             std::vector<Transformation*>& transformation_stack,
                             std::vector<RenderableTriangleMesh>& object_stack);

public:
    SceneGraph(const RenderableTriangleMesh& root_object);
    SceneGraph(const LinearTransformation& root_transformation);
    SceneGraph(const AffineTransformation& root_transformation);

    SceneGraph(const SceneGraph& other) = delete;
    ~SceneGraph();
    SceneGraph& operator=(const SceneGraph& other) = delete;
    
    SceneGraph* addObject(const RenderableTriangleMesh& new_object);
    SceneGraph* addTransformation(const LinearTransformation& new_transformation);
    SceneGraph* addTransformation(const AffineTransformation& new_transformation);
    
    static std::vector<RenderableTriangleMesh> getTransformedObjects(SceneGraph& root);
};

} // Rendering3D
} // Impact
