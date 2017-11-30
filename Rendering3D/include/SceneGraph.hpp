#pragma once
#include "precision.hpp"
#include "Image.hpp"
#include "TriangleMesh.hpp"
#include "Transformation.hpp"
#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"
#include <vector>

namespace Impact {
namespace Rendering3D {

class SceneGraph {

private:
    typedef Geometry3D::TriangleMesh TriangleMesh;
    typedef Geometry3D::Transformation Transformation;
    typedef Geometry3D::LinearTransformation LinearTransformation;
    typedef Geometry3D::AffineTransformation AffineTransformation;

    SceneGraph* _parent_ptr = NULL;
    std::vector<SceneGraph*> _child_ptrs;
    
    TriangleMesh* _object_ptr = NULL;
    Transformation* _transformation_ptr = NULL;

    bool _is_transformation_node;

    SceneGraph(SceneGraph* new_parent_ptr,
               TriangleMesh* new_object_ptr);

    SceneGraph(SceneGraph* new_parent_ptr,
               Transformation* new_transformation_ptr);

    static void _processNode(SceneGraph& node,
                             std::vector<Transformation*>& transformation_stack,
                             std::vector<TriangleMesh>& object_stack);

public:
    SceneGraph(const TriangleMesh& root_object);
    SceneGraph(const LinearTransformation& root_transformation);
    SceneGraph(const AffineTransformation& root_transformation);

    SceneGraph(const SceneGraph& other) = delete;
    ~SceneGraph();
    SceneGraph& operator=(const SceneGraph& other) = delete;
    
    SceneGraph* addObject(const TriangleMesh& new_object);
    SceneGraph* addTransformation(const LinearTransformation& new_transformation);
    SceneGraph* addTransformation(const AffineTransformation& new_transformation);
    
    static std::vector<TriangleMesh> getTransformedObjects(SceneGraph& root);
};

} // Rendering3D
} // Impact
