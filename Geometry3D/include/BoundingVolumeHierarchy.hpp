#pragma once
#include "precision.hpp"
#include "Ray.hpp"
#include "AxisAlignedBox.hpp"
#include <vector>
#include <memory>
#include <cassert>
#include <queue>

namespace Impact {
namespace Geometry3D {
    
class TriangleMesh;

class BVHNode;

class BoundingVolumeHierarchy {

private:
    typedef std::shared_ptr<BVHNode> node_ptr;

    node_ptr _root_node;

public:
    BoundingVolumeHierarchy();
    BoundingVolumeHierarchy(const AxisAlignedBox& bounding_volume, const std::vector< AABBContainer >& objects);
	BoundingVolumeHierarchy(const BoundingVolumeHierarchy& other);
	BoundingVolumeHierarchy& operator=(const BoundingVolumeHierarchy& other);

    imp_float evaluateRayIntersection(const TriangleMesh& mesh, const Ray& ray, imp_uint& intersected_face_idx) const;
	imp_float evaluateRayIntersection(const std::vector<TriangleMesh>& meshes, const Ray& ray, imp_uint& intersected_mesh_idx, imp_uint& intersected_face_idx) const;

    std::vector<imp_uint> getPotentiallyIntersectedObjectIDs(const Ray& ray) const;
};

struct BVHQueueElement 
{ 
    typedef std::shared_ptr<BVHNode> node_ptr;

    node_ptr node;
    imp_float distance;

    BVHQueueElement(node_ptr new_node, imp_float new_distance)
        : node(new_node), distance(new_distance) {} 

    friend bool operator<(const BVHQueueElement& first, const BVHQueueElement& second)
    {
        return first.distance > second.distance;
    } 
};

} // Geometry3D
} // Impact
