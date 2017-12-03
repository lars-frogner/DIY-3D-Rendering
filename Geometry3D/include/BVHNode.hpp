#pragma once
#include "precision.hpp"
#include "Ray.hpp"
#include "AxisAlignedBox.hpp"
#include "BoundingVolumeHierarchy.hpp"
#include <vector>
#include <memory>

namespace Impact {
namespace Geometry3D {

class BVHNode {
    
friend BoundingVolumeHierarchy;

private:
    typedef std::shared_ptr<BVHNode> node_ptr;

protected:
    AxisAlignedBox _aabb;
    std::vector<AxisAlignedBox> _octants;
    AABBContainer _object;
    std::vector<node_ptr> _child_nodes;
    bool _has_children = false;

    BVHNode(const AxisAlignedBox& new_bounding_volume, const AABBContainer& new_object);
	BVHNode(const BVHNode& other);
	BVHNode& operator=(const BVHNode& other);

    void _insertObject(const AABBContainer& object);
    const AxisAlignedBox& _computeBoundingVolumes();

    std::vector<imp_uint> _getIntersectedObjectIDs(const Ray& ray) const;

public:
	//~BVHNode();
};

} // Geometry3D
} // Impact
