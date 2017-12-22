#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Ray.hpp"
#include "TriangleMesh.hpp"
#include "AffineTransformation.hpp"
#include "Material.hpp"
#include "Color.hpp"
#include "Texture.hpp"
#include <vector>

namespace Impact {
namespace Rendering3D {

class Model {

private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;
	typedef Geometry3D::Ray Ray;
	typedef Geometry3D::TriangleMesh TriangleMesh;
	typedef Geometry3D::AffineTransformation AffineTransformation;

protected:
    const TriangleMesh* _mesh;
    const Material* _material;
	AffineTransformation _transformation;
	const Texture* _color_texture = nullptr;
	const Texture* _normal_map = nullptr;
	const Texture* _displacement_map = nullptr;
	imp_float _displacement_scale;

	bool _has_texture = false;
    
    //static std::vector<BlinnPhongMaterial> SceneObject::_getMtlFileData(const std::string& filename);

public:
	imp_uint id = -1;

    bool uses_direct_lighting = true;
	
	bool casts_shadows = true;
	bool shadows_toggable = true;
	
    bool render_faces = true;
    bool render_edges = false;

    bool remove_hidden_faces = true;
    bool perform_clipping = true;
	
	bool is_visible = true;
	bool is_currently_visible = is_visible;

    Model(const TriangleMesh* new_mesh,
		  const Material* new_material,
		  const AffineTransformation& new_transformation);

    Model(const TriangleMesh* new_mesh,
		  const Material* new_material);

	void setColorTexture(const Texture* texture);
	void setNormalMap(const Texture* normal_map);
	void setDisplacementMap(const Texture* displacement_map, imp_float displacement_scale);

	void applyTransformation(const AffineTransformation& transformation);

	virtual TriangleMesh getTransformedMesh() const;
	virtual TriangleMesh getTransformedMesh(const AffineTransformation& additional_transformation) const;

	const TriangleMesh* getMesh() const;

	const Material* getMaterial() const;

	const AffineTransformation& getTransformation() const;

	bool hasTexture() const;
	bool hasColorTexture() const;
	bool hasNormalMap() const;
	bool hasDisplacementMap() const;
	
	const Texture* getColorTexture() const;
	const Texture* getNormalMap() const;
	const Texture* getDisplacementMap() const;
	imp_float getDisplacementScale() const;
};

} // Rendering3D
} // Impact
