#pragma once
#include "precision.hpp"
#include "RenderableTriangleMesh.hpp"
#include "BlinnPhongMaterial.hpp"
#include "LightContainer.hpp"
#include "Image.hpp"
#include "Scene.hpp"
#include "Simulator.hpp"
#include "Vector3.hpp"
#include "Ray.hpp"
#include "Box.hpp"
#include "CoordinateFrame.hpp"
#include "AffineTransformation.hpp"
#include <vector>
#include <map>
#include <cassert>

namespace Impact {
namespace Rendering3D {

class Animator {

private:
	typedef Geometry3D::Vector Vector;
	typedef Geometry3D::Ray Ray;
	typedef Geometry3D::Box Box;
	typedef Geometry3D::CoordinateFrame CoordinateFrame;
	typedef Geometry3D::AffineTransformation AffineTransformation;

	Simulator _simulator;
	Scene _scene;
	std::vector<RenderableTriangleMesh> _meshes;
	LightContainer _lights;

	imp_uint _n_dynamic_meshes = 0;

	std::map<unsigned char, bool> _keys_pressed;
	bool _camera_moved = false;
	int _image_center_x, _image_center_y;
	AffineTransformation _camera_look_ray_transformation;

	void _transformDynamicMeshes(const std::vector<AffineTransformation>& transformations);

	static void _renderString(int x, int y, void* glut_font, const std::string& text);

public:
	int render_mode = 0;
	imp_float camera_movement_speed = static_cast<imp_float>(0.1);
	imp_float camera_rotation_speed = static_cast<imp_float>(0.001);

	Animator(const Simulator& new_simulator,
			 const Scene& new_scene,
			 const LightContainer& new_lights);

	void addMesh(const RenderableTriangleMesh& mesh);
	void removeMesh(imp_uint idx);

	void addDynamicBox(const Box& box, const BlinnPhongMaterial& material);

	void initialize();

	void updateScene();

	void startCameraMove(unsigned char key);
	void stopCameraMove(unsigned char key);
	void moveCamera();
	void rotateCamera(int x, int y);

	void drawInfo() const;

	imp_uint getNumberOfMeshes() const;
	imp_uint getNumberOfStaticMeshes() const;
	imp_uint getNumberOfDynamicMeshes() const;
	
	RenderableTriangleMesh& getMesh(imp_uint idx);
	const RenderableTriangleMesh& getMesh(imp_uint idx) const;

	const Image& getImage() const;
};

} // Rendering3D
} // Impact
