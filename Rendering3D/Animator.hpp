#pragma once
#include <glut.h>
#include <freeglut.h>
#include <vector>
#include <map>
#include <assert.h>
#include "../Geometry3D/Vector.hpp"
#include "../Geometry3D/Ray.hpp"
#include "../Geometry3D/Box.hpp"
#include "../Geometry3D/TriangleMesh.hpp"
#include "../Geometry3D/CoordinateFrame.hpp"
#include "../Transformations3D/AffineTransformation.hpp"
#include "BlinnPhongMaterial.hpp"
#include "LightContainer.hpp"
#include "Image.hpp"
#include "Scene.hpp"
#include "Simulator.hpp"

namespace Rendering3D {

template <typename F>
class Animator {

private:
	typedef Geometry3D::Vector<F> Vector;
	typedef Geometry3D::Ray<F> Ray;
	typedef Geometry3D::Box<F> Box;
	typedef Geometry3D::TriangleMesh<F> TriangleMesh;
	typedef Geometry3D::CoordinateFrame<F> CoordinateFrame;
	typedef Transformations3D::AffineTransformation<F> AffineTransformation;

	Simulator<F> _simulator;
	Scene<F> _scene;
	std::vector<TriangleMesh> _meshes;
	LightContainer<F> _lights;

	size_t _n_dynamic_meshes = 0;

	std::map<unsigned char, bool> _keys_pressed;
	bool _camera_moved = false;
	int _image_center_x, _image_center_y;
	AffineTransformation _camera_look_ray_transformation;

	void _transformDynamicMeshes(const std::vector<AffineTransformation>& transformations);

	static void _renderString(int x, int y, void* glut_font, const std::string& text);

public:
	int render_mode = 0;
	F camera_movement_speed = 0.1;
	F camera_rotation_speed = 0.001;

	Animator<F>(const Simulator<F>& new_simulator,
				const Scene<F>& new_scene,
				const LightContainer<F>& new_lights);

	void addMesh(const TriangleMesh& mesh);
	void removeMesh(size_t idx);

	void addDynamicBox(const Box& box, const BlinnPhongMaterial<F>& material);

	void initialize();

	void updateScene();

	void startCameraMove(unsigned char key);
	void stopCameraMove(unsigned char key);
	void moveCamera();
	void rotateCamera(int x, int y);

	void drawInfo() const;

	size_t getNumberOfMeshes() const;
	size_t getNumberOfStaticMeshes() const;
	size_t getNumberOfDynamicMeshes() const;
	
	TriangleMesh& getMesh(size_t idx);
	const TriangleMesh& getMesh(size_t idx) const;

	const Image<F>& getImage() const;
};

template <typename F>
Animator<F>::Animator(const Simulator<F>& new_simulator,
					  const Scene<F>& new_scene,
					  const LightContainer<F>& new_lights)
	: _simulator(new_simulator),
	  _scene(new_scene),
	  _lights(new_lights),
	  _meshes() {}

template <typename F>
void Animator<F>::addMesh(const TriangleMesh& mesh)
{
	if (mesh.is_dynamic)
	{
		_n_dynamic_meshes++;
	}

	_meshes.push_back(mesh);

	if (!mesh.hasNormals())
	{
		_meshes.back().computeNormals();
	}
}

template <typename F>
void Animator<F>::removeMesh(size_t idx)
{
	assert(idx < getNumberOfMeshes());

	if (_meshes[idx].is_dynamic)
		_n_dynamic_meshes--;

	_meshes.erase(_meshes.begin() + idx);
}

template <typename F>
void Animator<F>::addDynamicBox(const Box& box, const BlinnPhongMaterial<F>& material)
{
	TriangleMesh mesh = TriangleMesh::box(box);
	mesh.is_dynamic = true;
	mesh.setMaterial(material);
	mesh.computeNormals();

	addMesh(mesh);

	_simulator.addBox(box);
}

template <typename F>
void Animator<F>::initialize()
{
	_keys_pressed['w'] = false;
	_keys_pressed['a'] = false;
	_keys_pressed['s'] = false;
	_keys_pressed['d'] = false;

	const Image<F>& image = getImage();
	_image_center_x = static_cast<int>(image.getWidth())/2;
	_image_center_y = static_cast<int>(image.getHeight())/2;

	glutSetCursor(GLUT_CURSOR_NONE);

	_simulator.initialize();

	assert(_simulator.getNumberOfBoxes() == _n_dynamic_meshes);
}

template <typename F>
void Animator<F>::_transformDynamicMeshes(const std::vector<AffineTransformation>& transformations)
{
	assert(transformations.size() == _n_dynamic_meshes);

	for (size_t idx = 0; idx < getNumberOfMeshes(); idx++)
	{
		TriangleMesh& mesh = _meshes[idx];

		if (mesh.is_dynamic)
			mesh.applyTransformation(transformations[idx]);
	}
}

template <typename F>
void Animator<F>::updateScene()
{
	_simulator.step();
	_transformDynamicMeshes(_simulator.getTransformations());

	moveCamera();

	if (_camera_moved)
	{
		_scene.transformCameraLookRay(_camera_look_ray_transformation);
		_camera_look_ray_transformation.setToIdentity();
		_camera_moved = false;
	}

	if (render_mode == 0)
	{
		_scene.renderDirect(_meshes, _lights);
	}
	else if (render_mode == 1)
	{
		_scene.rayTrace(_meshes, _lights);
	}
	else
	{
		std::cerr << "Warning: rendering disabled" << std::endl;
	}
}

template <typename F>
void Animator<F>::startCameraMove(unsigned char key)
{
	if (key == 'w' || key == 'a' || key == 's' || key == 'd' || key == 'q' || key == 'e')
	{
		_keys_pressed[key] = true;
	}
}

template <typename F>
void Animator<F>::stopCameraMove(unsigned char key)
{
	if (key == 'w' || key == 'a' || key == 's' || key == 'd' || key == 'q' || key == 'e')
	{
		_keys_pressed[key] = false;
	}
}

template <typename F>
void Animator<F>::moveCamera()
{
	F du = 0.0, dv = 0.0, dw = 0.0;

	if (_keys_pressed['a'])
		du += camera_movement_speed;
	if (_keys_pressed['d'])
		du -= camera_movement_speed;
	if (_keys_pressed['q'])
		dv += camera_movement_speed;
	if (_keys_pressed['e'])
		dv -= camera_movement_speed;
	if (_keys_pressed['w'])
		dw += camera_movement_speed;
	if (_keys_pressed['s'])
		dw -= camera_movement_speed;

	if (du != 0 || dv != 0 || dw != 0)
	{
		const CoordinateFrame& cframe = _scene.getCamera().getCoordinateFrame();

		_camera_look_ray_transformation = AffineTransformation::translation(-du*cframe.basis_1 + dv*cframe.basis_2 - dw*cframe.basis_3)
										  *_camera_look_ray_transformation;

		_camera_moved = true;
	}
}

template <typename F>
void Animator<F>::rotateCamera(int x, int y)
{
	F dphi = -static_cast<F>(x - _image_center_x)*camera_rotation_speed;
	F dtheta = -static_cast<F>(y - _image_center_y)*camera_rotation_speed;

	const CoordinateFrame& cframe = _scene.getCamera().getCoordinateFrame();

	_camera_look_ray_transformation = AffineTransformation::rotationAboutRay(Ray(cframe.origin, cframe.basis_1), dtheta)
									  *AffineTransformation::rotationAboutRay(Ray(cframe.origin, cframe.basis_2), dphi)
									  *_camera_look_ray_transformation;

	glutWarpPointer(_image_center_x, _image_center_y);

	_camera_moved = true;
}

template <typename F>
void Animator<F>::drawInfo() const
{
	_renderString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, "test");
}

template <typename F>
size_t Animator<F>::getNumberOfMeshes() const
{
	return _meshes.size();
}

template <typename F>
size_t Animator<F>::getNumberOfStaticMeshes() const
{
	return getNumberOfMeshes() - getNumberOfDynamicMeshes();
}

template <typename F>
size_t Animator<F>::getNumberOfDynamicMeshes() const
{
	return _n_dynamic_meshes;
}

template <typename F>
const Geometry3D::TriangleMesh<F>& Animator<F>::getMesh(size_t idx) const
{
	assert(idx < getNumberOfMeshes());
	return _meshes[idx];
}

template <typename F>
Geometry3D::TriangleMesh<F>& Animator<F>::getMesh(size_t idx)
{
	assert(idx < getNumberOfMeshes());
	return _meshes[idx];
}

template <typename F>
const Image<F>& Animator<F>::getImage() const
{
	return _scene.getImage();
}

template <typename F>
void Animator<F>::_renderString(int x, int y, void* glut_font, const std::string& text)
{
	glRasterPos2i(x, y);
	glutBitmapString(glut_font, (const unsigned char*)text.c_str());
}

} // Rendering3D