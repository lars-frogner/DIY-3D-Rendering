#include "Animator.hpp"
#include <glut.h>
#include <freeglut.h>
#include <cassert>

namespace Impact {
namespace Rendering3D {

Animator::Animator(const Simulator& new_simulator,
				   const Scene& new_scene,
				   const LightContainer& new_lights)
	: _simulator(new_simulator),
	  _scene(new_scene),
	  _lights(new_lights),
	  _meshes() {}

void Animator::addMesh(const RenderableTriangleMesh& mesh)
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

void Animator::removeMesh(imp_uint idx)
{
	assert(idx < getNumberOfMeshes());

	if (_meshes[idx].is_dynamic)
		_n_dynamic_meshes--;

	_meshes.erase(_meshes.begin() + idx);
}

void Animator::addDynamicBox(const Box& box, const BlinnPhongMaterial& material)
{
	RenderableTriangleMesh mesh = RenderableTriangleMesh::box(box);
	mesh.is_dynamic = true;
	mesh.setMaterial(material);
	mesh.computeNormals();

	addMesh(mesh);

	_simulator.addBox(box);
}

void Animator::initialize()
{
	_keys_pressed['w'] = false;
	_keys_pressed['a'] = false;
	_keys_pressed['s'] = false;
	_keys_pressed['d'] = false;

	const Image& image = getImage();
	_image_center_x = static_cast<int>(image.getWidth())/2;
	_image_center_y = static_cast<int>(image.getHeight())/2;

	glutSetCursor(GLUT_CURSOR_NONE);

	_simulator.initialize();

	assert(_simulator.getNumberOfBoxes() == _n_dynamic_meshes);
}

void Animator::_transformDynamicMeshes(const std::vector<AffineTransformation>& transformations)
{
	assert(transformations.size() == _n_dynamic_meshes);

	for (imp_uint idx = 0; idx < getNumberOfMeshes(); idx++)
	{
		RenderableTriangleMesh& mesh = _meshes[idx];

		if (mesh.is_dynamic)
			mesh.applyTransformation(transformations[idx]);
	}
}

void Animator::updateScene()
{
	_simulator.step();
	_transformDynamicMeshes(_simulator.getTransformations());

	moveCamera();

	if (_camera_moved)
	{
		_scene.transformCameraLookRay(_camera_look_ray_transformation);
		_camera_look_ray_transformation.setToIdentity();
		glutWarpPointer(_image_center_x, _image_center_y);
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

void Animator::startCameraMove(unsigned char key)
{
	if (key == 'w' || key == 'a' || key == 's' || key == 'd' || key == 'q' || key == 'e')
	{
		_keys_pressed[key] = true;
	}
}

void Animator::stopCameraMove(unsigned char key)
{
	if (key == 'w' || key == 'a' || key == 's' || key == 'd' || key == 'q' || key == 'e')
	{
		_keys_pressed[key] = false;
	}
}

void Animator::moveCamera()
{
	imp_float du = 0.0, dv = 0.0, dw = 0.0;

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

void Animator::rotateCamera(int x, int y)
{
	imp_float dphi = -static_cast<imp_float>(x - _image_center_x)*camera_rotation_speed;
	imp_float dtheta = -static_cast<imp_float>(y - _image_center_y)*camera_rotation_speed;

	const CoordinateFrame& cframe = _scene.getCamera().getCoordinateFrame();

	_camera_look_ray_transformation = AffineTransformation::rotationAboutRay(Ray(cframe.origin, cframe.basis_1), dtheta)
									  *AffineTransformation::rotationAboutRay(Ray(cframe.origin, cframe.basis_2), dphi)
									  *_camera_look_ray_transformation;
	
	_camera_moved = true;
}

void Animator::drawInfo() const
{
	_renderString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, "test");
}

imp_uint Animator::getNumberOfMeshes() const
{
	return static_cast<imp_uint>(_meshes.size());
}

imp_uint Animator::getNumberOfStaticMeshes() const
{
	return getNumberOfMeshes() - getNumberOfDynamicMeshes();
}

imp_uint Animator::getNumberOfDynamicMeshes() const
{
	return _n_dynamic_meshes;
}

const RenderableTriangleMesh& Animator::getMesh(imp_uint idx) const
{
	assert(idx < getNumberOfMeshes());
	return _meshes[idx];
}

RenderableTriangleMesh& Animator::getMesh(imp_uint idx)
{
	assert(idx < getNumberOfMeshes());
	return _meshes[idx];
}

const Image& Animator::getImage() const
{
	return _scene.getImage();
}

void Animator::_renderString(int x, int y, void* glut_font, const std::string& text)
{
	glRasterPos2i(x, y);
	glutBitmapString(glut_font, (const unsigned char*)text.c_str());
}

} // Rendering3D
} // Impact
