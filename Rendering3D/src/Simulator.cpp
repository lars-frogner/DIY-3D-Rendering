#include "Simulator.hpp"

namespace Impact {
namespace Rendering3D {

Simulator::Simulator() {}

void Simulator::addBox(const Box& box)
{
	_boxes.push_back(box);
}

void Simulator::removeBox(imp_uint idx)
{
	_boxes.erase(_boxes.begin() + idx);
}

void Simulator::initialize()
{
	imp_uint n_boxes = getNumberOfBoxes();

	_transformations.resize(n_boxes);

	for (imp_uint idx = 0; idx < n_boxes; idx++)
	{
		_transformations[idx].setToIdentity();
	}
}

void Simulator::step()
{
	Vector displacement, axis;
	imp_float angle;

	for (imp_uint idx = 0; idx < getNumberOfBoxes(); idx++)
	{
		Box& box = _boxes[idx];
		AffineTransformation& transformation = _transformations[idx];

		displacement = Vector(0, 0, -0.01f*0);
		axis = Vector(0.5f, 1.0f, 0.3f).normalize();
		angle = 0.01f;

		transformation = AffineTransformation::rotationAboutRay(Ray(box.getCenter(), axis), angle)*AffineTransformation::translation(displacement);

		box = transformation*box;
	}
}

imp_uint Simulator::getNumberOfBoxes() const
{
	return static_cast<imp_uint>(_boxes.size());
}

const std::vector< Geometry3D::AffineTransformation >& Simulator::getTransformations() const
{
	return _transformations;
}

} // Rendering3D
} // Impact
