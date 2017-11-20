#pragma once
#include <vector>
#include "../Geometry3D/Vector.hpp"
#include "../Geometry3D/Ray.hpp"
#include "../Geometry3D/Box.hpp"
#include "../Transformations3D/AffineTransformation.hpp"

namespace Rendering3D {

template <typename F>
class Simulator {

private:
	typedef Geometry3D::Vector<F> Vector;
	typedef Geometry3D::Ray<F> Ray;
	typedef Geometry3D::Box<F> Box;
	typedef Transformations3D::AffineTransformation<F> AffineTransformation;

	std::vector<AffineTransformation> _transformations;
	std::vector<Box> _boxes;

public:
	Simulator<F>();

	void addBox(const Box& box);
	void removeBox(size_t idx);

	void initialize();

	void step();

	size_t getNumberOfBoxes() const;

	const std::vector<AffineTransformation>& getTransformations() const;
};

template <typename F>
Simulator<F>::Simulator() {}

template <typename F>
void Simulator<F>::addBox(const Box& box)
{
	_boxes.push_back(box);
}

template <typename F>
void Simulator<F>::removeBox(size_t idx)
{
	_boxes.erase(_boxes.begin() + idx);
}

template <typename F>
void Simulator<F>::initialize()
{
	size_t n_boxes = getNumberOfBoxes();

	_transformations.resize(n_boxes);

	for (size_t idx = 0; idx < n_boxes; idx++)
	{
		_transformations[idx].setToIdentity();
	}
}

template <typename F>
void Simulator<F>::step()
{
	Vector displacement, axis;
	F angle;

	for (size_t idx = 0; idx < getNumberOfBoxes(); idx++)
	{
		Box& box = _boxes[idx];
		AffineTransformation& transformation = _transformations[idx];

		displacement = Vector(0, 0, -0.01*0);
		axis = Vector(0.5, 1.0, 0.3).normalize();
		angle = 0.01;

		transformation = AffineTransformation::rotationAboutRay(Ray(box.getCenter(), axis), angle)*AffineTransformation::translation(displacement);

		box = transformation*box;
	}
}

template <typename F>
size_t Simulator<F>::getNumberOfBoxes() const
{
	return _boxes.size();
}

template <typename F>
const std::vector< Transformations3D::AffineTransformation<F> >& Simulator<F>::getTransformations() const
{
	return _transformations;
}

} // Rendering3D