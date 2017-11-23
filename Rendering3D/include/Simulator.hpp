#pragma once
#include "precision.hpp"
#include "Vector3.hpp"
#include "Ray.hpp"
#include "Box.hpp"
#include "AffineTransformation.hpp"
#include <vector>

namespace Impact {
namespace Rendering3D {

class Simulator {

private:
	typedef Geometry3D::Vector Vector;
	typedef Geometry3D::Ray Ray;
	typedef Geometry3D::Box Box;
	typedef Geometry3D::AffineTransformation AffineTransformation;

	std::vector<AffineTransformation> _transformations;
	std::vector<Box> _boxes;

public:
	Simulator();

	void addBox(const Box& box);
	void removeBox(imp_uint idx);

	void initialize();

	void step();

	imp_uint getNumberOfBoxes() const;

	const std::vector<AffineTransformation>& getTransformations() const;
};

} // Rendering3D
} // Impact
