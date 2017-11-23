#include "ParticleLink.hpp"

namespace Impact {
namespace Physics3D {

Geometry3D::Vector ParticleLink::getCurrentSeparation() const
{
	return _particle_2->getPosition() - _particle_1->getPosition();
}

imp_float ParticleLink::getCurrentDistance() const
{
	Vector separation = _particle_2->getPosition() - _particle_1->getPosition();
	return separation.getLength();
}

} // Physics3D
} // Impact
