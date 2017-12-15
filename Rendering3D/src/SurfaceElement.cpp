#include "SurfaceElement.hpp"
#include "Texture.hpp"

namespace Impact {
namespace Rendering3D {

void SurfaceElement::computeTextureColor()
{
	shading.color = model->getColorTexture()->getColor(shading.texture_coordinate);
}

void SurfaceElement::computeBumpMappedNormal()
{
	shading.normal = (shading.normal + _bump_values.x*shading.tangent + _bump_values.y*shading.bitangent).getNormalized();
}

void SurfaceElement::computeDisplacementMappedPosition()
{
}

bool SurfaceElement::evaluateBumpMapping()
{
	if (model->hasBumpMap())
	{
		_bump_values = model->getBumpMap()->getBumpValues(shading.texture_coordinate);

		if (abs(_bump_values.x) < 1.0e-3f && abs(_bump_values.y) < 1.0e-3f)
			return false;

		return true;
	}

	return false;
}

} // Rendering3D
} // Impact
