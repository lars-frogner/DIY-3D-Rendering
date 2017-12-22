#include "SurfaceElement.hpp"
#include "Texture.hpp"

namespace Impact {
namespace Rendering3D {

void SurfaceElement::computeTextureColor()
{
	shading.color = model->getColorTexture()->getColor(shading.texture_coordinate);
}

void SurfaceElement::computeNormalMappedNormal()
{
	const Vector& normal = shading.tangent*_normal_weights.r + shading.bitangent*_normal_weights.g + shading.normal*_normal_weights.b;
	imp_float length = normal.getLength();
	if (length > 0)
		shading.normal = normal/length;
}

void SurfaceElement::computeDisplacementMappedPosition()
{
	shading.position += shading.normal*(model->getDisplacementScale())*(model->getDisplacementMap()->getDisplacementValue(shading.texture_coordinate));
}

bool SurfaceElement::evaluateNormalMapping()
{
	if (model->hasNormalMap())
	{
		_normal_weights = model->getNormalMap()->getNormalDirectionWeights(shading.texture_coordinate);

		if (_normal_weights.r == 0.0f && _normal_weights.g == 0.0f && _normal_weights.b == 1.0f)
			return false;

		return true;
	}

	return false;
}

} // Rendering3D
} // Impact
