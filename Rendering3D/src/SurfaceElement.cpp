#include "SurfaceElement.hpp"
#include "Texture.hpp"

namespace Impact {
namespace Rendering3D {

void SurfaceElement::modifyShadingData(const TriangleMesh& mesh)
{
	if (model->hasTexture())
	{
		shading.color = model->getTexture()->getColor(shading.texture_coordinate);
	}

	if (model->hasBumpMap())
	{
	}

	if (model->hasDisplacementMap())
	{
	}
}

} // Rendering3D
} // Impact
