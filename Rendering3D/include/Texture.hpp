#pragma once
#include "precision.hpp"
#include "Point2.hpp"
#include "Vector2.hpp"
#include "Color.hpp"
#include <string>

namespace Impact {
namespace Rendering3D {

class Texture {

private:
	typedef Geometry2D::Point Point2;
	typedef Geometry2D::Vector Vector2;

protected:

	imp_uint _width, _height, _n_pixels, _n_components, _size;
    imp_float* _pixel_buffer;

public:
	enum ValueRange
	{
		COLOR_TEXTURE = 0,
		NORMAL_MAP = 1,
		DISPLACEMENT_MAP = 1
	};

	Texture(const std::string& filename, ValueRange range_type);
	Texture(imp_float* pixel_buffer, imp_uint width, imp_uint height, imp_uint n_components);
	~Texture();

	void write(const std::string& filename, ValueRange range_type) const;

	Color getColor(const Point2& uv_coord) const;
	Color getNormalDirectionWeights(const Point2& uv_coord) const;
	imp_float getDisplacementValue(const Point2& uv_coord) const;
	
    imp_uint getWidth() const;
    imp_uint getHeight() const;
	imp_uint getNumberOfComponents() const;

    const imp_float* getRawPixelArray() const;
};

} // Rendering3D
} // Impact
