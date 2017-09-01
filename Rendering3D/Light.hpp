#pragma once
#define _USE_MATH_DEFINES
#include <assert.h>
#include "../Geometry3D/Point.hpp"
#include "../Geometry3D/Vector.hpp"
#include "../Geometry3D/Vector4.hpp"
#include "../Geometry3D/CoordinateFrame.hpp"
#include "../Transformations3D/LinearTransformation.hpp"
#include "../Transformations3D/AffineTransformation.hpp"
#include "Color.hpp"

namespace Rendering3D {

template <typename F>
class Light {

private:
    typedef Geometry3D::Point<F> Point;
    typedef Geometry3D::Vector<F> Vector;
    typedef Geometry3D::Vector4<F> Vector4;
    typedef Geometry3D::CoordinateFrame<F> CoordinateFrame;
    typedef Transformations3D::LinearTransformation<F> LinearTransformation;
    typedef Transformations3D::AffineTransformation<F> AffineTransformation;

public:
    virtual CoordinateFrame getCoordinateFrame() const = 0;
    virtual F getSurfaceArea() const = 0;
    virtual size_t getNumberOfSamples() const = 0;

    virtual Vector4 getRandomPoint() const = 0;
    virtual Biradiance getBiradiance(const Vector4& source_point,
                                     const Point& surface_point) const = 0;

    virtual Power getTotalPower() const = 0;
    //virtual Color getEmittedPhoton(Point& photon_origin,
    //                               Vector& photon_direction) const = 0;

    virtual void applyTransformation(const LinearTransformation& transformation) = 0;
    virtual void applyTransformation(const AffineTransformation& transformation) = 0;
};

} //Rendering3D
