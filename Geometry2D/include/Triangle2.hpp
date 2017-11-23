#pragma once
#include "precision.hpp"
#include "Point2.hpp"
#include "Vector2.hpp"
#include "AxisAlignedRectangle.hpp"
#include <string>

namespace Impact {
namespace Geometry2D {

class Triangle {

private:
    Point _A, _B, _C;
    Vector _AB, _AC;
    imp_float _normalization;

public:
    Triangle(const Point& new_A,
             const Point& new_B,
             const Point& new_C);

    const Point& getPointA() const;
    const Point& getPointB() const;
    const Point& getPointC() const;

    void getBarycentricCoordinates(const Point X,
                                   imp_float& alpha, imp_float& beta, imp_float& gamma) const;

    Triangle& setPointA(const Point& new_A);
    Triangle& setPointB(const Point& new_B);
    Triangle& setPointC(const Point& new_C);
    Triangle& setPoints(const Point& new_A,
                        const Point& new_B,
                        const Point& new_C);

    Triangle& translate(imp_float dx, imp_float dy);
    Triangle& translate(const Vector& displacement);

    Triangle& precomputeBarycentricQuantities();
    
    AxisAlignedRectangle getAABR() const;
    AxisAlignedRectangle getAABR(const Point& lower_corner_limit,
                                 const Point& upper_corner_limit) const;

    Point getCentroid() const;

    std::string toString() const;
};

} // Geometry2D
} // Impact
