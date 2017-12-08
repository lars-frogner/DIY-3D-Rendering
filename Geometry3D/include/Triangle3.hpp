#pragma once
#include "precision.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Plane.hpp"
#include "AxisAlignedBox.hpp"
#include <string>

namespace Impact {
namespace Geometry3D {

class Triangle {

private:
    Point _A, _B, _C;
    Vector _normal, _AB_normal, _AC_normal;
    imp_float _area;
	bool _is_degenerate = false;

public:
    Triangle(const Point& new_A,
             const Point& new_B,
             const Point& new_C);

    Point operator()(imp_float alpha, imp_float beta, imp_float gamma) const;
    Point operator()(imp_float s, imp_float t) const;

    const Point& getPointA() const;
    const Point& getPointB() const;
    const Point& getPointC() const;
    const Vector& getNormalVector() const;
    imp_float getArea() const;

    static Vector areaVector(const Point& A,
                             const Point& B,
                             const Point& C);

    static Vector getNormalVector(const Point& A,
								  const Point& B,
								  const Point& C);

    void getBarycentricCoordinatesInside(const Point X,
                                         imp_float& alpha, imp_float& beta, imp_float& gamma) const;

    void getBarycentricCoordinates(const Point X,
                                   imp_float& alpha, imp_float& beta, imp_float& gamma) const;

    Triangle& setPointA(const Point& new_A);
    Triangle& setPointB(const Point& new_B);
    Triangle& setPointC(const Point& new_C);
    Triangle& setPoints(const Point& new_A,
                        const Point& new_B,
                        const Point& new_C);

    Triangle& translate(imp_float dx, imp_float dy, imp_float dz);
    Triangle& translate(const Vector& displacement);

    Triangle& computeNormalVectors();
    
	bool isDegenerate() const;

    Plane getPlane() const;
    Plane getPlaneNoBasis() const;
    AxisAlignedBox getAABB() const;
    Point getCentroid() const;

    std::string toString() const;
};

} // Geometry3D
} // Impact
