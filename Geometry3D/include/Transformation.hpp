#pragma once
#include "precision.hpp"
#include <armadillo>

namespace Impact {
namespace Geometry3D {

class Transformation {

public:
    virtual const arma::Mat<imp_float>& getMatrix() const = 0;
    virtual std::string getTransformationType() const = 0;
};

} // Transformations3D
} // Impact
