#pragma once
#include <armadillo>

namespace Transformations3D {

// Forward declaration of ProjectiveTransformation class
template <typename F>
class ProjectiveTransformation;

template <typename F>
class Transformation {

public:
    virtual const arma::Mat<F>& getMatrix() const = 0;
    virtual std::string getTransformationType() const = 0;
};

} // Transformations3D
