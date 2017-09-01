#include <iostream>
#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"
#include "ProjectiveTransformation.hpp"

int main(int argc, char *argv[])
{
    typedef Geometry3D::Point<float> Point;
    typedef Geometry3D::Vector<float> Vector;
    typedef Transformations3D::Transformation<float> Transformation;
    typedef Transformations3D::LinearTransformation<float> LinearTransformation;
    typedef Transformations3D::AffineTransformation<float> AffineTransformation;
    typedef Transformations3D::ProjectiveTransformation<float> ProjectiveTransformation;

    LinearTransformation T1 = LinearTransformation::rotationFromXToY(2);
    LinearTransformation T2 = LinearTransformation::rotationFromYToZ(1);

    Transformation* T_ptr1 = &T1;
    Transformation* T_ptr2 = &T2;

    //T1.getMatrix().print();
    //((*T_ptr2)*(*T_ptr1))->getMatrix().print();

    /*
    AffineTransformation T = AffineTransformation::pointsToPoints(Point(0, 0, 0),
                                                                  Point(1, 0, 0),
                                                                  Point(1, 1, 0),
                                                                  Point(0, 1, 1),
                                                                  Point(2, 2, 0),
                                                                  Point(4, 1, 0),
                                                                  Point(3, 3, 0),
                                                                  Point(1, 2, 0));

    ProjectiveTransformation T2 = ProjectiveTransformation::pointsToPoints(Point(0, 0, 0),
                                                                           Point(1, 0, 0),
                                                                           Point(1, 1, 0),
                                                                           Point(0, 1, 1),
                                                                           Point(0, 0, 2),
                                                                           Point(2, 1, 1),
                                                                           Point(5, 1, 0),
                                                                           Point(3, 5, 2),
                                                                           Point(1, 2, 0),
                                                                           Point(9, 1, 3));*/

    /*T2.getMatrix().print();

    std::cout << (T2*Point(0, 0, 0)).toString() << std::endl;
    std::cout << (T2*Point(1, 0, 0)).toString() << std::endl;
    std::cout << (T2*Point(1, 1, 0)).toString() << std::endl;
    std::cout << (T2*Point(0, 1, 1)).toString() << std::endl;
    std::cout << (T2*Point(0, 0, 2)).toString() << std::endl;*/
}
