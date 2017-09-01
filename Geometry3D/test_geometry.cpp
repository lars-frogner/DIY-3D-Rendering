#include <iostream>
#include <math.h>
#include "Point.hpp"
#include "Vector.hpp"
#include "Ray.hpp"
#include "Plane.hpp"
#include "Sphere.hpp"
#include "AxisAlignedBox.hpp"
#include "Box.hpp"
#include "Triangle.hpp"
#include "TriangleMesh.hpp"
#include "affine_combinations.hpp"
#include "../Transformations3D/Transformation.hpp"

int main(int argc, char *argv[])
{
    const float PI_F = 3.14159265358979f;

    Geometry3D::Point<float> point_0;
    Geometry3D::Point<float> point_1(0.1f, 3.2f, 4.0f);
    Geometry3D::Vector<float> vector_0;
    Geometry3D::Vector<float> vector_1(1.1f, 2.2f, 3.3f);

    std::cout << "Point 0: " << point_0.toString() << std::endl;
    std::cout << "Point 1: " << point_1.toString() << std::endl;
    std::cout << "0.5 Point 0 + 0.5 Point 1: " << Geometry3D::getAffineCombination<float>(point_0, point_1, 0.5f, 0.5f).toString() << std::endl;
    std::cout << "0.2 Point 0 + 0.4 Point 1 + 0.4 (1.2, 3, 1.1): " << Geometry3D::getAffineCombination<float>(point_0, point_1, Geometry3D::Point<float>(1.2f, 3.0f, 1.1f), 0.2f, 0.4f, 0.4f).toString() << std::endl;
    std::cout << "Point 0: " << point_0.moveTo(3.3f, 0.2f, 9.8f).toString() << std::endl;
    std::cout << "Point 1: " << point_1.translate(1.3f, 0.3f, -7.1f).toString() << std::endl;
    std::cout << "Point 0: " << point_0.swap(point_1).toString() << std::endl;
    std::cout << "Point 1: " << point_1.toString() << std::endl;
    std::cout << "Point 0: " << point_0.useSmallestCoordinates(point_1).toString() << std::endl;
    std::cout << "Point 1: " << point_1.useLargestCoordinates(point_0).toString() << std::endl << std::endl;

    std::cout << "Vector 0: " << vector_0.toString() << std::endl;
    std::cout << "Vector 1: " << vector_1.toString() << std::endl << std::endl;

    point_0.moveTo(1.0f, 2.0f, 3.0f);
    point_1.moveTo(3.0f, 2.0f, 1.0f);
    std::cout << "Point 0: " << point_0.toString() << std::endl;
    std::cout << "Point 1: " << point_1.toString() << std::endl;
    std::cout << "Point 0 - Point 1: " << (point_0 - point_1).toString() << std::endl;
    std::cout << "Point 0 + Vector 1: " << (point_0 + vector_1).toString() << std::endl;
    std::cout << "Point 0 - Vector 1: " << (point_0 - vector_1).toString() << std::endl;
    point_0 += vector_1;
    std::cout << "Point 0 + Vector 1: " << point_0.toString() << std::endl;
    point_1 -= vector_1;
    std::cout << "Point 1 - Vector 1: " << point_1.toString() << std::endl;
    std::cout << "vector(Point 0): " << point_0.toVector().toString() << std::endl << std::endl;

    vector_0.setComponents(1.0f, 2.0f, 3.0f);
    vector_1.setComponents(3.0f, 2.0f, 1.0f);
    std::cout << "Vector 0: " << vector_0.toString() << std::endl;
    std::cout << "Vector 1: " << vector_1.toString() << std::endl;
    std::cout << "Vector 0 + Vector 1: " << (vector_0 + vector_1).toString() << std::endl;
    std::cout << "-Vector 0: " << (-vector_0).toString() << std::endl;
    std::cout << "Vector 0 - Vector 1: " << (vector_0 - vector_1).toString() << std::endl;
    vector_0 += vector_1;
    std::cout << "Vector 0 + Vector 1: " << vector_0.toString() << std::endl;
    vector_0 -= vector_1;
    std::cout << "Vector 0: " << vector_0.toString() << std::endl;
    std::cout << "5 Vector 0: " << (5.0f*vector_0).toString() << std::endl;
    std::cout << "1/5 Vector 0: " << (vector_0/5.0f).toString() << std::endl;
    vector_0 *= 5.0f;
    std::cout << "5 Vector 0: " << vector_0.toString() << std::endl;
    vector_0 /= 10.0f;
    std::cout << "1/2 Vector 0: " << vector_0.toString() << std::endl;
    vector_0.setComponents(0.0f, 2.0f, 3.0f);
    std::cout << "Vector 0: " << vector_0.toString() << std::endl;
    std::cout << "Vector 0 / 0: " << (vector_0/0.0f).toString() << std::endl;
    std::cout << "Vector 1 + Point 0: " << (vector_1 + point_0).toString() << std::endl;
    std::cout << "Vector 0: " << vector_0.toString() << std::endl;
    std::cout << "Vector 1: " << vector_1.toString() << std::endl;
    std::cout << "Vector 0: " << vector_0.swap(vector_1).toString() << std::endl;
    std::cout << "Vector 1: " << vector_1.toString() << std::endl << std::endl;

    vector_0.setComponents(1.0f, 2.0f, 3.0f);
    vector_1.setComponents(3.0f, 2.0f, 1.0f);
    std::cout << "Vector 0: " << vector_0.toString() << std::endl;
    std::cout << "Vector 1: " << vector_1.toString() << std::endl;
    std::cout << "len(Vector 0): " << vector_0.getLength() << std::endl;
    std::cout << "len(Vector 0)^2: " << vector_0.getSquaredLength() << std::endl;
    std::cout << "norm(Vector 0): " << vector_0.normalize().toString() << std::endl;
    std::cout << "norm(Vector 1): " << vector_1.getNormalized().toString() << std::endl;
    //vector_0.setComponents(0.0f, 0.0f, 0.0f);
    //std::cout << "Vector 0: " << vector_0.toString() << std::endl;
    //std::cout << "norm(Vector 0): " << vector_0.getNormalized().toString() << std::endl;
    vector_0.setComponents(1.0f, 2.0f, 3.0f);
    std::cout << "dot(Vector 0, Vector 1): " << vector_0.dot(vector_1) << std::endl;
    std::cout << "cross(Vector 0, Vector 1): " << vector_0.cross(vector_1).toString() << std::endl;
    std::cout << "(Vector 0).rotatexy(pi/2): " << vector_0.getRotatedFromXToY(PI_F/2.0f).toString() << std::endl;
    std::cout << "(Vector 0).rotatexy(pi/2): " << vector_0.getRotatedFromYToZ(PI_F/2.0f).toString() << std::endl;
    std::cout << "(Vector 0).rotatexy(pi/2): " << vector_0.getRotatedFromZToX(PI_F/2.0f).toString() << std::endl;
    vector_0.setComponents(NAN, 2.0f, 3.0f);
    std::cout << "Vector 0: " << vector_0.toString() << std::endl;
    std::cout << "minN(Vector 0): " << vector_0.getSmallestComponentNaNSafe() << std::endl;
    std::cout << "maxN(Vector 0): " << vector_0.getLargestComponentNaNSafe() << std::endl;
    vector_0.setComponents(4.0f, -2.0f, 2.0f);
    vector_1.setComponents(3.0f, 2.0f, 1.0f);
    std::cout << "Vector 0: " << vector_0.toString() << std::endl;
    std::cout << "Vector 1: " << vector_1.toString() << std::endl;
    Geometry3D::Vector<float>::sortComponentwise(vector_0, vector_1);
    std::cout << "sorted Vector 0: " << vector_0.toString() << std::endl;
    std::cout << "sorted Vector 1: " << vector_1.toString() << std::endl;
    std::cout << "unitnormal(Vector 0, Vector 1): " << vector_1.getUnitNormalWith(Geometry3D::Vector<float>(2.0f, 5.0f, 1.0f)).toString() << std::endl;
    std::cout << "projection(Vector 0, Vector 1): " << vector_0.getProjectedOn(vector_1).toString() << std::endl;
    std::cout << "(Vector 0).projectiononnormal(Vector 1, [2, 5, 1]): " << vector_0.getProjectedOnNormalTo(vector_1, Geometry3D::Vector<float>(2.0f, 5.0f, 1.0f)).toString() << std::endl;
    std::cout << "point(Vector 0): " << vector_0.toPoint().toString() << std::endl << std::endl;

    const size_t n_vertices = 3;
    const size_t n_faces = 1;
    float vertices[3*n_vertices] = {0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
    size_t faces[3*n_faces] = {0, 1, 2};
    Geometry3D::TriangleMesh<float> mesh(n_vertices, vertices, n_faces, faces);
    std::cout << "Vertices:\n" << mesh.getVerticesString() << std::endl;
    std::cout << "Faces:\n" << mesh.getFacesString() << std::endl;
    std::cout << "n_vertices: " << mesh.getNumberOfVertices() << std::endl;
    std::cout << "n_faces: " << mesh.getNumberOfFaces() << std::endl;
    mesh.addVertex(-1.0f, 0.5f, 0.0f);
    std::cout << "Vertices:\n" << mesh.getVerticesString() << std::endl;
    std::cout << "n_vertices: " << mesh.getNumberOfVertices() << std::endl;
    mesh.addFace(0, 2, 3);
    std::cout << "Faces:\n" << mesh.getFacesString() << std::endl;
    std::cout << "n_faces: " << mesh.getNumberOfFaces() << std::endl;
    //mesh.removeFace(4);
    //std::cout << "Faces:\n" << mesh.getFacesString() << std::endl;
    //std::cout << "n_faces: " << mesh.getNumberOfFaces() << std::endl;
    //mesh.removeVertex(1);
    //std::cout << "Vertices:\n" << mesh.getVerticesString() << std::endl;
    //std::cout << "Faces:\n" << mesh.getFacesString() << std::endl;
    std::cout << "Triangle 0: " << mesh.getFace(0).toString() << std::endl;

    float alpha, beta, gamma;
    Geometry3D::Triangle<float> triangle = mesh.getFace(0);
    triangle.getBarycentricCoordinates(Geometry3D::Point<float>(0.1f, 0.0f, 0.9f), alpha, beta, gamma);
    std::cout << triangle(alpha, beta, gamma).toString() << std::endl << std::endl;

    const size_t n_vertices2 = 4;
    const size_t n_faces2 = 2;
    float vertices2[3*n_vertices2] = {0.0f, 0.0f, 0.0f,  1.0f, 0.0f, 0.0f,  0.0f, 1.0f, 0.0f,  1.0f, 1.0f, 0.0f};
    size_t faces2[3*n_faces2] = {0, 1, 2,  2, 1, 3};
    Geometry3D::TriangleMesh<float> mesh2(n_vertices2, vertices2, n_faces2, faces2);
    std::cout << "Vertices:\n" << mesh2.getVerticesString() << std::endl;
    std::cout << "Faces:\n" << mesh2.getFacesString() << std::endl;
    std::cout << "Vertices:\n" << mesh2.get4SpaceVerticesString() << std::endl;
    mesh2.addVertex(2.0f, 0.0f, 0.0f);
    mesh2.addFace(1, 4, 3);
    std::cout << "Vertices:\n" << mesh2.getVerticesString() << std::endl;
    std::cout << "Faces:\n" << mesh2.getFacesString() << std::endl;
    std::cout << "Vertices:\n" << mesh2.get4SpaceVerticesString() << std::endl;
}
