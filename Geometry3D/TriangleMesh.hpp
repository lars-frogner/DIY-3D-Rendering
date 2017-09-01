#pragma once
#include <armadillo>
#include <vector>
#include <limits>
#include <string>
#include <iostream>
#include <sstream>
#include "Point.hpp"
#include "Triangle.hpp"
#include "Box.hpp"
#include "AxisAlignedBox.hpp"
#include "Vertex.hpp"
#include "../Geometry2D/Point.hpp"
#include "../Geometry2D/Triangle.hpp"
#include "../Transformations3D/LinearTransformation.hpp"
#include "../Transformations3D/AffineTransformation.hpp"
#include "../Transformations3D/ProjectiveTransformation.hpp"
#include "../Rendering3D/Color.hpp"
#include "../Rendering3D/Material.hpp"
#include "../Rendering3D/BlinnPhongMaterial.hpp"
#include "../Rendering3D/Light.hpp"
#include "../Rendering3D/Image.hpp"

namespace Rendering3D {

// Forward declaration of Scene class
template <typename F>
class Scene;

} // Rendering3D

namespace Geometry3D {

template <typename F>
class TriangleMesh {

private:
    typedef Geometry2D::Point<F> Point2;
    typedef Geometry2D::Triangle<F> Triangle2;
    typedef Transformations3D::LinearTransformation<F> LinearTransformation;
    typedef Transformations3D::AffineTransformation<F> AffineTransformation;
    typedef Transformations3D::ProjectiveTransformation<F> ProjectiveTransformation;
    typedef Rendering3D::Color Color;
    typedef Rendering3D::Radiance Radiance;
    typedef Rendering3D::Biradiance Biradiance;
    typedef Rendering3D::Material<F> Material;
    typedef Rendering3D::BlinnPhongMaterial<F> BlinnPhongMaterial;
    typedef Rendering3D::Light<F> Light;
    typedef Rendering3D::Image<F> Image;
    typedef Rendering3D::Scene<F> Scene;

    arma::Mat<F> _vertices;
    arma::Mat<size_t> _faces;
    arma::Mat<F> _normals;

    Material* _material = NULL;
    std::vector<Color> _colors;

    AxisAlignedBox<F> _aabb;
    
    bool _is_homogenized;
    bool _has_normals;
    bool _has_material;
    
    TriangleMesh<F>& _clip(size_t component, F limit, int sign);
    void _addIntersectionVertices(size_t i, size_t j, size_t k, F origin[], F other_1[], F other_2[], size_t component_1, size_t component_2, size_t component_3, F limit);

public:
    TriangleMesh<F>();
    TriangleMesh<F>(size_t n_vertices, F vertex_array[],
                    size_t n_faces, size_t face_array[]);
    TriangleMesh<F>(const TriangleMesh<F>& mesh_1,
                    const TriangleMesh<F>& mesh_2);
    TriangleMesh<F>(const TriangleMesh<F>& other);
    ~TriangleMesh<F>();
    
    static TriangleMesh<F> triangle(const Triangle<F>& triangle_obj);
    static TriangleMesh<F> box(const Box<F>& box_obj);
    static TriangleMesh<F> sheet(const Point<F>& origin, const Vector<F>& width_vector, const Vector<F>& height_vector);

    size_t getNumberOfVertices() const;
    size_t getNumberOfFaces() const;

    TriangleMesh<F>& addVertex(F x, F y, F z);
    TriangleMesh<F>& addVertex(const Point<F>& vertex);
    TriangleMesh<F>& addFace(size_t i, size_t j, size_t k);

    TriangleMesh<F>& removeVertex(size_t idx);
    TriangleMesh<F>& removeFace(size_t idx);

    TriangleMesh<F>& setMaterial(const BlinnPhongMaterial& material);
    TriangleMesh<F> withMaterial(const BlinnPhongMaterial& material);

    TriangleMesh<F>& splitFaces(size_t n_times);
    
    TriangleMesh<F>& computeNormals();

    TriangleMesh<F>& applyTransformation(const LinearTransformation& transformation);
    TriangleMesh<F>& applyTransformation(const AffineTransformation& transformation);
    TriangleMesh<F>& applyTransformation(const ProjectiveTransformation& transformation);
    TriangleMesh<F>& applyWindowingTransformation(const AffineTransformation& transformation);

    TriangleMesh<F>& shadeVerticesDirect(const Scene& scene);

    TriangleMesh<F>& homogenizeVertices();

    Triangle2 getProjectedFace(const Scene& scene, size_t face_idx) const;
    void getVertexAttributes(size_t face_idx, Point<F> vertices[3], Vector<F> normals[3], Material*& material) const;

    TriangleMesh<F>& computeAABB();
    
    bool evaluateRayAABBIntersection(const Ray<F>& ray, F upper_distance_limit) const;
    F evaluateRayFaceIntersection(const Ray<F>& ray, size_t face_idx, F& alpha, F& beta, F& gamma) const;
    bool sampleRayTriangle(const Scene& scene, size_t x, size_t y, const Ray<F>& ray, size_t face_idx, Radiance& pixel_radiance, F& closest_distance) const;
    
    TriangleMesh<F>& clipNearPlaneAt(F z_near);
    TriangleMesh<F>& clipLeftPlane();
    TriangleMesh<F>& clipRightPlane();
    TriangleMesh<F>& clipTopPlane();
    TriangleMesh<F>& clipBottomPlane();
    TriangleMesh<F>& clipNearPlane();
    TriangleMesh<F>& clipFarPlane();

    TriangleMesh<F>& performClipping();

    TriangleMesh<F>& removeBackwardFacingFaces();

    TriangleMesh<F>& drawEdges(Image& image, float luminance);
    TriangleMesh<F>& drawFaces(Image& image);
    TriangleMesh<F>& drawFaces(Image& image, Color color);

    Point<F> getVertex(size_t idx) const;
    Triangle<F> getFace(size_t idx) const;

    const AxisAlignedBox<F>& getAABB() const;

    std::string getVerticesString() const;
    std::string get4SpaceVerticesString() const;
    std::string getFacesString() const;

    bool isHomogenized() const;
};

template <typename F>
TriangleMesh<F>::TriangleMesh()
    : _aabb(Point<F>::min(), Point<F>::max()),
      _is_homogenized(true),
      _has_normals(false),
      _has_material(false) {}

template <typename F>
TriangleMesh<F>::TriangleMesh(size_t n_vertices, F vertex_array[],
                              size_t n_faces, size_t face_array[])
    : _vertices(4, n_vertices, arma::fill::ones),
      _faces(face_array, 3, n_faces),
      _aabb(Point<F>::min(), Point<F>::max()),
      _is_homogenized(true),
      _has_normals(false),
      _has_material(false)
{
    _vertices.rows(0, 2) = arma::Mat<F>(vertex_array, 3, n_vertices);
}

template <typename F>
TriangleMesh<F>::TriangleMesh(const TriangleMesh<F>& mesh_1,
                              const TriangleMesh<F>& mesh_2)
    : _vertices(arma::join_cols(mesh_1._vertices, mesh_2._vertices)),
      _faces(arma::join_cols(mesh_1._faces, mesh_2._faces + mesh_2.getNumberOfVertices())),
      _aabb(Point<F>::min(), Point<F>::max()),
      _is_homogenized(mesh_1._is_homogenized && mesh_2._is_homogenized),
      _has_normals(false),
      _has_material(false) {}

template <typename F>
TriangleMesh<F>::TriangleMesh(const TriangleMesh<F>& other)
    : _vertices(other._vertices),
      _faces(other._faces),
      _normals(other._normals),
      _colors(other._colors),
      _aabb(other._aabb),
      _is_homogenized(other._is_homogenized),
      _has_normals(other._has_normals),
      _has_material(other._has_material)
{
    if (other._material)
    {
        if (BlinnPhongMaterial* material_ptr = dynamic_cast<BlinnPhongMaterial*>(other._material))
            _material = new BlinnPhongMaterial(*material_ptr);
    }
}

template <typename F>
TriangleMesh<F>::~TriangleMesh()
{
    if (_material) delete _material;
}

template <typename F>
TriangleMesh<F> TriangleMesh<F>::triangle(const Triangle<F>& triangle_obj)
{
    TriangleMesh<F> triangle_mesh;
    triangle_mesh._aabb = AxisAlignedBox<F>(Point<F>::min(), Point<F>::max());
    triangle_mesh._is_homogenized = true;
    triangle_mesh._has_normals = false;
    triangle_mesh._has_material = false;

    const Point<F>& A = triangle_obj.getPointA();
    const Point<F>& B = triangle_obj.getPointB();
    const Point<F>& C = triangle_obj.getPointC();

    triangle_mesh._vertices = {{A.x, B.x, C.x},
                               {A.y, B.y, C.y},
                               {A.z, B.z, C.z},
                               {1, 1, 1}};

    triangle_mesh._faces = {{0}, {1}, {2}};

    return triangle_mesh;
}

template <typename F>
TriangleMesh<F> TriangleMesh<F>::box(const Box<F>& box_obj)
{
    TriangleMesh<F> box_mesh;
    box_mesh._aabb = AxisAlignedBox<F>(Point<F>::min(), Point<F>::max());
    box_mesh._is_homogenized = true;
    box_mesh._has_normals = false;
    box_mesh._has_material = false;

    const std::vector< Point<F> >& corners = box_obj.getCorners();

    F x0 = corners[0].x, y0 = corners[0].y, z0 = corners[0].z;
    F x1 = corners[1].x, y1 = corners[1].y, z1 = corners[1].z;
    F x2 = corners[2].x, y2 = corners[2].y, z2 = corners[2].z;
    F x3 = corners[3].x, y3 = corners[3].y, z3 = corners[3].z;
    F x4 = corners[4].x, y4 = corners[4].y, z4 = corners[4].z;
    F x5 = corners[5].x, y5 = corners[5].y, z5 = corners[5].z;
    F x6 = corners[6].x, y6 = corners[6].y, z6 = corners[6].z;
    F x7 = corners[7].x, y7 = corners[7].y, z7 = corners[7].z;

    box_mesh._vertices = {{x0, x0, x0, x1, x1, x1, x2, x2, x2, x3, x3, x3, x4, x4, x4, x5, x5, x5, x6, x6, x6, x7, x7, x7},
                          {y0, y0, y0, y1, y1, y1, y2, y2, y2, y3, y3, y3, y4, y4, y4, y5, y5, y5, y6, y6, y6, y7, y7, y7},
                          {z0, z0, z0, z1, z1, z1, z2, z2, z2, z3, z3, z3, z4, z4, z4, z5, z5, z5, z6, z6, z6, z7, z7, z7},
                          { 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1}};

    box_mesh._faces = {{0, 3, 12, 21,  1, 13, 19, 22,  2, 11,  5, 17},
                       {9, 9, 15, 15,  4,  4,  7,  7, 14, 14,  8,  8},
                       {3, 6, 21, 18, 13, 16, 22, 10, 11, 23, 17, 20}};

    return box_mesh;
}

template <typename F>
TriangleMesh<F> TriangleMesh<F>::sheet(const Point<F>& origin, const Vector<F>& width_vector, const Vector<F>& height_vector)
{
    TriangleMesh<F> sheet_mesh;
    sheet_mesh._aabb = AxisAlignedBox<F>(Point<F>::min(), Point<F>::max());
    sheet_mesh._is_homogenized = true;
    sheet_mesh._has_normals = false;
    sheet_mesh._has_material = false;

    const Point<F>& corner_1 = origin;
    const Point<F>& corner_2 = corner_1 + width_vector;
    const Point<F>& corner_3 = corner_2 + height_vector;
    const Point<F>& corner_4 = corner_1 + height_vector;

    sheet_mesh._vertices = {{corner_1.x, corner_2.x, corner_3.x, corner_4.x},
                            {corner_1.y, corner_2.y, corner_3.y, corner_4.y},
                            {corner_1.z, corner_2.z, corner_3.z, corner_4.z},
                            {         1,          1,          1,          1}};

    sheet_mesh._faces = {{1, 0},
                         {2, 2},
                         {0, 3}};

    return sheet_mesh;
}

template <typename F>
size_t TriangleMesh<F>::getNumberOfVertices() const
{
    return _vertices.n_cols;
}

template <typename F>
size_t TriangleMesh<F>::getNumberOfFaces() const
{
    return _faces.n_cols;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::addVertex(F x, F y, F z)
{
    _vertices.insert_cols(_vertices.n_cols, arma::Col<F>({x, y, z, 1}));
    _has_normals = false;
    return *this;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::addVertex(const Point<F>& vertex)
{
    return addVertex(vertex.x, vertex.y, vertex.z);
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::addFace(size_t i, size_t j, size_t k)
{
    _faces.insert_cols(_faces.n_cols, arma::Col<size_t>({i, j, k}));
    _has_normals = false;
    return *this;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::removeVertex(size_t idx)
{
    arma::uvec occurences = arma::unique(arma::find(_faces == idx)/3);
    arma::uvec::const_iterator iter = occurences.end();
    while (iter != occurences.begin())
    {
        --iter;
        _faces.shed_col(*iter);
    }

    _vertices.shed_col(idx);

    occurences = arma::find(_faces > idx);
    for (iter = occurences.begin(); iter != occurences.end(); ++iter)
    {
        _faces(*iter) += 1;
    }

    _has_normals = false;

    return *this;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::removeFace(size_t idx)
{
    _faces.shed_col(idx);
    _has_normals = false;
    return *this;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::setMaterial(const BlinnPhongMaterial& material)
{
    if (_material) delete _material;
     _material = new BlinnPhongMaterial(material);
    _has_material = true;
    return *this;
}

template <typename F>
TriangleMesh<F> TriangleMesh<F>::withMaterial(const BlinnPhongMaterial& material)
{
    return TriangleMesh<F>(*this).setMaterial(material);
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::splitFaces(size_t n_times)
{
    size_t n_faces;
    size_t idx, i, j, k;
    Point<F> A, B, C;
    F AB_length, AC_length, BC_length;
    bool AB_exceeds_AC, AB_exceeds_BC, AC_exceeds_BC;
    size_t last_vertex_idx = getNumberOfVertices() - 1;

    for (size_t n = 0; n < n_times; n++)
    {
        n_faces = getNumberOfFaces();

        for (idx = 0; idx < n_faces; idx++)
        {
            i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);
            A.x = _vertices(0, i); A.y = _vertices(1, i); A.z = _vertices(2, i);
            B.x = _vertices(0, j); B.y = _vertices(1, j); B.z = _vertices(2, j);
            C.x = _vertices(0, k); C.y = _vertices(1, k); C.z = _vertices(2, k);

            const Vector<F>& AB = B - A;
            const Vector<F>& AC = C - A;
            const Vector<F>& BC = C - B;

            AB_length = AB.getLength();
            AC_length = AC.getLength();
            BC_length = BC.getLength();

            AB_exceeds_AC = AB_length > AC_length;
            AB_exceeds_BC = AB_length > BC_length;
            AC_exceeds_BC = AC_length > BC_length;

            if (AB_exceeds_AC && AB_exceeds_BC)
            {
                addVertex(A + AB*0.5);
                last_vertex_idx++;
                _faces(1, idx) = last_vertex_idx;
                addFace(last_vertex_idx, j, k);
            }
            else if (AC_exceeds_BC)
            {
                addVertex(A + AC*0.5);
                last_vertex_idx++;
                _faces(2, idx) = last_vertex_idx;
                addFace(last_vertex_idx, j, k);
            }
            else
            {
                addVertex(B + BC*0.5);
                last_vertex_idx++;
                _faces(2, idx) = last_vertex_idx;
                addFace(last_vertex_idx, k, i);
            }
        }
    }

    return *this;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::computeNormals()
{
    _normals = arma::Mat<F>(3, getNumberOfVertices(), arma::fill::zeros);
    
    size_t n_faces = getNumberOfFaces();
    size_t i, j, k;

    for (size_t idx = 0; idx < n_faces; idx++)
    {
        i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);

        const Vector<F>& normal = Triangle<F>::areaVector(Point<F>(_vertices(0, i), _vertices(1, i), _vertices(2, i)),
                                                          Point<F>(_vertices(0, j), _vertices(1, j), _vertices(2, j)),
                                                          Point<F>(_vertices(0, k), _vertices(1, k), _vertices(2, k)));

        _normals(0, i) += normal.x;
        _normals(1, i) += normal.y;
        _normals(2, i) += normal.z;

        _normals(0, j) += normal.x;
        _normals(1, j) += normal.y;
        _normals(2, j) += normal.z;

        _normals(0, k) += normal.x;
        _normals(1, k) += normal.y;
        _normals(2, k) += normal.z;
    }

    _normals = arma::normalise(_normals);
    
    _has_normals = true;

    return *this;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::applyTransformation(const LinearTransformation& transformation)
{
    _vertices.rows(0, 2) = transformation.getMatrix().submat(0, 0, 2, 2)*_vertices.rows(0, 2);

    if (_has_normals)
        _normals = transformation.getNormalTransformMatrix()*_normals;

    return *this;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::applyTransformation(const AffineTransformation& transformation)
{
    _vertices = transformation.getMatrix()*_vertices;

    if (_has_normals)
        _normals = transformation.getNormalTransformMatrix()*_normals;

    return *this;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::applyTransformation(const ProjectiveTransformation& transformation)
{
    _vertices = transformation.getMatrix()*_vertices;
    _is_homogenized = false;
    _has_normals = false;
    return *this;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::applyWindowingTransformation(const AffineTransformation& transformation)
{
    assert(_is_homogenized);
    _vertices.rows(0, 1) = transformation.getMatrix().submat(0, 0, 1, 3)*_vertices;
    _has_normals = false;
    return *this;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::shadeVerticesDirect(const Scene& scene)
{
    assert(_is_homogenized);
    assert(_has_normals);
    assert(_has_material);

    Point<F> vertex_point;
    Vector<F> normal_vector;
    size_t n_vertices = getNumberOfVertices();

    _colors.clear();

    for (size_t idx = 0; idx < n_vertices; idx++)
    {
        vertex_point.x = _vertices(0, idx);
        vertex_point.y = _vertices(1, idx);
        vertex_point.z = _vertices(2, idx);

        normal_vector.x = _normals(0, idx);
        normal_vector.y = _normals(1, idx);
        normal_vector.z = _normals(2, idx);

        const Vector<F>& scatter_direction = scene._camera_position - vertex_point;

        _colors.push_back(scene._getRadiance(vertex_point,
                                             normal_vector,
                                             scatter_direction,
                                             _material,
                                             true).clamp());
    }

    return *this;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::homogenizeVertices()
{
    const arma::Row<F>& norm_vals = 1/_vertices.row(3);
    _vertices.each_row(arma::uvec({0, 1, 2})) %= norm_vals;
    _vertices.row(3).ones();

    _is_homogenized = true;
    _has_normals = false;

    return *this;
}

template <typename F>
Geometry2D::Triangle<F> TriangleMesh<F>::getProjectedFace(const Scene& scene, size_t face_idx) const
{
    return Triangle2(scene._getPerspectiveProjected(_vertices.col(_faces(0, face_idx)).head(3)),
                     scene._getPerspectiveProjected(_vertices.col(_faces(1, face_idx)).head(3)),
                     scene._getPerspectiveProjected(_vertices.col(_faces(2, face_idx)).head(3)));
}

template <typename F>
void TriangleMesh<F>::getVertexAttributes(size_t face_idx, Point<F> vertices[3], Vector<F> normals[3], Material*& material) const
{
    assert(_has_normals);
    assert(_has_material);

    size_t i = _faces(0, face_idx), j = _faces(1, face_idx), k = _faces(2, face_idx);

    vertices[0].moveTo(_vertices(0, i), _vertices(1, i), _vertices(2, i));
    vertices[1].moveTo(_vertices(0, j), _vertices(1, j), _vertices(2, j));
    vertices[2].moveTo(_vertices(0, k), _vertices(1, k), _vertices(2, k));

    normals[0].setComponents(_normals(0, i), _normals(1, i), _normals(2, i));
    normals[1].setComponents(_normals(0, j), _normals(1, j), _normals(2, j));
    normals[2].setComponents(_normals(0, k), _normals(1, k), _normals(2, k));
    
    material = _material;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::computeAABB()
{
    _aabb = AxisAlignedBox<F>(Point<F>(_vertices.row(0).min(), _vertices.row(1).min(), _vertices.row(2).min()),
                              Point<F>(_vertices.row(0).max(), _vertices.row(1).max(), _vertices.row(2).max()));
    return *this;
}

template <typename F>
bool TriangleMesh<F>::evaluateRayAABBIntersection(const Ray<F>& ray, F upper_distance_limit) const
{
    return _aabb.evaluateRayIntersection(ray, upper_distance_limit);
}

template <typename F>
F TriangleMesh<F>::evaluateRayFaceIntersection(const Ray<F>& ray, size_t face_idx, F& alpha, F& beta, F& gamma) const
{
    size_t i = _faces(0, face_idx), j = _faces(1, face_idx), k = _faces(2, face_idx);
    Vector<F> AB(_vertices(0, j) - _vertices(0, i), _vertices(1, j) - _vertices(1, i), _vertices(2, j) - _vertices(2, i));
    Vector<F> AC(_vertices(0, k) - _vertices(0, i), _vertices(1, k) - _vertices(1, i), _vertices(2, k) - _vertices(2, i));

    const Vector<F>& q = ray.direction.cross(AC);
    F impact_factor = AB.dot(q); // Can be zero, resulting in infinite barycentric coordinates

    const Vector<F>& displacement = ray.origin - Point<F>(_vertices(0, i), _vertices(1, i), _vertices(2, i));
    const Vector<F>& r = displacement.cross(AB);

    beta = displacement.dot(q)/impact_factor;
    gamma = ray.direction.dot(r)/impact_factor;
    alpha = 1 - beta - gamma;

    F distance = AC.dot(r)/impact_factor;

    const F eps_impact_factor = 1e-7f;
    const F eps_coordinates = -1e-10f;

    if (impact_factor <= eps_impact_factor ||
        alpha < eps_coordinates || beta < eps_coordinates || gamma < eps_coordinates ||
        distance <= 0)
    {
        distance = std::numeric_limits<F>::infinity();
    }

    return distance;
}

template <typename F>
bool TriangleMesh<F>::sampleRayTriangle(const Scene& scene, size_t x, size_t y, const Ray<F>& ray, size_t face_idx, Radiance& pixel_radiance, F& closest_distance) const
{
    assert(_has_normals);
    assert(_has_material);

    F alpha, beta, gamma;
    F distance = evaluateRayFaceIntersection(ray, face_idx, alpha, beta, gamma);

    if (distance >= closest_distance)
        return false;

    closest_distance = distance;

    const Point<F>& intersection_point = ray(distance);

    size_t i = _faces(0, face_idx), j = _faces(1, face_idx), k = _faces(2, face_idx);
    Vector<F> interpolated_normal(_normals(0, i)*alpha + _normals(0, j)*beta + _normals(0, k)*gamma,
                                  _normals(1, i)*alpha + _normals(1, j)*beta + _normals(1, k)*gamma,
                                  _normals(2, i)*alpha + _normals(2, j)*beta + _normals(2, k)*gamma);
    interpolated_normal.normalize();

    pixel_radiance = scene._getRadiance(intersection_point,
                                        interpolated_normal,
                                        -ray.direction,
                                        _material,
                                        false);

    return true;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::clipNearPlaneAt(F z_near)
{
    assert(_is_homogenized);
    return _clip(2, z_near, 1); // Clip away points with z > z_near
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::clipLeftPlane()
{
    assert(_is_homogenized);
    return _clip(0, -1, -1); // Clip away points with x < -1
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::clipRightPlane()
{
    assert(_is_homogenized);
    return _clip(0, 1, 1); // Clip away points with x > 1
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::clipBottomPlane()
{
    assert(_is_homogenized);
    return _clip(1, -1, -1); // Clip away points with y < -1
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::clipTopPlane()
{
    assert(_is_homogenized);
    return _clip(1, 1, 1); // Clip away points with y > 1
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::clipNearPlane()
{
    assert(_is_homogenized);
    return _clip(2, 0, 1); // Clip away points with z > 0
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::clipFarPlane()
{
    assert(_is_homogenized);
    return _clip(2, -1, -1); // Clip away points with z < -1
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::performClipping()
{
    const Point<F>& lower_corner = _aabb.getLowerCorner();
    const Point<F>& upper_corner = _aabb.getUpperCorner();

    if (lower_corner.x < -1) clipLeftPlane();
    if (upper_corner.x >  1) clipRightPlane();
    if (lower_corner.y < -1) clipBottomPlane();
    if (upper_corner.y >  1) clipTopPlane();
    if (lower_corner.z < -1) clipFarPlane();
    if (upper_corner.z >  0) clipNearPlane();
    
    return *this;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::removeBackwardFacingFaces()
{
    size_t n_faces = getNumberOfFaces();
    size_t i, j, k;
    F AB_x, AB_y, AC_x, AC_y;

    for (int idx = static_cast<int>(n_faces) - 1; idx >= 0; idx--)
    {
        i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);

        AB_x = _vertices(0, j) - _vertices(0, i);
        AB_y = _vertices(1, j) - _vertices(1, i);
        AC_x = _vertices(0, k) - _vertices(0, i);
        AC_y = _vertices(1, k) - _vertices(1, i);

        if (AB_x*AC_y - AB_y*AC_x < 0)
        {
            removeFace(idx);
        }
    }

    return *this;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::_clip(size_t component, F limit, int sign)
{
    assert(component < 3);
    assert(sign == 1 || sign == -1);

    size_t n_vertices = getNumberOfVertices();
    assert(_colors.empty() || _colors.size() == n_vertices);

    size_t component_1 = (component + 1) % 3;
    size_t component_2 = (component + 2) % 3;

    F signed_limit = sign*limit;

    size_t i, j, k;
    F A[3], B[3], C[3];

    bool A_outside, B_outside, C_outside;

    size_t n_outside;
    size_t inside_idx_1, inside_idx_2;

    size_t n_faces = getNumberOfFaces();
    size_t last_vertex = n_vertices - 1;
    
    std::vector<size_t> deleted_faces;

    for (size_t idx = 0; idx < n_faces; idx++)
    {

        i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);
        
        A[0] = _vertices(0, i); A[1] = _vertices(1, i); A[2] = _vertices(2, i);
        B[0] = _vertices(0, j); B[1] = _vertices(1, j); B[2] = _vertices(2, j);
        C[0] = _vertices(0, k); C[1] = _vertices(1, k); C[2] = _vertices(2, k);

        A_outside = sign*A[component] > signed_limit;
        B_outside = sign*B[component] > signed_limit;
        C_outside = sign*C[component] > signed_limit;

        n_outside = 0;
        if (A_outside) n_outside++;
        if (B_outside) n_outside++;
        if (C_outside) n_outside++;

        if (n_outside > 0)
        {
            if (n_outside == 1)
            {
                if (A_outside)
                {
                    inside_idx_1 = j;
                    inside_idx_2 = k;
                    _addIntersectionVertices(i, j, k,
                                             A, B, C,
                                             component, component_1, component_2,
                                             limit);
                }
                else if (B_outside)
                {
                    inside_idx_1 = k;
                    inside_idx_2 = i;
                    _addIntersectionVertices(i, j, k,
                                             B, C, A,
                                             component, component_1, component_2,
                                             limit);
                }
                else
                {
                    inside_idx_1 = i;
                    inside_idx_2 = j;
                    _addIntersectionVertices(i, j, k,
                                             C, A, B,
                                             component, component_1, component_2,
                                             limit);
                }
            
                _faces(0, idx) = inside_idx_1;
                _faces(1, idx) = inside_idx_2;
                _faces(2, idx) = last_vertex + 1;

                _faces.insert_cols(_faces.n_cols, arma::Col<size_t>({last_vertex + 1, inside_idx_2, last_vertex + 2}));
                
                last_vertex += 2;
            }
            else if (n_outside == 2)
            {
                if (!A_outside) 
                {
                    inside_idx_1 = i;
                    _addIntersectionVertices(i, j, k,
                                             A, B, C,
                                             component, component_1, component_2,
                                             limit);
                }
                else if (!B_outside)
                {
                    inside_idx_1 = j;
                    _addIntersectionVertices(i, j, k,
                                             B, C, A,
                                             component, component_1, component_2,
                                             limit);
                }
                else                
                {
                    inside_idx_1 = k;
                    _addIntersectionVertices(i, j, k,
                                             C, A, B,
                                             component, component_1, component_2,
                                             limit);
                }
            
                _faces(0, idx) = inside_idx_1;
                _faces(1, idx) = last_vertex + 1;
                _faces(2, idx) = last_vertex + 2;
                
                last_vertex += 2;
            }
            else
            {
                deleted_faces.push_back(idx);
            }
        }
    }

    while (!deleted_faces.empty())
    {
        _faces.shed_col(deleted_faces.back());
        deleted_faces.pop_back();
    }

    return *this;
}

template <typename F>
void TriangleMesh<F>::_addIntersectionVertices(size_t i, size_t j, size_t k,
                                               F origin[], F other_1[], F other_2[],
                                               size_t component_0, size_t component_1, size_t component_2,
                                               F limit)
{
    // Find vertices

    arma::Col<F> vertex_vector1(4);
    arma::Col<F> vertex_vector2(4);
    vertex_vector1(3) = 1;
    vertex_vector2(3) = 1;

    F orig_plane_dist = limit - origin[component_0];

    F scaling = orig_plane_dist/(other_1[component_0] - origin[component_0]);
    vertex_vector1(component_0) = limit;
    vertex_vector1(component_1) = origin[component_1] + scaling*(other_1[component_1] - origin[component_1]);
    vertex_vector1(component_2) = origin[component_2] + scaling*(other_1[component_2] - origin[component_2]);

    scaling = orig_plane_dist/(other_2[component_0] - origin[component_0]);
    vertex_vector2(component_0) = limit;
    vertex_vector2(component_1) = origin[component_1] + scaling*(other_2[component_1] - origin[component_1]);
    vertex_vector2(component_2) = origin[component_2] + scaling*(other_2[component_2] - origin[component_2]);

    F alpha1, beta1, gamma1;
    F alpha2, beta2, gamma2;

    // Find barycentric coordinates of new vertices in old triangle
    if (_has_normals || !_colors.empty())
    {
        Triangle<F> face(Point<F>(_vertices(0, i), _vertices(1, i), _vertices(2, i)),
                         Point<F>(_vertices(0, j), _vertices(1, j), _vertices(2, j)),
                         Point<F>(_vertices(0, k), _vertices(1, k), _vertices(2, k)));

        face.computeNormals();
        
        Point<F> vertex1(vertex_vector1(0), vertex_vector1(1), vertex_vector1(2));
        Point<F> vertex2(vertex_vector2(0), vertex_vector2(1), vertex_vector2(2));

        face.getBarycentricCoordinatesInside(vertex1, alpha1, beta1, gamma1);
        face.getBarycentricCoordinatesInside(vertex2, alpha2, beta2, gamma2);
    }

    // Add normals
    if (_has_normals)
    {
        _normals.insert_cols(_normals.n_cols, arma::normalise(alpha1*_normals.col(i) + beta1*_normals.col(j) + gamma1*_normals.col(k)));
        _normals.insert_cols(_normals.n_cols, arma::normalise(alpha2*_normals.col(i) + beta2*_normals.col(j) + gamma2*_normals.col(k)));
    }
    
    // Add colors
    if (!_colors.empty())
    {
        const Color& color_A = _colors[i];
        const Color& color_B = _colors[j];
        const Color& color_C = _colors[k];

        const Color& color1 = alpha1*color_A + beta1*color_B + gamma1*color_C;
        const Color& color2 = alpha2*color_A + beta2*color_B + gamma2*color_C;

        _colors.push_back(color1);
        _colors.push_back(color2);
    }
    
    _vertices.insert_cols(_vertices.n_cols, vertex_vector1);
    _vertices.insert_cols(_vertices.n_cols, vertex_vector2);
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::drawEdges(Image& image, float luminance)
{
    assert(_is_homogenized);

    size_t n_faces = getNumberOfFaces();
    size_t i, j, k;

    for (size_t idx = 0; idx < n_faces; idx++)
    {
        i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);

        image.drawLine(_vertices(0, i), _vertices(1, i),
                          _vertices(0, j), _vertices(1, j), luminance);

        image.drawLine(_vertices(0, j), _vertices(1, j),
                          _vertices(0, k), _vertices(1, k), luminance);
        
        image.drawLine(_vertices(0, k), _vertices(1, k),
                          _vertices(0, i), _vertices(1, i), luminance);
    }

    return *this;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::drawFaces(Image& image)
{
    assert(_is_homogenized);
    assert(_colors.size() == getNumberOfVertices());

    size_t n_faces = getNumberOfFaces();
    size_t i, j, k;

    for (size_t idx = 0; idx < n_faces; idx++)
    {
        i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);

        image.drawTriangle(Vertex<F>(_vertices(0, i), _vertices(1, i), _vertices(2, i), _colors[i]),
                              Vertex<F>(_vertices(0, j), _vertices(1, j), _vertices(2, j), _colors[j]),
                              Vertex<F>(_vertices(0, k), _vertices(1, k), _vertices(2, k), _colors[k]));
    }
    
    return *this;
}

template <typename F>
TriangleMesh<F>& TriangleMesh<F>::drawFaces(Image& image, Color color)
{
    assert(_is_homogenized);

    size_t n_faces = getNumberOfFaces();
    size_t i, j, k;

    for (size_t idx = 0; idx < n_faces; idx++)
    {
        i = _faces(0, idx); j = _faces(1, idx); k = _faces(2, idx);

        image.drawTriangle(Vertex<F>(_vertices(0, i), _vertices(1, i), _vertices(2, i), color),
                           Vertex<F>(_vertices(0, j), _vertices(1, j), _vertices(2, j), color),
                           Vertex<F>(_vertices(0, k), _vertices(1, k), _vertices(2, k), color));
    }
    
    return *this;
}

template <typename F>
Point<F> TriangleMesh<F>::getVertex(size_t idx) const
{
    return Point<F>(_vertices(0, idx), _vertices(1, idx), _vertices(2, idx));
}

template <typename F>
Triangle<F> TriangleMesh<F>::getFace(size_t idx) const
{
    return Triangle<F>(getVertex(_faces(0, idx)), getVertex(_faces(1, idx)), getVertex(_faces(2, idx)));
}

template <typename F>
const AxisAlignedBox<F>& TriangleMesh<F>::getAABB() const
{
    return _aabb;
}

template <typename F>
std::string TriangleMesh<F>::getVerticesString() const
{
    std::ostringstream string_stream;
    string_stream << _vertices.rows(0, 2);
    return string_stream.str();
}

template <typename F>
std::string TriangleMesh<F>::get4SpaceVerticesString() const
{
    std::ostringstream string_stream;
    string_stream << _vertices;
    return string_stream.str();
}

template <typename F>
std::string TriangleMesh<F>::getFacesString() const
{
    std::ostringstream string_stream;
    string_stream << _faces;
    return string_stream.str();
}

template <typename F>
bool TriangleMesh<F>::isHomogenized() const
{
    return _is_homogenized;
}

} // Geometry3D
