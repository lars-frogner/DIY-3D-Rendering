#include <glut.h>
#include <vector>
#include <ctime>
#include "../Geometry3D/Point.hpp"
#include "../Geometry3D/Vector.hpp"
#include "../Geometry3D/Triangle.hpp"
#include "../Geometry3D/Ray.hpp"
#include "../Geometry3D/Box.hpp"
#include "../Geometry3D/TriangleMesh.hpp"
#include "../Geometry3D/Camera.hpp"
#include "../Geometry2D/Vertex.hpp"
#include "../Transformations3D/Transformation.hpp"
#include "../Transformations3D/LinearTransformation.hpp"
#include "../Transformations3D/AffineTransformation.hpp"
#include "../Transformations3D/ProjectiveTransformation.hpp"
#include "SceneGraph.hpp"
#include "Image.hpp"
#include "Color.hpp"
#include "BlinnPhongMaterial.hpp"
#include "Light.hpp"
#include "LightContainer.hpp"
#include "RectangularAreaLight.hpp"
#include "HemisphereAreaLight.hpp"
#include "OmnidirectionalLight.hpp"
#include "DirectionalLight.hpp"
#include "Scene.hpp"

const Rendering3D::Image<float>* PIXEL_ARRAY_PTR;

void renderImage()
{   
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    glDrawPixels(static_cast<GLsizei>(PIXEL_ARRAY_PTR->getWidth()),
                 static_cast<GLsizei>(PIXEL_ARRAY_PTR->getHeight()),
                 GL_RGB, GL_FLOAT,
                 PIXEL_ARRAY_PTR->getRawPixelArray());

    glutSwapBuffers();
}

template <typename F>
void showImage(int argc, char *argv[], const Rendering3D::Image<F>& image)
{
    glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(static_cast<int>(image.getWidth()),
                       static_cast<int>(image.getHeight()));
	glutInitWindowPosition(100, 100);
    
    glutCreateWindow("Test");
    
    PIXEL_ARRAY_PTR = &image;
    glutDisplayFunc(renderImage);

    glutMainLoop();
}

int main(int argc, char *argv[])
{
    typedef Geometry3D::Point<float> Point;
    typedef Geometry3D::Vector<float> Vector;
    typedef Geometry3D::Triangle<float> Triangle;
    typedef Geometry3D::Ray<float> Ray;
    typedef Geometry3D::Box<float> Box;
    typedef Geometry3D::TriangleMesh<float> TriangleMesh;
    typedef Geometry3D::Camera<float> Camera;
    typedef Geometry2D::Vertex<float> Vertex;
    typedef Transformations3D::LinearTransformation<float> LinearTransformation;
    typedef Transformations3D::AffineTransformation<float> AffineTransformation;
    typedef Transformations3D::ProjectiveTransformation<float> ProjectiveTransformation;
    typedef Rendering3D::SceneGraph<float> SceneGraph;
    typedef Rendering3D::Image<float> Image;
    typedef Rendering3D::Color Color;
    typedef Rendering3D::Power Power;
    typedef Rendering3D::Reflectance Reflectance;
    typedef Rendering3D::Biradiance Biradiance;
    typedef Rendering3D::BlinnPhongMaterial<float> BlinnPhongMaterial;
    typedef Rendering3D::Light<float> Light;
    typedef Rendering3D::RectangularAreaLight<float> RectangularAreaLight;
    typedef Rendering3D::HemisphereAreaLight<float> HemisphereAreaLight;
    typedef Rendering3D::OmnidirectionalLight<float> OmnidirectionalLight;
    typedef Rendering3D::DirectionalLight<float> DirectionalLight;
    typedef Rendering3D::LightContainer<float> LightContainer;
    typedef Rendering3D::Scene<float> Scene;

    BlinnPhongMaterial ground_material(Reflectance::grey(0.7f),
                                       Reflectance::grey(0.0f),
                                       0.0f);

    BlinnPhongMaterial blue_material(Reflectance(0.2f, 0.2f, 0.7f),
                                     Reflectance(0.0f, 0.0f, 0.3f),
                                     0.3f);

    BlinnPhongMaterial red_material(Reflectance(0.7f, 0.2f, 0.2f),
                                    Reflectance(0.3f, 0.0f, 0.0f),
                                    0.3f);

    TriangleMesh box_mesh = TriangleMesh::box(Box(Point(-0.5f, -0.5f, -0.5f),
                                                  1, 1, 1));

    /*TriangleMesh ground_mesh = TriangleMesh::sheet(Point(-50.0f, -1.4f, -0.1f),
                                                   Vector(100.0f, 0.0f, 0.0f),
                                                   Vector(0.0f, 0.0f, -100.0f));*/

    SceneGraph scene_graph(AffineTransformation::translation(-0.1f, 0, -5.0f));

    scene_graph.addTransformation(AffineTransformation::translation(-0.5f, 0, 0))
               ->addTransformation(LinearTransformation::rotationFromYToZ(0.5f))
               ->addTransformation(LinearTransformation::rotationFromXToY(-0.6f))
               ->addObject(box_mesh.withMaterial(blue_material));

    scene_graph.addTransformation(AffineTransformation::translation(1.0f, -0.3f, -1.0f))
               ->addTransformation(LinearTransformation::rotationFromYToZ(-0.4f))
               ->addTransformation(LinearTransformation::scaling(0.5f, 1, 2.5f))
               ->addObject(box_mesh.withMaterial(red_material));


    /*Triangle triangle(Point(0.0f, 1.0f, 0.0f),
                      Point(-1.9f, -1.0f, 0.0f),
                      Point(1.6f, -0.5f, 0.0f));

    TriangleMesh mesh = TriangleMesh::triangle(triangle);

    SceneGraph scene_graph(AffineTransformation::translation(-0.3f, 0.0f, -5.0f));
    scene_graph.addObject(mesh.withMaterial(blue_material))
               ->addTransformation(AffineTransformation::translation(0.5f, -1.0f, 0.0f))
               ->addTransformation(LinearTransformation::rotationFromYToZ(M_PI_4))
               ->addObject(mesh.withMaterial(red_material));*/

    

    /*TriangleMesh ground_mesh = TriangleMesh::sheet(Point(-0.05f, 0.0f, -2.0f),
                                                   Vector(0.1f, 0.0f, 0.0f),
                                                   Vector(0.0f, 0.0f, 0.1f));

    ground_mesh.setMaterial(blue_material);

    SceneGraph scene_graph(ground_mesh);*/



    Camera camera(Ray(Point(0.0f, 0.0f, 0.0f), Vector(0.0f, 0.0f, -1.0f)),
                  Vector(0.0f, 1.0f, 0),
                  0.1f, 100,
                  45);

    RectangularAreaLight light_sheet(Point(2.5f, 1.0f, -5.0f),
                                     Vector(-1.0f, -0.1f, 0.0f),
                                     Vector(0.001f, 0, 0),
                                     0.001f,
                                     Power::grey(250.0f),
                                     1);

    HemisphereAreaLight light_dome(Point::origin(),
                                   Vector(0, 0, -1),
                                   10.0f,
                                   Power::grey(680.0f),
                                   10000);

    OmnidirectionalLight light_point(Point(-3.5f, 0.0f, -5.0f),
                                     Power::grey(300.0f));

    OmnidirectionalLight light_point2(Point(1.0f, 0.0f, -1.0f),
                                     Power::grey(300.0f));

    DirectionalLight light_rays(Vector(-1.0f, -1.0f, -1.0f),
                                Biradiance::grey(0.1f));

    LightContainer lights;
    //lights.addLight(light_rays);
    lights.addLight(light_point);
    lights.addLight(light_point2);

    Image image(800, 600);
    image.setBackgroundColor(Color::black());

    Scene scene(scene_graph,
                lights,
                camera,
                image);

    scene.n_splits = 0;
    scene.use_direct_lighting = 1;
    scene.remove_hidden_faces = 1;
    scene.perform_clipping = 1;
    scene.draw_faces = 1;
    scene.draw_edges = 0;
    scene.edge_brightness = 0.6f;
    
    scene.use_omp = 1;

    scene.generateGround(-2.0f, 0.49f, ground_material);
    
    clock_t begin = std::clock();

    //scene.renderDirect();
    scene.rayTrace();
    //scene.rasterize();

    clock_t end = std::clock();
    double elapsed_time = double(end - begin)/CLOCKS_PER_SEC;
    std::cout << "Rendering time: " << elapsed_time << " s" << std::endl;

    showImage<float>(argc, argv, scene.getImage());
}
