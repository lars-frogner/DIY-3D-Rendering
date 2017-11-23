#include "RenderableTriangleMesh.hpp"
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
#include "Simulator.hpp"
#include "Animator.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Triangle3.hpp"
#include "Ray.hpp"
#include "Box.hpp"
#include "Sphere.hpp"
#include "TriangleMesh.hpp"
#include "Camera.hpp"
#include "Transformation.hpp"
#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"
#include "ProjectiveTransformation.hpp"
#include <glut.h>
#include <freeglut.h>
#include <vector>
#include <ctime>

using namespace Impact;

Rendering3D::Animator* ANIMATOR_PTR;

void renderScene()
{
	glClearColor(0, 0, 0, 1);
	glClear(GL_COLOR_BUFFER_BIT);

	ANIMATOR_PTR->updateScene();

	const Rendering3D::Image& image = ANIMATOR_PTR->getImage();

	glDrawPixels(static_cast<GLsizei>(image.getWidth()),
				 static_cast<GLsizei>(image.getHeight()),
				 GL_RGB, GL_FLOAT,
			     image.getRawPixelArray());

	//ANIMATOR_PTR->drawInfo();

	glutSwapBuffers();
}

void pressKey(unsigned char key, int x, int y)
{
	if (key == 27)
	{
		glutLeaveMainLoop();
	}

	ANIMATOR_PTR->startCameraMove(key);
}

void releaseKey(unsigned char key, int x, int y)
{
	ANIMATOR_PTR->stopCameraMove(key);
}

void moveMouse(int x, int y)
{
	ANIMATOR_PTR->rotateCamera(x, y);
}

void startMainLoop(Rendering3D::Animator& animator, int argc, char *argv[])
{
	glutInit(&argc, argv);

	const Rendering3D::Image& image = animator.getImage();

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(static_cast<int>(image.getWidth()),
					   static_cast<int>(image.getHeight()));
	glutInitWindowPosition(100, 100);

	glutCreateWindow("Impact window");

	ANIMATOR_PTR = &animator;
	glutDisplayFunc(renderScene);
	glutIdleFunc(renderScene);

	glutKeyboardFunc(pressKey);
	glutKeyboardUpFunc(releaseKey);
	glutPassiveMotionFunc(moveMouse);

	glutIgnoreKeyRepeat(1);

	animator.initialize();

	glutMainLoop();
}

Rendering3D::Scene getScene()
{
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;
	typedef Geometry3D::Ray Ray;
	typedef Geometry3D::Camera Camera;

	typedef Rendering3D::Image Image;

	typedef Rendering3D::Scene Scene;
	
    Camera camera(Ray(Point(0.0f, 0.0f, 0.0f), Vector(0.2f, 0.0f, -1.0f)),
                  Vector(0.0f, 1.0f, 0),
                  1.0f, 100,
                  45);

    Image image(800, 600);

    Scene scene(camera,
                image);

	scene.gamma_encode = 1;
    scene.use_omp = 0;

	return scene;
}

Rendering3D::LightContainer getLights()
{
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;

	typedef Rendering3D::Power Power;
	typedef Rendering3D::Biradiance Biradiance;
	typedef Rendering3D::DirectionalLight DirectionalLight;
	typedef Rendering3D::OmnidirectionalLight OmnidirectionalLight;
	typedef Rendering3D::LightContainer LightContainer;
	
    DirectionalLight light_rays(Vector(0.8f, -0.3f, -0.3f),
                                Biradiance::grey(5.0f));
    DirectionalLight light_rays2(Vector(-0.2f, 0.7f, 0.6f),
                                Biradiance::grey(3.0f));
	
    OmnidirectionalLight light_point(Point(2.2f, 0.6f, 0.4f),
                                     Power::grey(600.0f));

	LightContainer lights;
	lights.addLight(light_point);

	return lights;
}

/*
void render_teapot(int argc, char *argv[])
{
    typedef Geometry3D::Point Point;
    typedef Geometry3D::Vector Vector;
    typedef Geometry3D::Triangle Triangle;
    typedef Geometry3D::Ray Ray;
    typedef Geometry3D::Box Box;
    typedef Geometry3D::Sphere Sphere;
    typedef Geometry3D::TriangleMesh TriangleMesh;
    typedef Geometry3D::Camera Camera;
    typedef Geometry3D::LinearTransformation LinearTransformation;
    typedef Geometry3D::AffineTransformation AffineTransformation;
    typedef Geometry3D::ProjectiveTransformation ProjectiveTransformation;
    typedef Rendering3D::SceneGraph SceneGraph;
    typedef Rendering3D::Image Image;
    typedef Rendering3D::Color Color;
    typedef Rendering3D::Power Power;
    typedef Rendering3D::Reflectance Reflectance;
    typedef Rendering3D::Biradiance Biradiance;
    typedef Rendering3D::BlinnPhongMaterial BlinnPhongMaterial;
    typedef Rendering3D::Light Light;
    typedef Rendering3D::RectangularAreaLight RectangularAreaLight;
    typedef Rendering3D::HemisphereAreaLight HemisphereAreaLight;
    typedef Rendering3D::OmnidirectionalLight OmnidirectionalLight;
    typedef Rendering3D::DirectionalLight DirectionalLight;
    typedef Rendering3D::LightContainer LightContainer;
    typedef Rendering3D::Scene Scene;

    BlinnPhongMaterial ground_material(Reflectance::grey(0.1f),
                                       Reflectance::grey(0.1f),
                                       10.0f);

    BlinnPhongMaterial blue_material(Reflectance(0.2f, 0.2f, 0.7f),
                                     Reflectance(0.0f, 0.0f, 0.0f),
                                     100.0f);

    BlinnPhongMaterial red_material(Reflectance(0.7f, 0.2f, 0.2f),
                                    Reflectance(0.3f, 0.0f, 0.0f),
                                    100.0f);

    BlinnPhongMaterial sphere_material(Color(0.2f, 0.2f, 0.7f),
                                       1.0f, 0.4f, 0.7f);

    BlinnPhongMaterial teapot_material(Reflectance::red(), Reflectance::grey(0.35f), 800.0f);

    BlinnPhongMaterial diffuse_brown(Reflectance(0x3E211B), Reflectance::grey(0.2f), 40.0f);


    TriangleMesh teapot = TriangleMesh::file("teapot2.obj");
    TriangleMesh table_top = TriangleMesh::box(Box(Point(-0.5f, 0.0f, -0.5f), 1.0f, 0.1f, 1.0f)); table_top.setMaterial(diffuse_brown);
    TriangleMesh table_leg = TriangleMesh::box(Box(Point(-0.5f, 0.0f, -0.5f), 1.0f, 15.0f, 1.0f)); table_leg.setMaterial(diffuse_brown);
    TriangleMesh ground = TriangleMesh::sheet(Point(-50.0f, 0.0f, 5.0e3f),
                                              Vector(100.0f, 0.0f, 0.0f),
                                              Vector(0.0f, 0.0f, -1.0e4f)); ground.setMaterial(ground_material);
    
    SceneGraph scene_graph(LinearTransformation::scaling(1.0f));

    scene_graph.addTransformation(AffineTransformation::translation(0.0f, -0.5f, 0.0f))
               ->addObject(ground);

    scene_graph.addTransformation(AffineTransformation::translation(0.0f, 0.0f, -4.0f))
               ->addTransformation(LinearTransformation::rotationFromYToZ(M_PI_2))
               ->addObject(ground);

    scene_graph.addTransformation(AffineTransformation::translation(-2.0f, 0.0f, 0.0f))
               ->addTransformation(LinearTransformation::rotationFromZToX(M_PI_2))
               ->addTransformation(LinearTransformation::rotationFromYToZ(M_PI_2))
               ->addObject(ground);

    SceneGraph* objects_branch = scene_graph.addTransformation(AffineTransformation::translation(0.0f, 0.0f, -2.0f));

    objects_branch->addTransformation(AffineTransformation::translation(0.0f, 0.0f, 0.0f))
                  ->addTransformation(LinearTransformation::scaling(1.0f/180.0f))
                  ->addTransformation(LinearTransformation::rotationFromZToX(0.2f))
                  ->addObject(teapot.withMaterialReplaceOld(teapot_material));

    SceneGraph* table_branch = objects_branch->addTransformation(AffineTransformation::translation(0.0f, 0.0f, 0.0f))
                                             ->addTransformation(LinearTransformation::scaling(1.0f, 1.0f/2.0f, 1.0f))
                                             ->addTransformation(AffineTransformation::translation(0.0f, -1.0f, 0.0f))
                                             ->addTransformation(LinearTransformation::scaling(1.0f/17.0f));

    table_branch->addTransformation(AffineTransformation::translation(0.0f, 15.0f, 0.0f))
                ->addTransformation(LinearTransformation::scaling(20.0f, 20.0f, 20.0f))
                ->addObject(table_top);

    table_branch->addTransformation(AffineTransformation::translation(9.0f, 0.0f, 9.0f))
                ->addObject(table_leg);

    table_branch->addTransformation(AffineTransformation::translation(9.0f, 0.0f, -9.0f))
                ->addObject(table_leg);

    table_branch->addTransformation(AffineTransformation::translation(-9.0f, 0.0f, 9.0f))
                ->addObject(table_leg);

    table_branch->addTransformation(AffineTransformation::translation(-9.0f, 0.0f, -9.0f))
                ->addObject(table_leg);

    OmnidirectionalLight light_point1(Point(2.2f, 0.6f, 0.4f),
                                      Power::grey(300.0f));

    OmnidirectionalLight light_point2(Point(0.0f, 5.0f, -2.0f),
                                      Power::grey(10.0f));
	
    LightContainer lights;
	lights.addLight(light_point1);
    lights.addLight(light_point2);

    Camera camera(Ray(Point(0.8f, 0.8f, 0.0f), Vector(-0.4f, -0.3f, -1.0f)),
                  Vector(0.0f, 1.0f, 0),
                  0.1f, 100,
                  45);

    Image image(800, 600);
    image.setBackgroundColor(Color::black());

    Scene scene(lights,
                camera,
                image);

    scene.use_direct_lighting = 1;
    scene.remove_hidden_faces = 1;
    scene.perform_clipping = 1;
    scene.draw_faces = 1;
    
    scene.use_omp = 1;
    
    clock_t begin = std::clock();

    scene.rayTrace(scene_graph);

    clock_t end = std::clock();
    double elapsed_time = double(end - begin)/CLOCKS_PER_SEC;
    std::cout << "Rendering time: " << elapsed_time << " s" << std::endl;

    showImage(argc, argv, scene.getImage());
}
*/

int main(int argc, char *argv[])
{
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Box Box;
	typedef Geometry3D::Sphere Sphere;

	typedef Geometry3D::LinearTransformation LinearTransformation;
	typedef Geometry3D::AffineTransformation AffineTransformation;
	
	typedef Rendering3D::RenderableTriangleMesh RenderableTriangleMesh;

	typedef Rendering3D::Color Color;
	typedef Rendering3D::Reflectance Reflectance;
	typedef Rendering3D::BlinnPhongMaterial BlinnPhongMaterial;

	typedef Rendering3D::Biradiance Biradiance;
	typedef Rendering3D::DirectionalLight DirectionalLight;
	typedef Rendering3D::LightContainer LightContainer;

	typedef Rendering3D::Simulator Simulator;
	typedef Rendering3D::Animator Animator;

	BlinnPhongMaterial blue_material(Reflectance(0.2f, 0.2f, 0.7f),
								  	 Reflectance(0.2f, 0.2f, 0.2f),
									 100.0f);

	BlinnPhongMaterial red_material(Reflectance(0.7f, 0.2f, 0.2f),
								  	Reflectance(0.2f, 0.2f, 0.2f),
									100.0f);

	BlinnPhongMaterial teapot_material(Reflectance::red(), Reflectance::grey(0.35f), 400.0f);

	BlinnPhongMaterial diffuse_brown(Reflectance(0x3E211B), Reflectance::grey(0.2f), 40.0f);

	Simulator simulator;
	Animator animator(simulator, getScene(), getLights());

	Box box1(Point(-0.5f, -0.5f, -0.5f), 1, 1, 1);
	box1.rotateFromXToY(0.2f);
	box1.rotateFromYToZ(0.2f);
	box1.origin.translate(-1.0, 0.0, -5.0);

	Box box2(Point(-0.5f, -0.5f, -0.5f), 1, 1, 1);
	box2.rotateFromYToZ(0.4f);
	box2.rotateFromZToX(0.2f);
	box2.origin.translate(1.0, 0.0, -5.0);

	Box box3(Point::origin(), 8.0f, 3.0f, 8.0f);
	box3.setCenter(Point::origin());
	box3.origin.translate(0.0f, 0.5f, -5.0f);

	Sphere sphere1(Point::origin(), 0.1f);
	sphere1.center.translate(0.0f, 0.5f, -5.0f);

	/*RenderableTriangleMesh teapot = RenderableTriangleMesh::file("teapot.obj");
	teapot.setMaterial(teapot_material);
	teapot.applyTransformation(LinearTransformation::scaling(1.0f/50.0f));
	teapot.applyTransformation(AffineTransformation::translation(0.0f, 0.0f, -5.0f));*/

	//animator.addDynamicBox(box1, blue_material);
	//animator.addDynamicBox(box2, red_material);
	animator.addMesh(RenderableTriangleMesh::room(box3).withMaterial(diffuse_brown));
	//animator.addMesh(teapot);
	animator.addMesh(RenderableTriangleMesh::sphere(sphere1, 5).withMaterial(teapot_material));

	//animator.getMesh(0).casts_shadows = 1;
	//animator.getMesh(1).casts_shadows = 1;

	animator.render_mode = 0;

	startMainLoop(animator, argc, argv);
}
