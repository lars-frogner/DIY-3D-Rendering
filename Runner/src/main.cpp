#include "precision.hpp"
#include "World.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Box.hpp"
#include "Sphere.hpp"
#include "mesh_assets.hpp"
#include "Color.hpp"
#include "Material.hpp"
#include "BlinnPhongMaterial.hpp"
#include "OmnidirectionalLight.hpp"
#include "DirectionalLight.hpp"
#include "material_assets.hpp"
#include <glut.h>
#include <freeglut.h>
#include <random>
#include "Matrix3.hpp"
#include "Matrix4.hpp"
#include "AffineTransformation.hpp"
#include "AffineTransformation.hpp"
#include "LinearTransformation.hpp"
#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"
#include "AffineTransformation.hpp"

using namespace Impact;
using namespace Geometry3D;
using namespace Physics3D;
using namespace Rendering3D;

World* WORLD = nullptr;

/* Create mainloop functions */

void render()
{
	WORLD->render();
}

void processKeyPress(unsigned char key, int x, int y)
{
	WORLD->processKeyPress(key, x, y);
}

void processKeyRelease(unsigned char key, int x, int y)
{
	WORLD->processKeyRelease(key, x, y);
}

void processMouseMovement(int x, int y)
{
	WORLD->processMouseMovement(x, y);
}

void processMouseClick(int button, int state, int x, int y)
{
	WORLD->processMouseClick(button, state, x, y);
}

void startMainLoop(int argc, char *argv[])
{
	glutInit(&argc, argv);

	WORLD->initialize();

	glutDisplayFunc(render);
	glutIdleFunc(render);

	glutKeyboardFunc(processKeyPress);
	glutKeyboardUpFunc(processKeyRelease);
	glutPassiveMotionFunc(processMouseMovement);
	glutMouseFunc(processMouseClick);

	glutMainLoop();
}

void setupGravityDragTest()
{
	WORLD->addRoom(20.0f, 10.0f, 20.0f, &IMP_DIFFUSE_DARKSLATEGREY);
	
	WORLD->addLight(new OmnidirectionalLight(Point(3.0f, 4.0f, 6.0f),
						 			         Power::grey(3000.0f)));

	WORLD->addParticle(Point(0.0f, 1.0f, 0.0f),
					   Vector(3.0f, 10.0f, 0),
					   Vector::zero(),
					   1.0f, 0.1f, 1.0f,
					   &IMP_SHINY_GREEN);
	
	WORLD->addParticle(Point(0.0f, 1.0f, 0.0f),
					   Vector(3.0f, 10.0f, 0),
					   Vector::zero(),
					   1.0f, 0.1f, 1.0f,
					   &IMP_SHINY_RED);

	WORLD->addUniformGravityForce(Vector(0, -9.81f, 0), -1);
	WORLD->addDragForce(0.2f, 0.05f, 0);
}

void setupSpringTest()
{
	WORLD->addRoom(20.0f, 10.0f, 20.0f, &IMP_DIFFUSE_DARKSLATEGREY);
	
	WORLD->addLight(new OmnidirectionalLight(Point(3.0f, 4.0f, 6.0f),
								             Power::grey(3000.0f)));

	imp_float gravity = 0;
	imp_float damping = 1.0;

	WORLD->addParticle(Point(-4.0f, 4.0f, 0.0f),
					   Vector(0, 0, 0),
					   Vector(0, gravity, 0),
					   1.0f, 0.1f, damping,
					   &IMP_SHINY_FORESTGREEN);

	WORLD->addAnchoredSpringForce(Point(-4.0f, 6.0f, 0.0f),
						   5.0f, 1.0f, 0);

	WORLD->addParticle(Point(-2.0f, 4.0f, 0.0f),
					   Vector(0, 0, 0),
					   Vector(0, gravity, 0),
					   1.0f, 0.1f, damping,
					   &IMP_SHINY_CRIMSON);

	WORLD->addAnchoredBungeeForce(Point(-2.0f, 6.0f, 0.0f),
								  5.0f, 1.0f, 1);

	WORLD->addParticle(Point(0.0f, 6.0f, 0.0f),
					   Vector(0, 0, 0),
					   Vector(0, gravity, 0),
					   1.0f, 0.1f, damping,
					   &IMP_SHINY_BLUE);

	WORLD->addParticle(Point(0.0f, 2.0f, 0.0f),
					   Vector(0, 0, 0),
					   Vector(0, gravity, 0),
					   1.0f, 0.1f, damping,
					   &IMP_SHINY_CYAN);

	WORLD->addSpringForce(5.0f, 1.5f, 2, 3);

	WORLD->addParticle(Point(2.0f, 6.0f, 0.0f),
					   Vector(0, 0, 0),
					   Vector(0, gravity, 0),
					   1.0f, 0.1f, damping,
					   &IMP_SHINY_ORANGE);

	WORLD->addParticle(Point(2.0f, 2.0f, 0.0f),
					   Vector(0, 0, 0),
					   Vector(0, gravity, 0),
					   1.0f, 0.1f, damping,
					   &IMP_SHINY_MAGENTA);

	WORLD->addBungeeForce(5.0f, 1.5f, 4, 5);

	/*WORLD->addParticle(Point(4.0f, 4.0f, 0.0f),
					   Vector(0, 0, 0),
					   Vector(0, gravity, 0),
					   1.0f, 0.1f, 1.0f,
					   &IMP_SHINY_GOLD);

	WORLD->addAnchoredStiffSpringForce(Point(4.0f, 4.2f, 0.0f),
									   1000.0f, 1.0f, 6);*/
}

void setupPlaneContactTest()
{
	imp_float width = 20.0f;
	imp_float height = 8.0f;
	imp_float depth = 20.0f;

	imp_float restitution_coef = 0.9f;

	WORLD->addRoom(width, height, depth, &IMP_DIFFUSE_DARKSLATEGREY);
	
	WORLD->addLight(new OmnidirectionalLight(Point(3.0f, 4.0f, 6.0f),
										     Power::grey(3000.0f)));

	WORLD->addParticle(Point(0.0f, 2.0f, 0.0f),
					   Vector(0.0f, 0.0f, 0.0f),
					   Vector(0, 0.0f, 0),
					   1.0f, 0.5f, 1.0f,
					   &IMP_SHINY_FORESTGREEN);

	WORLD->addParticle(Point(0.95f, 2.0f, 0.0f),
					   Vector(0.0f, 0.0f, 0.0f),
					   Vector(0, 0, 0),
					   1.0f, 0.5f, 1.0f,
					   &IMP_SHINY_CRIMSON);

	WORLD->addParticle(Point(0.5f, 2.8f, 0.0f),
					   Vector(0.0f, 0.0f, 0.0f),
					   Vector(0, -9.81f, 0),
					   1.0f, 0.5f, 1.0f,
					   &IMP_SHINY_NAVY);

	//WORLD->addDragForce(0.02f, 0.003f, 0);

	//WORLD->addCableContact(1.0f, restitution_coef, 0, 1);
	//WORLD->addRodContact(1.0f, 1, 2);
	//WORLD->addGroundContact(restitution_coef, -1);
	WORLD->addAllParticleContacts(restitution_coef);
	//WORLD->addRoomContact(width, height, depth, restitution_coef, -1);
}

void setupParticlesContactTest()
{
	imp_float width = 15.0f;
	imp_float height = 8.0f;
	imp_float depth = 20.0f;

	imp_uint mesh_quality = 0;
	imp_uint n_particles_per_axis = 8;
	imp_float wall_separation_x = 2.0f;
	imp_float wall_separation_y = 2.0f;
	imp_float wall_separation_z = 2.0f;
	imp_float x_shift = 0.3f;
	imp_float y_shift = 0.3f;
	imp_float z_shift = 0.3f;
	imp_float mass = 1.0f;
	imp_float radius = 0.2f;
	imp_float damping = 1.0f;
	imp_float restitution_coef = 0.9f;
	imp_float gravitational_constant = 0.01f;
	Vector velocity(0, 0, 0);
	Vector gravity(0, -9.81f, 0);
	//Vector gravity(0, 0, 0);

	const Material* materials[3] = {&IMP_SHINY_FORESTGREEN, &IMP_SHINY_NAVY, &IMP_SHINY_CRIMSON};
	
	WORLD->addRoom(width, height, depth, &IMP_DIFFUSE_DARKSLATEGREY);
	
	Point light_position(0.0f, 7.5f, 7.0f);
	Power light_power = Power::grey(1000.0f);
	Color light_color = light_power/light_power.getMax();
	WORLD->addMaterial(new BlinnPhongMaterial(light_color, Color::black(), 1));

	WORLD->addLight(new OmnidirectionalLight(light_position,
											 light_power));
	WORLD->addLight(new OmnidirectionalLight(light_position,
											 light_power/20));
	WORLD->getLight(1)->creates_shadows = false;

	/*WORLD->addSphere(Sphere(light_position, 0.05f), WORLD->getMaterial(0), 0);
	WORLD->getModel(1)->shadows_toggable = false;
	WORLD->getModel(1)->uses_direct_lighting = false;*/

	imp_float x_start = -width/2 + wall_separation_x, x_end = width/2 - wall_separation_x;
	imp_float y_start = wall_separation_y, y_end = height - wall_separation_y;
	imp_float z_start = -depth/2 + wall_separation_z, z_end = depth/2 - wall_separation_z;
	imp_float dx = (x_end - x_start)/(n_particles_per_axis - 1);
	imp_float dy = (y_end - y_start)/(n_particles_per_axis - 1);
	imp_float dz = (z_end - z_start)/(n_particles_per_axis - 1);
	imp_float x, y, z;
	imp_uint i, j, k;

	x_start -= x_shift*n_particles_per_axis/2;
	y_start -= y_shift*n_particles_per_axis/2;
	z_start -= z_shift*n_particles_per_axis/2;
	
	std::random_device random_device;
    std::mt19937 generator(random_device());
    std::uniform_int_distribution<> distribution(0, 2);
	
	y = y_start;
	for (j = 0; j < n_particles_per_axis; j++)
	{
		x = x_start + j*x_shift;
		for (i = 0; i < n_particles_per_axis; i++)
		{
			z = z_start + i*z_shift;
			for (k = 0; k < n_particles_per_axis; k++)
			{
				WORLD->addParticle(Point(x, y + k*y_shift - wall_separation_y/2, z),
								   velocity,
								   gravity,
								   mass, radius, damping,
								   materials[distribution(generator)],
								   mesh_quality);

				z += dz;
			}
			x += dx;
		}
		y += dy;
	}

	WORLD->addDragForce(0.02f, 0.003f, -1);

	WORLD->addAllParticleContacts(restitution_coef);
	//WORLD->addAllParticleGravityForces(gravitational_constant);
	WORLD->addRoomContact(width, height, depth, restitution_coef, -1);
}

void switchGravityMode(World* world)
{
	Vector gravity(0, -9.81f, 0);

	for (imp_uint idx = 0; idx < world->getNumberOfParticles(); idx++)
	{
		world->getParticle(idx)->setDefaultAcceleration(gravity);
	}
	
	world->clearAllForces();
	world->addDragForce(0.02f, 0.003f, -1);
}

void setupParticlesGravityContactTest()
{
	imp_float width = 15.0f;
	imp_float height = 10.0f;
	imp_float depth = 15.0f;

	imp_uint mesh_quality = 0;
	imp_uint n_particles_per_axis = 8;
	imp_float wall_separation_x = 3.5f;
	imp_float wall_separation_y = 1.0f;
	imp_float wall_separation_z = 3.5f;
	imp_float x_shift = 0.3f*0;
	imp_float y_shift = 0.3f*0;
	imp_float z_shift = 0.3f*0;
	imp_float mass = 1.0f;
	imp_float radius = 0.1f;
	imp_float damping = 1.0f;
	imp_float restitution_coef = 0.9f;
	imp_float gravitational_constant = 0.004f;
	//Vector velocity(0, 0, 0);
	//Vector gravity(0, -9.81f, 0);
	Vector gravity(0, 0, 0);

	const Material* materials[5] = {&IMP_SHINY_FORESTGREEN,
									&IMP_SHINY_NAVY,
									&IMP_SHINY_CRIMSON,
									&IMP_SHINY_GOLD,
									&IMP_SHINY_MAGENTA};
	
	WORLD->addRoom(width, height, depth, &IMP_DIFFUSE_DARKSLATEGREY);
	
	Point light_position(0.0f, 9.5f, 7.0f);
	Power light_power = Power::grey(1000.0f);
	Color light_color = light_power/light_power.getMax();
	WORLD->addMaterial(new BlinnPhongMaterial(light_color, Color::black(), 1));

	WORLD->addLight(new OmnidirectionalLight(light_position,
											 light_power));
	WORLD->addLight(new OmnidirectionalLight(light_position,
											 light_power/2));
	WORLD->getLight(1)->creates_shadows = false;

	/*WORLD->addSphere(Sphere(light_position, 0.05f), WORLD->getMaterial(0), 0);
	WORLD->getModel(1)->shadows_toggable = false;
	WORLD->getModel(1)->uses_direct_lighting = false;*/

	//WORLD->setTimeTrigger(switchGravityMode, 20.0);

	imp_float x_start = -width/2 + wall_separation_x, x_end = width/2 - wall_separation_x;
	imp_float y_start = wall_separation_y, y_end = height - wall_separation_y;
	imp_float z_start = -depth/2 + wall_separation_z, z_end = depth/2 - wall_separation_z;
	imp_float dx = (x_end - x_start)/(n_particles_per_axis - 1);
	imp_float dy = (y_end - y_start)/(n_particles_per_axis - 1);
	imp_float dz = (z_end - z_start)/(n_particles_per_axis - 1);
	imp_float x, y, z;
	imp_uint i, j, k;

	x_start -= x_shift*n_particles_per_axis/2;
	y_start -= y_shift*n_particles_per_axis/2;
	z_start -= z_shift*n_particles_per_axis/2;
	
	std::random_device random_device;
    std::mt19937 generator(random_device());
    std::uniform_int_distribution<> distribution(0, 4);
	
	y = y_start;
	for (j = 0; j < n_particles_per_axis; j++)
	{
		x = x_start + j*x_shift;
		for (i = 0; i < n_particles_per_axis; i++)
		{
			z = z_start + i*z_shift;
			for (k = 0; k < n_particles_per_axis; k++)
			{
				const Point& position = Point(x, y + k*y_shift - ((y_shift == 0)? 0 : wall_separation_y/2), z);
				const Vector& velocity = (Vector::unitY().cross(position - Point(0, height/2, 0))).getNormalized()*0.07f;
				WORLD->addParticle(position,
								   velocity,
								   gravity,
								   mass, radius, damping,
								   materials[distribution(generator)],
								   mesh_quality);

				z += dz;
			}
			x += dx;
		}
		y += dy;
	}
	
	WORLD->addAllGravityForces(gravitational_constant);

	//WORLD->addDragForce(0.02f, 0.003f, -1);

	WORLD->addAllParticleContacts(restitution_coef);
	WORLD->addRoomContact(width, height, depth, restitution_coef, -1);
}

int main(int argc, char *argv[])
{
	WORLD = new World(1280, 720);

	WORLD->setCameraPointing(Point(0, 2, 25));

	//setupGravityDragTest();
	//setupSpringTest();
	//setupPlaneContactTest();
	//setupParticlesContactTest();
	setupParticlesGravityContactTest();

	startMainLoop(argc, argv);
}
