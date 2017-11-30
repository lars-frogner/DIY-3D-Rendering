#include "World.hpp"
#include "CoordinateFrame.hpp"
#include "mesh_assets.hpp"
#include "material_assets.hpp"
#include "ParticleDragForce.hpp"
#include "ParticleUniformGravityForce.hpp"
#include "ParticleGravityForce.hpp"
#include "ParticleSpringForce.hpp"
#include "ParticleBungeeForce.hpp"
#include "ParticleAnchoredSpringForce.hpp"
#include "ParticleAnchoredBungeeForce.hpp"
#include "ParticleAnchoredStiffSpringForce.hpp"
#include "ParticlePlaneContact.hpp"
#include "ParticleCableContact.hpp"
#include "ParticleRodContact.hpp"
#include "ParticleParticleContact.hpp"
#include <glut.h>
#include <freeglut.h>

namespace Impact {
namespace Rendering3D {

World::World(imp_uint image_width,
			 imp_uint image_height)
	: _image(new Image(image_width, image_height)),
	  _camera(new Camera()),
	  _physics_world(new ParticleWorld(1000)),
	  _renderer(new Renderer(_image, _camera,
							 &_lights, &_objects)),
	  _animator(new Animator(_renderer, _physics_world)) {}

World::~World()
{
	delete _animator;
	delete _renderer;
	delete _camera;
	delete _image;

	clearParticleContacts();
	clearParticleForces();
	clearParticles();
	
	delete _physics_world;

	clearObjects();
	clearLights();
}

void World::setCameraPointing(const Point& position,
							  const Vector& look_direction,
							  const Vector& up_direction)
{
	const Geometry3D::CoordinateFrame& original_frame = _camera->getCoordinateFrame();
	const Geometry3D::CoordinateFrame& new_frame = Camera::getCoordinateFrame(position, look_direction, up_direction);

	_renderer->transformCameraLookRay(AffineTransformation::pointAndVectorsToPointAndVectors(original_frame.origin,
																							 original_frame.basis_1,
																							 original_frame.basis_2,
																							 original_frame.basis_3,
																						     new_frame.origin,
																							 new_frame.basis_1,
																							 new_frame.basis_2,
																							 new_frame.basis_3));
}
	
void World::addLight(Light* light)
{
	_lights.push_back(light);
}

void World::addObject(RenderableObject* object)
{
	_objects.push_back(object);
}

void World::addParticle(Particle* particle)
{
	_particles.push_back(particle);
	_physics_world->addParticle(particle);
}

void World::addParticleForceGenerator(ParticleForceGenerator* force_generator)
{
	_force_generators.push_back(force_generator);
}

void World::addParticleContact(ParticleContactGenerator* contact_generator)
{
	_contact_generators.push_back(contact_generator);
	_physics_world->addContactGenerator(contact_generator);
}

void World::addRenderableParticle(RenderableParticle* renderable_particle)
{
	addObject(renderable_particle);
	addParticle(renderable_particle->getParticle());
}

void World::addParticleForce(ParticleForceGenerator* force_generator, Particle* particle)
{
	_physics_world->addForceGenerator(particle, force_generator);
}

void World::clearLights()
{
	for (std::vector<Light*>::iterator iter = _lights.begin(); iter != _lights.end(); iter++)
	{
		if (*iter)
			delete *iter;
	}

	_lights.clear();
}

void World::clearObjects()
{
	for (std::vector<RenderableObject*>::iterator iter = _objects.begin(); iter != _objects.end(); iter++)
	{
		if (*iter)
			delete *iter;
	}

	_objects.clear();
}

void World::clearParticles()
{
	_physics_world->clearParticles();

	for (std::vector<Particle*>::iterator iter = _particles.begin(); iter != _particles.end(); iter++)
	{
		if (*iter)
			delete *iter;
	}

	_particles.clear();
}

void World::clearParticleForces()
{
	_physics_world->clearForceGenerators();

	for (std::vector<ParticleForceGenerator*>::iterator iter = _force_generators.begin(); iter != _force_generators.end(); iter++)
	{
		if (*iter)
			delete *iter;
	}

	_force_generators.clear();
}

void World::clearParticleContacts()
{
	_physics_world->clearContactGenerators();

	for (std::vector<ParticleForceGenerator*>::iterator iter = _force_generators.begin(); iter != _force_generators.end(); iter++)
	{
		if (*iter)
			delete *iter;
	}

	_contact_generators.clear();
}

imp_uint World::getNumberOfParticles() const
{
	return static_cast<imp_uint>(_particles.size());
}

void World::initialize()
{
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(static_cast<int>(_image->getWidth()),
					   static_cast<int>(_image->getHeight()));
	glutInitWindowPosition(100, 100);

	glutCreateWindow("Impact");

	glutIgnoreKeyRepeat(1);

	_image->use_omp = use_omp;
	_renderer->use_omp = use_omp;

	_renderer->initialize();
	_animator->initialize(_image->getWidth(), _image->getHeight());
}

void World::render()
{
	glClearColor(0, 0, 0, 1);
	glClear(GL_COLOR_BUFFER_BIT);

	_animator->updateFrame();

	glDrawPixels(static_cast<GLsizei>(_image->getWidth()),
				 static_cast<GLsizei>(_image->getHeight()),
				 GL_RGB, GL_FLOAT,
			     _image->getRawPixelArray());

	glutSwapBuffers();
}

void World::processKeyPress(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'p':
		_animator->togglePhysics();
		break;
	case 'o':
		_animator->cycleRenderingMode();
		break;
	case 'u':
		_renderer->toggleObjectShadows();
		break;
	case 'c':
		_animator->toggleInteractiveCamera();
		break;
	case 'r':
		_animator->toggleRealtimeSimulation();
		break;
	case 'j':
		_animator->decreaseFixedSimulationTimestep();
		break;
	case 'k':
		_animator->increaseFixedSimulationTimestep();
		break;
	case 'n':
		_animator->decreaseSimulationFrequency();
		break;
	case 'm':
		_animator->increaseSimulationFrequency();
		break;
	case 'b':
		_animator->toggleAutomaticSimulationTimestep();
		break;
	case ',':
		_animator->decreaseSimulationPriority();
		break;
	case '.':
		_animator->increaseSimulationPriority();
		break;
	case 'i':
		_animator->toggleInfoPrinting();
		break;
	case 'z':
		_animator->cycleDepthMapRendering();
		break;
	case 'l':
		_animator->toggleEdgeRendering();
		break;
	case 't':
		_animator->toggleRecording();
		break;
	case 'g':
		_animator->toggleGammaEncoding();
		break;
	case 'y':
		toggleParallelization();
		break;
	case 'x':
		_animator->toggleSinglestepping();
		break;
	case 'v':
		_animator->performSingleStep();
		break;
	case 'f':
		addParticle(_camera->getPosition() + _camera->getLookDirection()*3.0f,
					_camera->getLookDirection()*20.0f,
					Vector::zero(),
					15.0f, 0.1f, 1.0f, &IMP_SHINY_GOLD);
		addParticleContacts(0.9f, getNumberOfParticles()-1);
		break;
	case 27: // escape
		_animator->terminate();
		break;
	default:
		break;
	}

	_animator->startCameraMove(key);
}

void World::processKeyRelease(unsigned char key, int x, int y)
{
	_animator->stopCameraMove(key);
}

void World::processMouseMovement(int x, int y)
{
	_animator->rotateCamera(x, y);
}

void World::processMouseClick(int button, int state, int x, int y)
{
	
}

void World::toggleParallelization()
{
	use_omp = !use_omp;

	_image->use_omp = use_omp;
	_renderer->use_omp = use_omp;
}

void World::addRoom(imp_float width, imp_float height, imp_float depth, const Material* material)
{
	addObject(new RenderableObject(&Geometry3D::IMP_ROOM_MESH,
								   material,
								   LinearTransformation::scaling(width, height, depth)));
}

void World::addGround(imp_float width, imp_float depth, const Material* material)
{
	addObject(new RenderableObject(&Geometry3D::IMP_SHEET_MESH,
								   material,
								   LinearTransformation::scaling(width, 1, depth)));
}

void World::addBox(const Box& box, const Material* material)
{
	addObject(new RenderableObject(&Geometry3D::IMP_BOX_MESH,
								   material,
								   AffineTransformation::translationTo(box.getCenter())*
								   LinearTransformation::vectorsToVectors(Vector::unitX(), Vector::unitY(), Vector::unitZ(),
									  									  box.getWidthVector(), box.getHeightVector(), box.getDepthVector())));
}

void World::addSphere(const Sphere& sphere,
					  const Material* material,
					  imp_uint quality /* = 0 */)
{
	assert(quality < IMP_N_SPHERE_MESHES);

	addObject(new RenderableObject(Geometry3D::IMP_SPHERE_MESHES + quality,
								   material,
								   AffineTransformation::translationTo(sphere.center)*
								   LinearTransformation::scaling(sphere.radius)));
}

void World::addParticle(const Point& position,
						const Vector& velocity,
						const Vector& acceleration,
						imp_float mass,
						imp_float radius,
						imp_float damping,
						const Material* material,
						imp_uint quality /* = 0 */)
{
	assert(quality < IMP_N_SPHERE_MESHES);

	addRenderableParticle(new RenderableParticle(Geometry3D::IMP_SPHERE_MESHES + quality,
								   material,
												 new Particle(position,
															  velocity,
															  acceleration,
															  mass,
															  radius,
															  damping),
											     LinearTransformation::scaling(radius)));
}

void World::addDragForce(imp_float coef_1,
						 imp_float coef_2,
						 imp_int particle_idx)
{
	Physics3D::ParticleDragForce* force_generator = new Physics3D::ParticleDragForce(coef_1, coef_2);
	addParticleForceGenerator(force_generator);

	if (particle_idx < 0)
	{
		for (std::vector<Particle*>::const_iterator iter = _particles.begin(); iter != _particles.end(); iter++)
		{
			addParticleForce(force_generator, *iter);
		}
	}
	else
	{
		assert(static_cast<imp_uint>(particle_idx) < getNumberOfParticles());
		addParticleForce(force_generator, _particles[particle_idx]);
	}
}

void World::addUniformGravityForce(const Vector& gravity,
								   imp_int particle_idx)
{
	Physics3D::ParticleUniformGravityForce* force_generator = new Physics3D::ParticleUniformGravityForce(gravity);
	addParticleForceGenerator(force_generator);

	if (particle_idx < 0)
	{
		for (std::vector<Particle*>::const_iterator iter = _particles.begin(); iter != _particles.end(); iter++)
		{
			addParticleForce(force_generator, *iter);
		}
	}
	else
	{
		assert(static_cast<imp_uint>(particle_idx) < getNumberOfParticles());
		addParticleForce(force_generator, _particles[particle_idx]);
	}
}

void World::addGravityForce(imp_float gravitational_constant,
							imp_uint particle_idx_1,
							imp_uint particle_idx_2)
{
	assert(particle_idx_1 < getNumberOfParticles() && particle_idx_2 < getNumberOfParticles());

	Physics3D::ParticleGravityForce* force_generator = new Physics3D::ParticleGravityForce(_particles[particle_idx_1],
																						   _particles[particle_idx_2],
																						   gravitational_constant);

	addParticleForce(force_generator, _particles[particle_idx_1]);
	addParticleForce(force_generator, _particles[particle_idx_2]);
}

void World::addAllGravityForces(imp_float gravitational_constant)
{
	imp_uint i, j;
	imp_uint n_particles = getNumberOfParticles();

	for (i = 0; i < n_particles-1; i++)
	{
		for (j = i+1; j < n_particles; j++)
		{
			addGravityForce(gravitational_constant, i, j);
		}
	}
}

void World::addSpringForce(imp_float spring_constant,
						   imp_float rest_length,
						   imp_uint particle_idx_1,
						   imp_uint particle_idx_2)
{
	assert(particle_idx_1 < getNumberOfParticles() && particle_idx_2 < getNumberOfParticles());

	Physics3D::ParticleSpringForce* force_generator = new Physics3D::ParticleSpringForce(_particles[particle_idx_1],
																						 _particles[particle_idx_2],
																						 spring_constant, rest_length);

	addParticleForce(force_generator, _particles[particle_idx_1]);
	addParticleForce(force_generator, _particles[particle_idx_2]);
}

void World::addBungeeForce(imp_float spring_constant,
						   imp_float rest_length,
						   imp_uint particle_idx_1,
						   imp_uint particle_idx_2)
{
	assert(particle_idx_1 < getNumberOfParticles() && particle_idx_2 < getNumberOfParticles());

	Physics3D::ParticleBungeeForce* force_generator = new Physics3D::ParticleBungeeForce(_particles[particle_idx_1],
																						 _particles[particle_idx_2],
																						 spring_constant, rest_length);

	addParticleForce(force_generator, _particles[particle_idx_1]);
	addParticleForce(force_generator, _particles[particle_idx_2]);
}

void World::addAnchoredSpringForce(const Point& anchor_point,
								   imp_float spring_constant,
								   imp_float rest_length,
								   imp_int particle_idx)
{
	Physics3D::ParticleAnchoredSpringForce* force_generator = new Physics3D::ParticleAnchoredSpringForce(anchor_point,
																									     spring_constant,
																									     rest_length);
	addParticleForceGenerator(force_generator);

	if (particle_idx < 0)
	{
		for (std::vector<Particle*>::const_iterator iter = _particles.begin(); iter != _particles.end(); iter++)
		{
			addParticleForce(force_generator, *iter);
		}
	}
	else
	{
		assert(static_cast<imp_uint>(particle_idx) < getNumberOfParticles());
		addParticleForce(force_generator, _particles[particle_idx]);
	}
}

void World::addAnchoredBungeeForce(const Point& anchor_point,
								   imp_float spring_constant,
								   imp_float rest_length,
								   imp_int particle_idx)
{
	Physics3D::ParticleAnchoredBungeeForce* force_generator = new Physics3D::ParticleAnchoredBungeeForce(anchor_point,
																									     spring_constant,
																									     rest_length);
	addParticleForceGenerator(force_generator);

	if (particle_idx < 0)
	{
		for (std::vector<Particle*>::const_iterator iter = _particles.begin(); iter != _particles.end(); iter++)
		{
			addParticleForce(force_generator, *iter);
		}
	}
	else
	{
		assert(static_cast<imp_uint>(particle_idx) < getNumberOfParticles());
		addParticleForce(force_generator, _particles[particle_idx]);
	}
}

void World::addAnchoredStiffSpringForce(const Point& anchor_point,
										imp_float spring_constant,
										imp_float damping,
										imp_int particle_idx)
{
	Physics3D::ParticleAnchoredStiffSpringForce* force_generator = new Physics3D::ParticleAnchoredStiffSpringForce(anchor_point,
																												   spring_constant,
																												   damping);
	addParticleForceGenerator(force_generator);

	if (particle_idx < 0)
	{
		for (std::vector<Particle*>::const_iterator iter = _particles.begin(); iter != _particles.end(); iter++)
		{
			addParticleForce(force_generator, *iter);
		}
	}
	else
	{
		assert(static_cast<imp_uint>(particle_idx) < getNumberOfParticles());
		addParticleForce(force_generator, _particles[particle_idx]);
	}
}

void World::addPlaneContact(const Plane& plane,
							imp_float restitution_coef,
							imp_uint particle_idx)
{
	assert(particle_idx < getNumberOfParticles());

	addParticleContact(new Physics3D::ParticlePlaneContact(_particles[particle_idx],
														   plane,
														   restitution_coef));
}

void World::addCableContact(imp_float cable_length,
							imp_float restitution_coef,
							imp_uint particle_idx_1,
							imp_uint particle_idx_2)
{
	assert(particle_idx_1 < getNumberOfParticles() && particle_idx_2 < getNumberOfParticles());

	addParticleContact(new Physics3D::ParticleCableContact(_particles[particle_idx_1],
														   _particles[particle_idx_2],
														   cable_length,
														   restitution_coef));
}

void World::addRodContact(imp_float rod_length,
						  imp_uint particle_idx_1,
						  imp_uint particle_idx_2)
{
	assert(particle_idx_1 < getNumberOfParticles() && particle_idx_2 < getNumberOfParticles());

	addParticleContact(new Physics3D::ParticleRodContact(_particles[particle_idx_1],
														 _particles[particle_idx_2],
														 rod_length));
}

void World::addParticleContact(imp_float restitution_coef,
							   imp_uint particle_idx_1,
							   imp_uint particle_idx_2)
{
	assert(particle_idx_1 < getNumberOfParticles() && particle_idx_2 < getNumberOfParticles());

	addParticleContact(new Physics3D::ParticleParticleContact(_particles[particle_idx_1],
															  _particles[particle_idx_2],
															  restitution_coef));
}

void World::addParticleContacts(imp_float restitution_coef,
								imp_uint particle_idx)
{
	imp_uint i;

	for (i = 0; i < particle_idx; i++)
	{
		addParticleContact(restitution_coef, particle_idx, i);
	}

	for (i = particle_idx+1; i < getNumberOfParticles(); i++)
	{
		addParticleContact(restitution_coef, particle_idx, i);
	}
}

void World::addAllParticleContacts(imp_float restitution_coef)
{
	imp_uint i, j;
	imp_uint n_particles = getNumberOfParticles();

	for (i = 0; i < n_particles-1; i++)
	{
		for (j = i+1; j < n_particles; j++)
		{
			addParticleContact(restitution_coef, i, j);
		}
	}
}

void World::addRoomContact(imp_float width,
						   imp_float height,
						   imp_float depth,
						   imp_float restitution_coef,
						   imp_int particle_idx)
{
	Plane floor(Point(0, 0, 0), Vector::unitY());
	Plane ceiling(Point(0, height, 0), -Vector::unitY());
	Plane left_wall(Point(-width/2, 0, 0), Vector::unitX());
	Plane right_wall(Point(width/2, 0, 0), -Vector::unitX());
	Plane front_wall(Point(0, 0, -depth/2), -Vector::unitZ());
	Plane back_wall(Point(0, 0, depth/2), Vector::unitZ());
	
	imp_uint start_idx;
	imp_uint end_idx;

	if (particle_idx < 0)
	{
		start_idx = 0;
		end_idx = getNumberOfParticles();
	}
	else
	{
		start_idx = static_cast<imp_uint>(particle_idx);
		end_idx = start_idx + 1;
		assert(start_idx < getNumberOfParticles());
	}

	for (imp_uint idx = start_idx; idx < end_idx; idx++)
	{
		addPlaneContact(floor, restitution_coef, idx);
		addPlaneContact(ceiling, restitution_coef, idx);
		addPlaneContact(left_wall, restitution_coef, idx);
		addPlaneContact(right_wall, restitution_coef, idx);
		addPlaneContact(front_wall, restitution_coef, idx);
		addPlaneContact(back_wall, restitution_coef, idx);
	}
}

void World::addGroundContact(imp_float restitution_coef,
							 imp_int particle_idx)
{
	Plane ground(Point::origin(), Vector::unitY());
	
	imp_uint start_idx;
	imp_uint end_idx;

	if (particle_idx < 0)
	{
		start_idx = 0;
		end_idx = getNumberOfParticles();
	}
	else
	{
		start_idx = static_cast<imp_uint>(particle_idx);
		end_idx = start_idx + 1;
		assert(start_idx < getNumberOfParticles());
	}

	for (imp_uint idx = start_idx; idx < end_idx; idx++)
	{
		addPlaneContact(ground, restitution_coef, idx);
	}
}

} // Rendering3D
} // Impact
