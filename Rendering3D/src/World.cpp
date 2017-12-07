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
	  _physics_world(new ParticleWorld()),
	  _renderer(new Renderer(_image, _camera,
							 &_point_lights, &_directional_lights, &_area_lights, &_models)),
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
	
	clearLights();
	clearModels();
	clearMeshes();
	clearMaterials();
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
	
void World::addLight(OmnidirectionalLight* light)
{
	_point_lights.push_back(light);
}
	
void World::addLight(DirectionalLight* light)
{
	_directional_lights.push_back(light);
}
	
void World::addLight(AreaLight* light)
{
	_area_lights.push_back(light);
}

void World::addMesh(TriangleMesh* mesh)
{
	_meshes.push_back(mesh);
}

void World::addModel(Model* model)
{
	_models.push_back(model);
}

void World::addMaterial(Material* material)
{
	_materials.push_back(material);
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

void World::addParticleModel(ParticleModel* particle_model)
{
	addModel(particle_model);
	addParticle(particle_model->getParticle());
}

void World::addParticleForce(ParticleForceGenerator* force_generator, Particle* particle)
{
	_physics_world->addForceGenerator(particle, force_generator);
}

void World::clearLights()
{
	for (std::vector<OmnidirectionalLight*>::iterator iter = _point_lights.begin(); iter != _point_lights.end(); iter++)
	{
		if (*iter)
			delete *iter;
	}

	_point_lights.clear();

	for (std::vector<DirectionalLight*>::iterator iter = _directional_lights.begin(); iter != _directional_lights.end(); iter++)
	{
		if (*iter)
			delete *iter;
	}

	_directional_lights.clear();
	
	for (std::vector<AreaLight*>::iterator iter = _area_lights.begin(); iter != _area_lights.end(); iter++)
	{
		if (*iter)
			delete *iter;
	}

	_area_lights.clear();
}

void World::clearMeshes()
{
	for (std::vector<TriangleMesh*>::iterator iter = _meshes.begin(); iter != _meshes.end(); iter++)
	{
		if (*iter)
			delete *iter;
	}

	_meshes.clear();
}

void World::clearModels()
{
	for (std::vector<Model*>::iterator iter = _models.begin(); iter != _models.end(); iter++)
	{
		if (*iter)
			delete *iter;
	}

	_models.clear();
}

void World::clearMaterials()
{
	for (std::vector<Material*>::iterator iter = _materials.begin(); iter != _materials.end(); iter++)
	{
		if (*iter)
			delete *iter;
	}

	_materials.clear();
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

OmnidirectionalLight* World::getPointLight(imp_uint idx)
{
	assert(idx < static_cast<imp_uint>(_point_lights.size()));
	return _point_lights[idx];
}

DirectionalLight* World::getDirectionalLight(imp_uint idx)
{
	assert(idx < static_cast<imp_uint>(_directional_lights.size()));
	return _directional_lights[idx];
}

AreaLight* World::getAreaLight(imp_uint idx)
{
	assert(idx < static_cast<imp_uint>(_area_lights.size()));
	return _area_lights[idx];
}

Geometry3D::TriangleMesh* World::getMesh(imp_uint idx)
{
	assert(idx < static_cast<imp_uint>(_meshes.size()));
	return _meshes[idx];
}

Model* World::getModel(imp_uint idx)
{
	assert(idx < static_cast<imp_uint>(_models.size()));
	return _models[idx];
}

Material* World::getMaterial(imp_uint idx)
{
	assert(idx < static_cast<imp_uint>(_materials.size()));
	return _materials[idx];
}

Physics3D::Particle* World::getParticle(imp_uint idx)
{
	assert(idx < static_cast<imp_uint>(_particles.size()));
	return _particles[idx];
}

imp_uint World::getNumberOfParticles() const
{
	return static_cast<imp_uint>(_particles.size());
}

imp_uint World::getNumberOfForceGenerators() const
{
	return static_cast<imp_uint>(_force_generators.size());
}

void World::setTimeTrigger(void (*performTimeTriggeredEvent)(World*), imp_float trigger_time)
{
	_performTimeTriggeredEvent = performTimeTriggeredEvent;
	_trigger_time = trigger_time;
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
	_physics_world->use_omp = use_omp;

	_renderer->initialize();
	_animator->initialize(_image->getWidth(), _image->getHeight());
	_physics_world->initialize();
}

void World::performPerFrameInitialization()
{
	if (_performTimeTriggeredEvent && _animator->getElapsedSimulationTime() > _trigger_time)
	{
		_performTimeTriggeredEvent(this);
		_performTimeTriggeredEvent = nullptr;
	}
}

void World::render()
{
	performPerFrameInitialization();

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
		_renderer->toggleModelShadows();
		break;
	case '0':
		toggleFresnel();
		break;
	case 'c':
		_animator->toggleInteractiveCamera();
		break;
	case 'r':
		_animator->toggleRealtimeSimulation();
		break;
	case '8':
		_animator->decreasePathTracingSamples();
		break;
	case '9':
		_animator->increasePathTracingSamples();
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
	case ' ':
		_animator->saveSnapshot();
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
	if (state == GLUT_DOWN)
	{
		_renderer->pixel_was_picked = true;
		_renderer->picked_x = static_cast<imp_uint>(x);
		_renderer->picked_y = static_cast<imp_uint>(y);
	}
}

void World::toggleParallelization()
{
	use_omp = !use_omp;

	_image->use_omp = use_omp;
	_renderer->use_omp = use_omp;
	_physics_world->use_omp = use_omp;
}

void World::toggleFresnel()
{
	for (std::vector<Material*>::iterator iter = _materials.begin(); iter != _materials.end(); iter++)
	{
		(*iter)->use_fresnel = !((*iter)->use_fresnel);
	}
}

void World::addRoom(imp_float width, imp_float height, imp_float depth, const Material* material)
{
	Model* model = new Model(&Geometry3D::IMP_ROOM_MESH,
							 material,
							 LinearTransformation::scaling(width, height, depth));
	addModel(model);
}

void World::addGround(imp_float width, imp_float depth, const Material* material)
{
	Model* model = new Model(&Geometry3D::IMP_SHEET_MESH,
							 material,
							 LinearTransformation::scaling(width, 1, depth));
	addModel(model);
}

void World::addBox(const Box& box, const Material* material)
{
	Model* model = new Model(&Geometry3D::IMP_BOX_MESH,
						     material,
						     AffineTransformation::translationTo(box.getCenter())(
						     LinearTransformation::vectorsToVectors(Vector::unitX(), Vector::unitY(), Vector::unitZ(),
									  							    box.getWidthVector(), box.getHeightVector(), box.getDepthVector())));
	addModel(model);
}

void World::addSphere(const Sphere& sphere,
					  const Material* material,
					  imp_uint quality /* = 0 */)
{
	assert(quality < IMP_N_SPHERE_MESHES);

	Model* model = new Model(Geometry3D::IMP_SPHERE_MESHES + quality,
						     material,
						     AffineTransformation::translationTo(sphere.center)(
						     LinearTransformation::scaling(sphere.radius)));
	addModel(model);
}

void World::addTwoSidedSphere(const Sphere& sphere,
							  const Material* material,
							  imp_uint quality /* = 0 */)
{
	assert(quality < IMP_N_SPHERE_MESHES);

	Model* model = new Model(Geometry3D::IMP_TWOSIDED_SPHERE_MESHES + quality,
						     material,
						     AffineTransformation::translationTo(sphere.center)(
						     LinearTransformation::scaling(sphere.radius)));
	addModel(model);
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

	ParticleModel* model = new ParticleModel(Geometry3D::IMP_SPHERE_MESHES + quality,
											 material,
											 new Particle(position,
										 				  velocity,
														  acceleration,
														  mass,
														  radius,
														  damping),
											 LinearTransformation::scaling(radius));

	addParticleModel(model);
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

	addParticleForceGenerator(force_generator);
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
	
	addParticleForceGenerator(force_generator);
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
	
	addParticleForceGenerator(force_generator);
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

	ParticleContactGenerator* contact = new Physics3D::ParticlePlaneContact(_particles[particle_idx],
																		    plane,
																			restitution_coef);

	addParticleContact(contact);
}

void World::addCableContact(imp_float cable_length,
							imp_float restitution_coef,
							imp_uint particle_idx_1,
							imp_uint particle_idx_2)
{
	assert(particle_idx_1 < getNumberOfParticles() && particle_idx_2 < getNumberOfParticles());

	ParticleContactGenerator* contact = new Physics3D::ParticleCableContact(_particles[particle_idx_1],
																		    _particles[particle_idx_2],
																		    cable_length,
																		    restitution_coef);

	addParticleContact(contact);
}

void World::addRodContact(imp_float rod_length,
						  imp_uint particle_idx_1,
						  imp_uint particle_idx_2)
{
	assert(particle_idx_1 < getNumberOfParticles() && particle_idx_2 < getNumberOfParticles());

	ParticleContactGenerator* contact = new Physics3D::ParticleRodContact(_particles[particle_idx_1],
																		  _particles[particle_idx_2],
																		  rod_length);

	addParticleContact(contact);
}

void World::addParticleContact(imp_float restitution_coef,
							   imp_uint particle_idx_1,
							   imp_uint particle_idx_2)
{
	assert(particle_idx_1 < getNumberOfParticles() && particle_idx_2 < getNumberOfParticles());

	ParticleContactGenerator* contact = new Physics3D::ParticleParticleContact(_particles[particle_idx_1],
																			   _particles[particle_idx_2],
																			   restitution_coef);

	addParticleContact(contact);
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

void World::removeForceGenerator(imp_uint generator_idx, imp_uint particle_idx)
{
	assert(particle_idx < getNumberOfParticles() && generator_idx < getNumberOfForceGenerators());
	_physics_world->removeForceGenerator(_particles[particle_idx], _force_generators[generator_idx]);
}

void World::clearAllForces()
{
	_physics_world->clearForceGenerators();
}

void World::removeAllGravityForces(imp_uint generator_start_idx)
{
	imp_uint i, j, n;
	imp_uint n_particles = getNumberOfParticles();

	n = 0;

	for (i = 0; i < n_particles-1; i++)
	{
		for (j = i+1; j < n_particles; j++)
		{
			removeForceGenerator(generator_start_idx + n, i);
			removeForceGenerator(generator_start_idx + n, j);

			n++;
		}
	}
}

} // Rendering3D
} // Impact
