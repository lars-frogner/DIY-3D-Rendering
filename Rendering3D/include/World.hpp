#pragma once
#include "precision.hpp"
#include "Material.hpp"
#include "Light.hpp"
#include "Image.hpp"
#include "Camera.hpp"
#include "Renderer.hpp"
#include "Animator.hpp"
#include "Model.hpp"
#include "ParticleModel.hpp"
#include "Point3.hpp"
#include "Vector3.hpp"
#include "Box.hpp"
#include "Plane.hpp"
#include "Sphere.hpp"
#include "TriangleMesh.hpp"
#include "LinearTransformation.hpp"
#include "AffineTransformation.hpp"
#include "Particle.hpp"
#include "ParticleWorld.hpp"
#include "ParticleForceGenerator.hpp"
#include "ParticleContactGenerator.hpp"
#include <vector>
#include <memory>

namespace Impact {
namespace Rendering3D {

class World {
    
private:
	typedef Geometry3D::Point Point;
	typedef Geometry3D::Vector Vector;
	typedef Geometry3D::Box Box;
	typedef Geometry3D::Plane Plane;
	typedef Geometry3D::Sphere Sphere;
	typedef Geometry3D::TriangleMesh TriangleMesh;
	typedef Geometry3D::LinearTransformation LinearTransformation;
	typedef Geometry3D::AffineTransformation AffineTransformation;
	typedef Geometry3D::Camera Camera;
	typedef Physics3D::Particle Particle;
	typedef Physics3D::ParticleWorld ParticleWorld;
	typedef Physics3D::ParticleForceGenerator ParticleForceGenerator;
	typedef Physics3D::ParticleContactGenerator ParticleContactGenerator;

protected:

	// Rendering assets
	std::vector<Light*> _lights;
	std::vector<Model*> _models;
	std::vector<Material*> _materials;
	
	// Physics assets
	std::vector<Particle*> _particles;
	std::vector<ParticleForceGenerator*> _force_generators;
	std::vector<ParticleContactGenerator*> _contact_generators;
	
	Image* _image;
	Camera* _camera;
	
	ParticleWorld* _physics_world;
	Renderer* _renderer;
	Animator* _animator;

	void (*_performTimeTriggeredEvent)(World*) = nullptr;
	imp_float _trigger_time;

public:
	bool use_omp = true;

	World(imp_uint image_width,
		  imp_uint image_height);
	~World();

	World(const World& other) = delete;
	World& operator=(const World& other) = delete;

	void setCameraPointing(const Point& position,
						   const Vector& look_direction = -Vector::unitZ(),
						   const Vector& up_direction = Vector::unitY());
	
	void addLight(Light* light);
	void addModel(Model* model);
	void addMaterial(Material* material);

	void addParticle(Particle* particle);
	void addParticleForceGenerator(ParticleForceGenerator* force_generator);
	void addParticleContact(ParticleContactGenerator* contact_generator);

	void addParticleModel(ParticleModel* particle_model);
	void addParticleForce(ParticleForceGenerator* force_generator, Particle* particle);

	void clearLights();
	void clearModels();
	void clearMaterials();

	void clearParticles();
	void clearParticleForces();
	void clearParticleContacts();
	
	Light* getLight(imp_uint idx);
	Model* getModel(imp_uint idx);
	Material* getMaterial(imp_uint idx);
	
	Particle* getParticle(imp_uint idx);

	imp_uint getNumberOfParticles() const;
	imp_uint getNumberOfForceGenerators() const;

	void setTimeTrigger(void (*performTimeTriggeredEvent)(World*), imp_float trigger_time);

	void initialize();
	void performPerFrameInitialization();
	void render();

	void processKeyPress(unsigned char key, int x, int y);
	void processKeyRelease(unsigned char key, int x, int y);
	void processMouseMovement(int x, int y);
	void processMouseClick(int button, int state, int x, int y);
	
	void toggleParallelization();

	void addRoom(imp_float width, imp_float height, imp_float depth, const Material* material);
	void addGround(imp_float width, imp_float depth, const Material* material);
	void addBox(const Box& box, const Material* material);
	void addSphere(const Sphere& sphere, const Material* material, imp_uint quality = 0);

	void addParticle(const Point& position,
					 const Vector& velocity,
					 const Vector& acceleration,
					 imp_float mass,
					 imp_float radius,
					 imp_float damping,
					 const Material* material,
					 imp_uint quality = 0);

	void addDragForce(imp_float coef_1,
					  imp_float coef_2,
					  imp_int particle_idx);
	
	void addUniformGravityForce(const Vector& gravity,
								imp_int particle_idx);

	void addGravityForce(imp_float gravitational_constant,
						 imp_uint particle_idx_1,
						 imp_uint particle_idx_2);

	void addAllGravityForces(imp_float gravitational_constant);

	void addSpringForce(imp_float spring_constant,
						imp_float rest_length,
						imp_uint particle_idx_1,
						imp_uint particle_idx_2);

	void addBungeeForce(imp_float spring_constant,
						imp_float rest_length,
						imp_uint particle_idx_1,
						imp_uint particle_idx_2);

	void addAnchoredSpringForce(const Point& anchor_point,
								imp_float spring_constant,
								imp_float rest_length,
								imp_int particle_idx);

	void addAnchoredBungeeForce(const Point& anchor_point,
								imp_float spring_constant,
								imp_float rest_length,
								imp_int particle_idx);

	void addAnchoredStiffSpringForce(const Point& anchor_point,
									 imp_float spring_constant,
									 imp_float damping,
									 imp_int particle_idx);

	void addPlaneContact(const Plane& plane,
						 imp_float restitution_coef,
						 imp_uint particle_idx);

	void addCableContact(imp_float cable_length,
						 imp_float restitution_coef,
						 imp_uint particle_idx_1,
						 imp_uint particle_idx_2);

	void addRodContact(imp_float rod_length,
					   imp_uint particle_idx_1,
					   imp_uint particle_idx_2);

	void addParticleContact(imp_float restitution_coef,
							imp_uint particle_idx_1,
							imp_uint particle_idx_2);

	void addParticleContacts(imp_float restitution_coef,
							 imp_uint particle_idx);

	void addAllParticleContacts(imp_float restitution_coef);

	void addRoomContact(imp_float width,
						imp_float height,
						imp_float depth,
						imp_float restitution_coef,
						imp_int particle_idx);

	void addGroundContact(imp_float restitution_coef,
						  imp_int particle_idx);

	void removeForceGenerator(imp_uint generator_idx, imp_uint particle_idx);
	
	void clearAllForces();

	void removeAllGravityForces(imp_uint generator_start_idx);
};

} // Rendering3D
} // Impact
