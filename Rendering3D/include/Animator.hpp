#pragma once
#include "precision.hpp"
#include "ParticleWorld.hpp"
#include "TriangleMesh.hpp"
#include "Image.hpp"
#include "Renderer.hpp"
#include "Vector3.hpp"
#include "Ray.hpp"
#include "Box.hpp"
#include "CoordinateFrame.hpp"
#include "AffineTransformation.hpp"
#include <vector>
#include <map>
#include <cassert>
#include <ctime>

namespace Impact {
namespace Rendering3D {

class Animator {

private:
	typedef Physics3D::Particle Particle;
	typedef Physics3D::ParticleWorld ParticleWorld;
	typedef Geometry3D::Vector Vector;
	typedef Geometry3D::Ray Ray;
	typedef Geometry3D::Box Box;
	typedef Geometry3D::CoordinateFrame CoordinateFrame;
	typedef Geometry3D::TriangleMesh TriangleMesh;
	typedef Geometry3D::AffineTransformation AffineTransformation;

	Renderer* _renderer;
	ParticleWorld* _physics_world;

	static const imp_uint _fps_running_average_size = 3;
	const imp_float _fps_start_value = 0.03f;
	const imp_float _video_frame_duration = 1.0f/30.0f;
	
	const imp_float _simulation_timestep_increment_ratio = 1.1f;
	const imp_float _simulation_frequency_increment_ratio = 1.1f;
	const imp_float _simulation_weight_increment_ratio = 1.1f;

	const imp_uint _default_simulation_frequency = 100;

	const imp_float _minimum_simulation_timestep = 1e-6f;

	int _rendering_mode = 0;
	bool _physics_active = false;
	bool _camera_active = false;
	bool _simulate_realtime = true;
	bool _print_time_info = false;
	bool _auto_simulation_frequency = true;
	bool _recording_active = false;
	bool _singlestepping_active = false;
	bool _single_step_requested = false;
	
	imp_float _fixed_simulation_timestep = 1.0e-5f;

	imp_uint _simulation_frequency = _default_simulation_frequency;
	imp_float _optimal_simulation_to_rendering_ratio = 0.1f;
	imp_float _auto_simulation_frequency_response = 400.0f;
	
	imp_float _recording_playback_speed = 1.0f;

	imp_float _camera_movement_speed = 5.0f;
	imp_float _camera_rotation_speed = 0.001f;

	imp_uint _n_path_tracing_samples = 1;

	std::map<unsigned char, bool> _keys_pressed;
	bool _camera_moved = false;
	int _image_center_x, _image_center_y;
	AffineTransformation _camera_look_ray_transformation;

	imp_uint _frame_count;
	imp_uint _saved_frames_count;
	std::chrono::steady_clock::time_point _previous_time;
	imp_float _previous_frame_durations[_fps_running_average_size];
	imp_float _elapsed_simulation_time = 0;

	void estimateFrameDuration(imp_float& last_duration, imp_float& running_average);

	void saveFrame();

public:
	typedef unsigned char frame_buffer_data;

	Animator(Renderer* new_renderer,
		     ParticleWorld* new_physics_world);

    Animator(const Animator& other) = delete;
	Animator& operator=(const Animator& other) = delete;

	void initialize(imp_uint image_width, imp_uint image_height);

	void updateFrame();

	void startCameraMove(unsigned char key);
	void stopCameraMove(unsigned char key);
	void moveCamera(imp_float frame_duration);
	void rotateCamera(int x, int y);

	void togglePhysics();
	void cycleRenderingMode();
	void toggleInteractiveCamera();
	void toggleRealtimeSimulation();
	void increasePathTracingSamples();
	void decreasePathTracingSamples();
	void increaseFixedSimulationTimestep();
	void decreaseFixedSimulationTimestep();
	void increaseSimulationFrequency();
	void decreaseSimulationFrequency();
	void toggleAutomaticSimulationTimestep();
	void increaseSimulationPriority();
	void decreaseSimulationPriority();
	void toggleInfoPrinting();
	void cycleDepthMapRendering();
	void toggleGammaEncoding();
	void toggleEdgeRendering();
	void toggleRecording();
	void toggleSinglestepping();
	void performSingleStep();
	void terminate();
	
	void saveSnapshot();

	imp_float getElapsedSimulationTime() const;
};

} // Rendering3D
} // Impact
