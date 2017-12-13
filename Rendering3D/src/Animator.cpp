#include "Animator.hpp"
#include <glut.h>
#include <freeglut.h>
#include <cassert>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>

namespace Impact {
namespace Rendering3D {

Animator::Animator(Renderer* new_renderer,
				   ParticleWorld* new_physics_world)
	: _renderer(new_renderer),
	  _physics_world(new_physics_world) {}

void Animator::estimateFrameDuration(imp_float& last_duration, imp_float& running_average)
{	
	std::chrono::steady_clock::time_point current_time = std::chrono::high_resolution_clock::now();

	last_duration = std::chrono::duration_cast< std::chrono::duration<imp_float> >(current_time - _previous_time).count();

	_previous_frame_durations[_frame_count % _fps_running_average_size] = last_duration;
	_previous_time = current_time;

	running_average = 0;
	
	for (imp_uint i = 0; i < _fps_running_average_size; i++)
		running_average += _previous_frame_durations[i];

	running_average /= _fps_running_average_size;
}

void Animator::initialize(imp_uint image_width, imp_uint image_height)
{
	_keys_pressed['w'] = false;
	_keys_pressed['a'] = false;
	_keys_pressed['s'] = false;
	_keys_pressed['d'] = false;
	_keys_pressed['q'] = false;
	_keys_pressed['e'] = false;

	_image_center_x = static_cast<int>(image_width)/2;
	_image_center_y = static_cast<int>(image_height)/2;

	if (_camera_active)
		glutSetCursor(GLUT_CURSOR_NONE);

	_frame_count = 0;
	_previous_time = std::chrono::high_resolution_clock::now();

	for (imp_uint i = 0; i < _fps_running_average_size; i++)
		_previous_frame_durations[i] = _fps_start_value;

	_saved_frames_count = 0;
}

void Animator::updateFrame()
{
	imp_float simulation_duration;
	imp_float simulation_timestep;
	imp_float residual_simulation_duration;

	imp_float current_realtime_frame_duration_estimate;
	imp_float measured_previous_realtime_frame_duration;

	std::chrono::time_point<std::chrono::steady_clock> start_time, end_time;

	bool valid_recording_mode = _recording_active && !_simulate_realtime && !_camera_active && _physics_active;
	bool valid_singlestepping_mode = _singlestepping_active && _physics_active && !valid_recording_mode && !_simulate_realtime && !_auto_simulation_frequency;
	
	estimateFrameDuration(measured_previous_realtime_frame_duration, current_realtime_frame_duration_estimate);
	
	if (valid_recording_mode)
	{
		simulation_timestep = _fixed_simulation_timestep;
		simulation_duration = _recording_playback_speed*_video_frame_duration;
		_simulation_frequency = static_cast<imp_uint>(simulation_duration/simulation_timestep);
		residual_simulation_duration = simulation_duration - _simulation_frequency*simulation_timestep;
		
		if (residual_simulation_duration < _minimum_simulation_timestep)
		{
			simulation_duration -= residual_simulation_duration;
			residual_simulation_duration = 0;
		}
	}
	else if (_simulate_realtime)
	{
		simulation_duration = current_realtime_frame_duration_estimate;
		simulation_timestep = simulation_duration/_simulation_frequency;

		if (simulation_timestep < _minimum_simulation_timestep)
		{
			simulation_timestep = _minimum_simulation_timestep;
			_simulation_frequency = static_cast<imp_uint>(simulation_duration/simulation_timestep);
		}
	}
	else if (valid_singlestepping_mode)
	{
		simulation_timestep = _fixed_simulation_timestep;
		simulation_duration = simulation_timestep;
	}
	else
	{
		simulation_timestep = _fixed_simulation_timestep;
		simulation_duration = _simulation_frequency*simulation_timestep;
	}

	if (_physics_active && !(valid_singlestepping_mode && !_single_step_requested))
	{
		start_time = std::chrono::high_resolution_clock::now();

		for (imp_uint i = 0; i < _simulation_frequency; i++)
		{
			_physics_world->performPerFrameInitialization();
			_physics_world->advance(simulation_timestep);
		}

		end_time = std::chrono::high_resolution_clock::now();

		imp_float measured_realtime_simulation_duration = std::chrono::duration_cast< std::chrono::duration<imp_float> >(end_time - start_time).count();

		if (valid_recording_mode)
		{
			if (residual_simulation_duration > 0)
			{
				_physics_world->performPerFrameInitialization();
				_physics_world->advance(residual_simulation_duration);
			}
		}
		else if (_auto_simulation_frequency)
		{
			imp_int adjustment = static_cast<imp_int>(_optimal_simulation_to_rendering_ratio
												  	  *(current_realtime_frame_duration_estimate
													  /measured_realtime_simulation_duration
													  - 1)
													  *_auto_simulation_frequency_response);

			if (adjustment > 1 - static_cast<imp_int>(_simulation_frequency))
				_simulation_frequency = static_cast<imp_uint>(static_cast<imp_int>(_simulation_frequency) + adjustment);
			else
				_simulation_frequency = 1;
		}
		else if (valid_singlestepping_mode)
		{
			_single_step_requested = false;
		}

		_physics_world->updateTransformations();

		_elapsed_simulation_time += simulation_duration;
	}

	if (_camera_active)
	{
		moveCamera(current_realtime_frame_duration_estimate);

		if (_camera_moved)
		{
			_renderer->transformCameraLookRay(_camera_look_ray_transformation);
			_camera_look_ray_transformation.setToIdentity();
			glutWarpPointer(_image_center_x, _image_center_y);
			_camera_moved = false;
		}
	}

	start_time = std::chrono::high_resolution_clock::now();

	if (_rendering_mode == 0)
	{
		_renderer->renderDirect();
	}
	else if (_rendering_mode == 1)
	{
		_renderer->rayTrace();
	}
	else if (_rendering_mode == 2)
	{
		_renderer->rasterize();
	}
	else if (_rendering_mode == 3)
	{
		_renderer->pathTrace(_n_path_tracing_samples);
	}
	else
	{
		std::cerr << "Warning: rendering disabled" << std::endl;
	}

	end_time = std::chrono::high_resolution_clock::now();

	imp_float measured_rendering_duration = std::chrono::duration_cast< std::chrono::duration<imp_float> >(end_time - start_time).count();

	if (valid_recording_mode)
	{
		saveFrame();
	}

	if (_print_time_info)
	{
		printf("%d%d%s%s%s%s%s%s%s%s%s%s | fps: %.1f | duration: %.2g s | dt: %.2g s | freq: %d | time: %.2g s\n",
			    _rendering_mode,
				_n_path_tracing_samples,
				(_renderer->gamma_encode)? "g" : "",
				(_renderer->render_depth_map)? "z" : "",
				(_renderer->draw_edges)? "l" : "",
				(_physics_active)? "p" : "",
				(_simulate_realtime)? "r" : "",
				(_auto_simulation_frequency)? "b" : "",
				(_singlestepping_active)? "x" : "",
				(_camera_active)? "c" : "",
				(_recording_active)? "t" : "",
				(_renderer->use_omp)? "y" : "",
				1/current_realtime_frame_duration_estimate,
			    measured_rendering_duration,
				simulation_timestep,
				_simulation_frequency,
				_elapsed_simulation_time);
	}
}

void Animator::startCameraMove(unsigned char key)
{
	if (key == 'w' || key == 'a' || key == 's' || key == 'd' || key == 'q' || key == 'e')
	{
		_keys_pressed[key] = true;
	}
}

void Animator::stopCameraMove(unsigned char key)
{
	if (key == 'w' || key == 'a' || key == 's' || key == 'd' || key == 'q' || key == 'e')
	{
		_keys_pressed[key] = false;
	}
}

void Animator::moveCamera(imp_float frame_duration)
{
	imp_float du = 0.0, dv = 0.0, dw = 0.0;
	imp_float camera_movement = _camera_movement_speed*frame_duration;

	if (_keys_pressed['a'])
		du += camera_movement;
	if (_keys_pressed['d'])
		du -= camera_movement;
	if (_keys_pressed['q'])
		dv += camera_movement;
	if (_keys_pressed['e'])
		dv -= camera_movement;
	if (_keys_pressed['w'])
		dw += camera_movement;
	if (_keys_pressed['s'])
		dw -= camera_movement;

	if (du != 0 || dv != 0 || dw != 0)
	{
		const CoordinateFrame& cframe = _renderer->getCamera().getCoordinateFrame();

		_camera_look_ray_transformation = AffineTransformation::translation(-du*cframe.basis_1 + dv*cframe.basis_2 - dw*cframe.basis_3)(
										  _camera_look_ray_transformation);

		_camera_moved = true;
	}
}

void Animator::rotateCamera(int x, int y)
{
	if (!_camera_active)
		return;

	imp_float dphi = -static_cast<imp_float>(x - _image_center_x)*_camera_rotation_speed;
	imp_float dtheta = -static_cast<imp_float>(y - _image_center_y)*_camera_rotation_speed;

	const CoordinateFrame& cframe = _renderer->getCamera().getCoordinateFrame();

	_camera_look_ray_transformation = AffineTransformation::rotationAboutRay(Ray(cframe.origin, cframe.basis_1), dtheta)(
									  AffineTransformation::rotationAboutRay(Ray(cframe.origin, cframe.basis_2), dphi)(
									  _camera_look_ray_transformation));
	
	_camera_moved = true;
}

void Animator::togglePhysics()
{
	_physics_active = !_physics_active;
}

void Animator::cycleRenderingMode()
{
	_rendering_mode = (_rendering_mode + 1) % 4;
}

void Animator::toggleInteractiveCamera()
{
	_camera_active = !_camera_active;

	if (_camera_active)
		glutSetCursor(GLUT_CURSOR_NONE);
	else
		glutSetCursor(GLUT_CURSOR_INHERIT);
}

void Animator::toggleRealtimeSimulation()
{
	_simulate_realtime = !_simulate_realtime;
}

void Animator::increasePathTracingSamples()
{
	_n_path_tracing_samples += 10;
}

void Animator::decreasePathTracingSamples()
{
	if (_n_path_tracing_samples > 10)
		_n_path_tracing_samples -= 10;
}

void Animator::increaseFixedSimulationTimestep()
{
	_fixed_simulation_timestep *= _simulation_timestep_increment_ratio;
}

void Animator::decreaseFixedSimulationTimestep()
{
	_fixed_simulation_timestep /= _simulation_timestep_increment_ratio;
}

void Animator::increaseSimulationFrequency()
{
	if (static_cast<imp_uint>(_simulation_frequency*_simulation_frequency_increment_ratio) > _simulation_frequency)
		_simulation_frequency = static_cast<imp_uint>(_simulation_frequency*_simulation_frequency_increment_ratio);
	else
		_simulation_frequency++;
}

void Animator::decreaseSimulationFrequency()
{
	if (static_cast<imp_uint>(_simulation_frequency/_simulation_frequency_increment_ratio) < _simulation_frequency)
		_simulation_frequency = static_cast<imp_uint>(_simulation_frequency/_simulation_frequency_increment_ratio);
	else if (_simulation_frequency > 1)
		_simulation_frequency--;
}

void Animator::toggleAutomaticSimulationTimestep()
{
	_auto_simulation_frequency = !_auto_simulation_frequency;
}

void Animator::increaseSimulationPriority()
{
	_optimal_simulation_to_rendering_ratio *= _simulation_weight_increment_ratio;
}

void Animator::decreaseSimulationPriority()
{
	_optimal_simulation_to_rendering_ratio /= _simulation_weight_increment_ratio;
}

void Animator::toggleInfoPrinting()
{
	_print_time_info = !_print_time_info;
}

void Animator::cycleDepthMapRendering()
{
	if (!(_renderer->render_depth_map))
	{
		_renderer->render_depth_map = true;
		_renderer->renormalize_depth_map = true;
	}
	else if (_renderer->render_depth_map && _renderer->renormalize_depth_map)
	{
		_renderer->renormalize_depth_map = false;
	}
	else
	{
		_renderer->render_depth_map = false;
	}
}

void Animator::toggleGammaEncoding()
{
	_renderer->gamma_encode = !(_renderer->gamma_encode);
}

void Animator::toggleEdgeRendering()
{
	_renderer->draw_edges = !(_renderer->draw_edges);
}

void Animator::toggleRecording()
{
	_recording_active = !_recording_active;
}

void Animator::toggleSinglestepping()
{
	_singlestepping_active = !_singlestepping_active;

	if (_singlestepping_active)
		_simulation_frequency = 1;
	else
		_simulation_frequency = _default_simulation_frequency;
}

void Animator::performSingleStep()
{
	_single_step_requested = true;
}

void Animator::terminate()
{
	glutLeaveMainLoop();
}

void Animator::saveFrame()
{
	std::ostringstream string_stream;
    string_stream << "data/saved_frames/frame_" << std::setfill('0') << std::setw(4) << _saved_frames_count << ".ppm";
	_renderer->saveImage(string_stream.str());
	_saved_frames_count++;
}

void Animator::saveSnapshot()
{
	time_t t = time(0);
    struct tm now;
	localtime_s(&now, &t);

	std::ostringstream string_stream;
    string_stream << "data/saved_snapshots/snapshot_" << (now.tm_year + 1900) << "Y_"
													  << std::setfill('0') << std::setw(2) << (now.tm_mon + 1) << "M_"
													  << std::setfill('0') << std::setw(2) << now.tm_mday << "D_"
													  << std::setfill('0') << std::setw(2) << now.tm_hour << "h_"
													  << std::setfill('0') << std::setw(2) << now.tm_min << "m_"
													  << std::setfill('0') << std::setw(2) << now.tm_sec << "s.ppm";
	_renderer->saveImage(string_stream.str());
}

imp_float Animator::getElapsedSimulationTime() const
{
	return _elapsed_simulation_time;
}

} // Rendering3D
} // Impact
