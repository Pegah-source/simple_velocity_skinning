#pragma once

#include "cgp/cgp.hpp"

#include "skeleton/skeleton.hpp"
#include "skeleton/skeleton_drawable.hpp"
#include "skinning/skinning.hpp"
#include "velocity_skinning/velocity_skinning_weights.hpp"
#include "velocity_skinning/velocity_components.hpp"
#include "velocity_skinning/displacement.hpp"




// The element of the GUI that are not already stored in other structures
struct gui_parameters {
	bool display_frame     = false;

	bool surface_skinned = true;
	bool wireframe_skinned = false;
	bool surface_rest_pose = false;
	bool wireframe_rest_pose = false;

	bool skeleton_current_bone = true;
	bool skeleton_current_frame = false;
	bool skeleton_current_sphere = false;

	bool skeleton_rest_pose_bone = false;
	bool skeleton_rest_pose_frame = false;
	bool skeleton_rest_pose_sphere = false;
};


struct visual_shapes_parameters
{
	cgp::mesh_drawable surface_skinned;
	cgp::mesh_drawable surface_rest_pose;
	cgp::skeleton_drawable skeleton_current;
	cgp::skeleton_drawable skeleton_rest_pose;
};

struct skinning_current_data
{
	cgp::buffer<cgp::vec3> position_rest_pose;
	cgp::buffer<cgp::vec3> position_skinned;
	cgp::buffer<cgp::vec3> normal_rest_pose;
	cgp::buffer<cgp::vec3> normal_skinned;

	cgp::buffer<cgp::affine_rt> skeleton_current;
	cgp::buffer<cgp::affine_rt> skeleton_rest_pose;
};

// The structure of the custom scene
struct scene_structure {
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	
	/*
	* velocity related variables:
	*/
	bool starting_moment;
	cgp::displacement disp;

	cgp::timer_interval timer;

	cgp::scene_environment_basic environment; // Standard environment controler
	gui_parameters gui;                       // Standard GUI element storage
	cgp::mesh_drawable global_frame;          // The standard global frame
	

	visual_shapes_parameters visual_data;
	cgp::skeleton_animation_structure skeleton_data;
	cgp::rig_structure rig;
	skinning_current_data skinning_data;

	// ****************************** //
	// Functions
	// ****************************** //

	void initialize();  // Standard initialization to be called before the animation loop
	void display();     // The frame display to be called within the animation loop
	void display_gui(); // The display of the GUI, also called within the animation loop

	void compute_deformation();
	void update_new_content(cgp::mesh const& shape, GLuint texture_id);
};





