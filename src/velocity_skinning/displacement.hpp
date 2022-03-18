#pragma once

#include "cgp/cgp.hpp"
#include "velocity_skinning/velocity_skinning_weights.hpp"
#include "velocity_skinning/velocity_components.hpp"


namespace cgp
{
	// Helper structure storing an animated skeleton
	struct displacement
	{
        cgp::velocity_weights v_weights;
        buffer<buffer<float>> v_skinning_weights;
        cgp::velocity_components v_components;
        float k_squash;
        float k_floppy;
	};
        void initialize_disp(displacement& displacement, buffer<int> parent_index, cgp::rig_structure rig);
        void adding_velocity_def(displacement& displacement, buffer<cgp::affine_rt> skeleton, bool first_step, buffer<cgp::vec3>& vertex_positions, buffer<cgp::vec3> joint_positions);

}