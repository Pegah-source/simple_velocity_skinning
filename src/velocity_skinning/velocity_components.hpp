#pragma once

#include "cgp/cgp.hpp"
#include "skeleton/skeleton.hpp"
#include "skinning/skinning.hpp"


namespace cgp
{

    struct velocity_components{

        buffer<cgp::affine_rt> current_local_skeleton;
        buffer<cgp::affine_rt> last_local_skeleton;
        buffer<cgp::vec3> translational_velocities; // of size N_joint, for each call of compute_deformation in scene
        buffer<cgp::vec3> angular_velocities;

        void update_skeletons(buffer<cgp::affine_rt> skeleton, bool first_step);
    };
    
    struct quaternion_dual{
        cgp::quaternion q;
        cgp::quaternion qe;

        quaternion_dual(quaternion const& q_arg, quaternion const qe_arg);
        quaternion_dual(quaternion const& q_arg, vec3 const& t);
        vec3 translation() const;/*
        quaternion_dual& operator+=(quaternion_dual& a, quaternion_dual const& b);
        quaternion_dual operator+(quaternion_dual const& a, quaternion_dual const& b);
        quaternion_dual operator*(float s, quaternion_dual const& d);
        quaternion_dual operator/(quaternion_dual const& d, float s);*/
    };
    void cal_comp_velocities(velocity_components& v_components);
}