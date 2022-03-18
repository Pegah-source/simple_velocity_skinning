#include "velocity_components.hpp"

namespace cgp
{
    // definition of implicitly declared default constructor
    /*quaternion_dual::quaternion_dual()
        :q(), qe()
    {}*/
    quaternion_dual::quaternion_dual(quaternion const& q_arg, quaternion const qe_arg)
        :q(q_arg),qe(qe_arg)
    {}

    quaternion_dual::quaternion_dual(quaternion const& q_arg, vec3 const& t)
        :q(q_arg), qe()
    {
        qe = 0.5f * quaternion(t.x, t.y, t.z, 0.0f) * q_arg;
    //    qe.x = 0.5f * ( t.x*q.w + t.y*q.z - t.z*q.y);
    //    qe.y = 0.5f * (-t.x*q.z + t.y*q.w + t.z*q.x);
    //    qe.z = 0.5f * ( t.x*q.y - t.y*q.x + t.z*q.w);
    //    qe.w = 0.5f * (-t.x*q.x - t.y*q.y - t.z*q.z);
    }

    vec3 quaternion_dual::translation() const
    {
        quaternion q_t = 2 * qe * conjugate(q);
        return {q_t.x, q_t.y, q_t.z};

    //    float const tx = 2.0f * ( -qe.w*q.x + qe.x*q.w - qe.y*q.z + qe.z*q.y );
    //    float const ty = 2.0f * ( -qe.w*q.y + qe.x*q.z + qe.y*q.w - qe.z*q.x );
    //    float const tz = 2.0f * ( -qe.w*q.z - qe.x*q.y + qe.y*q.x + qe.z*q.w );
    //    return {tx,ty,tz};
    }

    quaternion_dual& operator+=(quaternion_dual& a, quaternion_dual const& b)
    {
        a.q += b.q;
        a.qe += b.qe;
        return a;
    }
    quaternion_dual operator+(quaternion_dual const& a, quaternion_dual const& b)
    {
        return {a.q+b.q, a.qe+b.qe};
    }
    quaternion_dual operator*(float s, quaternion_dual const& d)
    {
        return {s*d.q, s*d.qe};
    }
    quaternion_dual operator/(quaternion_dual const& d, float s)
    {
        return {d.q/s, d.qe/s};
    }

    void velocity_components::update_skeletons(buffer<cgp::affine_rt> skeleton, bool first_step){
        //std::cout << "skeleton updated" << std::endl;
        if(first_step){
            current_local_skeleton = skeleton;
        }else{
            last_local_skeleton = current_local_skeleton;
            current_local_skeleton = skeleton;
        }
    }
    void cal_comp_velocities(velocity_components& v_components){

        buffer<cgp::affine_rt> current_local_skeleton = v_components.current_local_skeleton;
        buffer<cgp::affine_rt> last_local_skeleton = v_components.last_local_skeleton;
        buffer<cgp::vec3>& translational_velocities = v_components.translational_velocities; // of size N_joint, for each call of compute_deformation in scene
        buffer<cgp::vec3>& angular_velocities = v_components.angular_velocities;

        int N_joint = current_local_skeleton.size();
        //buffer<vec3> translational_velocities = &v_components.translational_velocities;
        //buffer<vec3> angular_velocities = &v_components.angular_velocities;
        translational_velocities.resize(N_joint);
        angular_velocities.resize(N_joint);
        current_local_skeleton = v_components.current_local_skeleton;
        last_local_skeleton = v_components.last_local_skeleton;

        /*buffer<cgp::quaternion_dual> dq;
		dq.resize(N_joint);*/

        // the velocity should be calculated for each joint
        for(int j = 0; j < N_joint; j++){
            // last_local_skeleton and current_local_skeleton
            // translation from the last frame:
            
            rotation_transform const& r  = current_local_skeleton[j].rotation;
			rotation_transform const& r0 = last_local_skeleton[j].rotation;
			vec3 const& t = current_local_skeleton[j].translation;
			vec3 const& t0 = last_local_skeleton[j].translation;
			quaternion_dual dq( (r*inverse(r0)).data, r*inverse(r0)*(-t0)+t );
            translational_velocities[j] = t - t0;

            // to extract the rotational velocity
            rotation_transform rot = rotation_transform(dq.q); 

            /*quaternion Q1 = current_local_skeleton[j].rotation_transform.data;
            quaternion Q2 = last_local_skeleton[j].rotation_transform.data;

            quaternion Qprime = current_local_skeleton * inverse(last_local_skeleton);
            rotation_transform rot = from_quaternion(Qprime);*/

            vec3 axis; float angle;
            rot.to_axis_angle(axis,  angle);
            

            angular_velocities[j] = axis*angle;
        }
    }

}