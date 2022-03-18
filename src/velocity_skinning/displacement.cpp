#include "displacement.hpp"

namespace cgp
{
    //the two following functions are taken from vcl library
    mat3 rotation_from_axis_angle_mat3(const vec3& axis, float angle) {
        vec3 u = axis;
        if(norm(u) > 1e-3){
            u = normalize(u);
        }
        const float x = u.x;
        const float y = u.y;
        const float z = u.z;
        const float c = std::cos(angle);
        const float s = std::sin(angle);

        return mat3 {c+x*x*(1-c)  , x*y*(1-c)-z*s, x*z*(1-c)+y*s,
                     y*x*(1-c)+z*s, c+y*y*(1-c)  , y*z*(1-c)-x*s,
                     z*x*(1-c)-y*s, z*y*(1-c)+x*s, c+z*z*(1-c)   };
    }

    mat3 rotation_between_vector_mat3(const vec3& a, const vec3& b) {
        vec3 u0 = a;
        vec3 u1 = b;

        if (norm(a) > 1e-3) {
            u0 = a/norm(a);
        }
        if (norm(b) > 1e-3) {
            u1 = b/norm(b);
        }
        

        if( norm(u0-u1)<1e-4f )
            return mat3::identity();
        if( norm(u0+u1)<1e-4f )
            return -1.*mat3::identity();

        const float d = dot(u0,u1);
        const float angle = std::acos( d );
        vec3 axis;
        if(norm(cross(u0,u1))){
            axis = normalize(cross(u0,u1));
        }else{
            axis = cross(u0,u1);
        }
        return rotation_from_axis_angle_mat3(axis,angle);
    }

    vec3 displacement_squashy_linear(vec3 const& position, vec3 const& center,
                                     vec3 const& linear_velocity, float const k_squash) {
        float s = k_squash*norm(linear_velocity);
        mat3 R = rotation_between_vector_mat3({1., 0., 0.}, linear_velocity);
        mat3 S = { 1.f+s, 0., 0.,
                   0., 1.f/sqrt(1.f+s), 0.,
                   0., 0., 1.f/sqrt(1.f+s)};

        return (R*S*transpose(R) - mat3::identity())*(position - center);
    }

    // Update: replacing the medial_axis with joint - position
    // not more medial_axis as argument
    vec3 displacement_squashy_rotation(vec3 const& position, vec3 const& joint,
                                       vec3 const& v_r, vec3 const& angular_velocity,
                                       float const k_squash) {
        float s = k_squash*norm(v_r);
        mat3 S = { 1.f+s, 0., 0.,
                   0., 1.f, 0.,
                   0., 0., 1.f/(1.f+s)};

        vec3 const& medial_axis = position - joint;
        vec3 medial_axis_normalized = medial_axis;

        if(norm(medial_axis) > 1e-3f){
            medial_axis_normalized = normalize(medial_axis);
        }
        vec3 normalized_angular_velocity = angular_velocity;
        if(norm(angular_velocity) > 1e-3f){
            normalized_angular_velocity = normalize(angular_velocity);
        }
        vec3 const r1 = cross({medial_axis_normalized}, normalized_angular_velocity);
        //check if angular velocity is parallel to the medial axis

        if (norm(r1) < 1e-3) {
            return {0., 0., 0.};
         }

        vec3 r1_normalized = r1;
        if(norm(r1) > 1e-3f){
            r1_normalized = normalize(r1);
        }
        vec3 const r2 = cross(medial_axis_normalized, r1_normalized);
        mat3 const R = mat3({r1_normalized, r2, medial_axis_normalized});
        //projecting points into the medial axis
        vec3 const projection_to_medial = joint + dot(position - joint,
                                                      medial_axis_normalized)*medial_axis_normalized;

        return (R*S*transpose(R) - mat3::identity())*(position - projection_to_medial);
    }

    vec3 displacement_floppy_linear(float const k_floppy, vec3 const& linear_velocity) {

        return -k_floppy*linear_velocity;
    }

    vec3 displacement_floppy_rotation(vec3 const& position, vec3 const& joint,
                                      vec3 const& angular_velocity, vec3 const& v_r,
                                      float const k_floppy) {
        vec3 const norm_angular_vel = normalize(angular_velocity);
        vec3 const proj_rotation_axis = joint + dot(position - joint, norm_angular_vel)*norm_angular_vel;

        float const theta = -k_floppy * norm(v_r);
        mat3 const R = rotation_from_axis_angle_mat3(norm_angular_vel, -theta);

        return (R - mat3::identity()) * (position - proj_rotation_axis);
    }
    void initialize_disp(displacement& displacement, buffer<int> parent_index, cgp::rig_structure rig){
        // constructing the graph corresponding to joint connections
        cgp::velocity_weights& velocity_weights = displacement.v_weights;
        velocity_weights.initialize_v_weight_struct(parent_index, rig);
        velocity_weights.initialize_bones_graph();
        velocity_weights.cal_v_weights();

        displacement.v_skinning_weights = velocity_weights.v_skinning_weights;
        displacement.k_squash = 0.5;
        displacement.k_floppy = 0.5;
    }
    void adding_velocity_def(displacement& displacement, buffer<cgp::affine_rt> skeleton, bool first_step, buffer<cgp::vec3>& vertex_positions, buffer<cgp::vec3> joint_positions){
        buffer<buffer<float>> v_skinning_weights = displacement.v_skinning_weights;
        cgp::velocity_components& v_components = displacement.v_components;
        float k_squash = displacement.k_squash;
        float k_floppy = displacement.k_floppy;
        v_components.update_skeletons(skeleton, first_step);

        if(first_step){
            return;
        }
        //std::cout << "adding the displacements induced by velocity" << std::endl;

        cal_comp_velocities(v_components);
        //std::cout << "velocity components calculated" << std::endl;
        buffer<cgp::vec3> translational_velocities = v_components.translational_velocities;
        buffer<cgp::vec3> angular_velocities = v_components.angular_velocities;
        // along with v_skinning_weights


        // have to compute the displacement for all the vertices

        int N_joint = translational_velocities.size();
        int N_vertex = vertex_positions.size();

        //std::cout << "N_vertex: " << N_vertex << std::endl;
        //std::cout << "N_joint: " << N_joint << std::endl;

        for(int i = 0; i < N_vertex; i++){
            cgp::vec3& vertex_position = vertex_positions[i];
            for(int j = 0; j < N_joint; j++){

                cgp::vec3 displacement;
                displacement.fill(0);
                cgp::vec3 joint_position = joint_positions[j];
                /*displacement_squashy_linear(vec3 const& position, vec3 const& center,
                                     vec3 const& linear_velocity, float const k_squash)*/
                /*displacement_squashy_rotation(vec3 const& position, vec3 const& joint,
                                       vec3 const& v_r, vec3 const& angular_velocity,
                                       float const k_squash)*/

                displacement = displacement + displacement_squashy_linear(vertex_position, joint_position,
                                                                          translational_velocities[j], k_squash);

                displacement = displacement + displacement_squashy_rotation(vertex_position, joint_position,
                                                                            translational_velocities[j], angular_velocities[j],
                                                                            k_squash);
                std::cout << v_skinning_weights[i][j] << std::endl;
                vertex_positions = vertex_positions + displacement * v_skinning_weights[i][j];
            }

        }
    }
}