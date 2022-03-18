#include "velocity_skinning_weights.hpp"

namespace cgp
{
    void velocity_weights::initialize_v_weight_struct(buffer<int> parent_index_, rig_structure rig_){
        rig = rig_;
        parent_index = parent_index_;
        int N_vertex = rig.joint.size();
        int N_joint = parent_index.size();
        v_skinning_weights.resize(N_vertex);
        // initialize the components of the graph:
        graph.root = 0;
        graph.Nodes.resize(N_joint); // each element => bone_node => children and parent
        graph.N_children.resize(N_joint);
        graph.N_children.fill(0);

    }

    void velocity_weights::initialize_bones_graph(){

        for(int b = 0; b < N_joint; b++){
            int parent_b = parent_index[b];
            graph.Nodes[b].parent = parent_b;
            graph.N_children[b] += 1;
            graph.Nodes[parent_b].children.push_back(b);
        }

    }

    void velocity_weights::cal_v_weights(){

        int N_vertex = rig.joint.size();
        int N_joint = graph.Nodes.size();
        assert_cgp_no_msg(rig.weight.size()==N_vertex);

        for(int u = 0; u < N_vertex; u++){
            // to be filled with right values
            buffer<float> &v_rig_vertex = v_skinning_weights[u];


            // rig information
            buffer<int> connected_joints = rig.joint[u];
            buffer<float> connected_joints_weights = rig.weight[u];
            int N_connected_joints = connected_joints.size();
            assert_cgp_no_msg(connected_joints_weights.size()==N_connected_joints);

            v_rig_vertex.resize(N_joint);
            v_rig_vertex.fill(0);

            // initialize all the weights with LBS weights
            // for all the joints that are connected to the bone u
            for(int i = 0; i < N_connected_joints; i++){
                int joint_index = connected_joints[i];
                float joint_weight = connected_joints_weights[i];
                v_rig_vertex[joint_index] += joint_weight;
            }

            int root = graph.root;
            // make a copy of graph information:
            // MAKE SURE THAT IT REALLY COPIES!!
            buffer<bone_node> Nodes = graph.Nodes;
            buffer<int> N_children = graph.N_children;
            assert_cgp_no_msg(N_children.size()==N_joint);

            bone_node current_node = Nodes[root];
            int current_node_index = root;
            bool TheEnd = false;

            // Traverse the graph with algorithm 2 (in the report), update weights

            while(!TheEnd){
                if(N_children[current_node_index] != 0){// descending in the tree
                    // current_node has at least one unvisited child
                    int last_child_index = N_children[current_node_index];
                    // last_child_index = index of the next node in the children list of the current_node
                    N_children[current_node_index] -= 1;
                    // update the current_node and its index
                    current_node_index = current_node.children[last_child_index-1];
                    current_node = Nodes[current_node_index];
                }else{
                    // it means: we can descend no more
                    // so climb in the tree and add child's weight to his parent
                    if(current_node_index == root){
                        TheEnd = true;
                    }else{
                        v_rig_vertex[current_node.parent] += v_rig_vertex[current_node_index];
                        current_node_index = current_node.parent;
                        current_node = Nodes[current_node_index];
                    }
                    
                }
            }
            

        }
    }

	
#ifdef SOLUTION
quaternion_dual::quaternion_dual()
    :q(), qe()
{}
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

void skinning_DQS_compute(
		buffer<vec3>& position_skinned,
		buffer<vec3>& normal_skinned,
		buffer<affine_rt> const& skeleton_current,
		buffer<affine_rt> const& skeleton_rest_pose,
		buffer<vec3> const& position_rest_pose,
		buffer<vec3> const& normal_rest_pose,
		rig_structure const& rig)
	{
		size_t const N_vertex = position_rest_pose.size();
		size_t const N_joint = skeleton_current.size();

		assert_cgp_no_msg(position_skinned.size()==N_vertex);
		assert_cgp_no_msg(normal_skinned.size()==N_vertex);
		assert_cgp_no_msg(normal_rest_pose.size()==N_vertex);
		assert_cgp_no_msg(skeleton_rest_pose.size()==N_joint);
		assert_cgp_no_msg(rig.joint.size()==N_vertex);
		assert_cgp_no_msg(rig.weight.size()==N_vertex);

		buffer<quaternion_dual> dq;
		dq.resize(N_joint);
		for (size_t kj = 0; kj < N_joint; ++kj) {
			rotation_transform const& r  = skeleton_current[kj].rotation;
			rotation_transform const& r0 = skeleton_rest_pose[kj].rotation;
			vec3 const& t = skeleton_current[kj].translation;
			vec3 const& t0 = skeleton_rest_pose[kj].translation;
			dq[kj] = quaternion_dual( (r*inverse(r0)).data, r*inverse(r0)*(-t0)+t );
		}

		
		for (size_t k_vertex = 0; k_vertex < N_vertex; ++k_vertex)
		{
			vec3& p = position_skinned[k_vertex];
			vec3& n = normal_skinned[k_vertex];
			vec3 const& p0 = position_rest_pose[k_vertex];
			vec3 const& n0 = normal_rest_pose[k_vertex];

			size_t const N_dep = rig.joint[k_vertex].size();
			quaternion_dual d = { {0,0,0,0}, {0,0,0,0} };
			for (size_t kj = 0; kj < N_dep; ++kj)
			{
				size_t const joint = rig.joint[k_vertex][kj];
				float const w = rig.weight[k_vertex][kj];
				d += w*dq[joint];
			}

			float const q_n = norm(d.q);
			if( std::abs(q_n)>1e-5f)
				d = d/q_n;

			p = rotation_transform(d.q)*p0 + d.translation();
			n = rotation_transform(d.q)*n0;
		}

	}
#endif
}