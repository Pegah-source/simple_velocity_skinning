#pragma once

#include "cgp/cgp.hpp"
#include "skeleton/skeleton.hpp"
#include "skinning/skinning.hpp"


namespace cgp
{
	struct velocity_rig_structure
	{
		buffer<buffer<float>> v_skinning_weights; // of size N_vertex*N_joint
	};
    struct bone_node
    {
        buffer<int> children;
        int parent;
    };
    struct bones_graph
    {
        buffer<bone_node> Nodes;
        buffer<int> N_children; // for each joint stores the number of children
        int root; // the index of the bone_node that does not have any parent
    };

    // the function to construct the graph of bones from the parent list
    void initialize_bones_graph(bones_graph &graph,
                              skeleton_animation_structure anim_struct);

	void initialize_v_weights(velocity_rig_structure &v_rig,
                              bones_graph &graph,
                              rig_structure rig);
}