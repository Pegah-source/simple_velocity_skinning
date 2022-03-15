#pragma once

#include "cgp/cgp.hpp"
#include "skeleton/skeleton.hpp"
#include "skinning/skinning.hpp"


namespace cgp
{

	buffer<buffer<float>> v_skinning_weights; // of size N_vertex*N_joint

    struct bone_node // for each joint we store his children and his parent
    {
        buffer<int> children;
        int parent;
    };
    struct bones_graph // storing the information that would help in traversing the graph of joints
    {
        buffer<bone_node> Nodes;
        buffer<int> N_children; // for each joint stores the number of children
        int root; // the index of the bone_node that does not have any parent
    };

    // the function to construct the graph of bones from the parent list
    void initialize_bones_graph(bones_graph &graph,
                                buffer<int> parent_index);
    
    // function to traverse the graph and compute the velocity skinning weights
	void initialize_v_weights(velocity_rig_structure &v_rig,
                              bones_graph &graph,
                              rig_structure rig);
}