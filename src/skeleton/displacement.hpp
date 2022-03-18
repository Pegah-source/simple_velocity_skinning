#pragma once

#include "cgp/cgp.hpp"
#include "velocity_skinning/velocit_skinning_weights.hpp"
#include ""


namespace cgp
{
    struct velocity_weights{

        buffer<buffer<float>> v_skinning_weights; // of size N_vertex*N_joint
        bones_graph graph;
        rig_structure rig;
        buffer<int> parent_index;

        // the function to construct the graph of bones from the parent list
        /*void initialize_bones_graph(bones_graph &graph,
                                    buffer<int> parent_index);*/
        
        // function to traverse the graph and compute the velocity skinning weights
        /*void initialize_v_weights(buffer<buffer<float>> &v_skinning_weights;,
                                bones_graph &graph,
                                rig_structure rig);*/

        void initialize(buffer<int> parent_index, rig_structure rig);
        void initialize_bones_graph();
        void initialize_v_weights();
    }
}