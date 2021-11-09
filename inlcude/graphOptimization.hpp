#ifndef INCLUDE_GRAPHOPTIMIZATION_HPP_
#define INCLUDE_GRAPHOPTIMIZATION_HPP_

#include <ros/ros.h>
#include <cmath>
#include <iostream>

class GraphOptimization
{
    public:

        GraphOptimization();

        ~GraphOptimization();

        void addSE2Vertex();

        void addSE2Edge();

        void optimizeGraph();

    private:


};

#endif  //  INCLUDE_GRAPHOPTIMIZATION_HPP_