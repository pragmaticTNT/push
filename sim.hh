#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <math.h>
#include <stdio.h>

#include <string>
#include <vector>

#include "push.hh"

struct SimSettings{
    int numTrials;
    int incNumRobotsBy;
    int stepsNumRobots;
};

class Sim{
    public:
        Sim( int numBoxes, int numRobots );
        Sim( const std::string& goalFile );
        ~Sim();
        void Run( void );

    private:
        float timestep;
        SimSettings simSet;
        WorldSettings worldSet;
        GridLightControllerSettings glcSet;
        SimResults results;
        std::vector<Goal*> goals;

        void ParseFile( const std::string& goalFile );
        void ParseSettings( int index, const std::string& value );
};

#endif
