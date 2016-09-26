#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <math.h>
#include <stdio.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "push.hh"

/***
 * CLASS:   Sim
 * PURPOSE: Starting point of all simulations. This class controls
 *          all the type of simulation, and the data collection. 
 *          Sets up all the settings for the simulator, world and
 *          grid light controller classes. Using DataWriter class
 *          to write to various files.
 *
 ***/
struct SimSettings{
    int controlVal;
    int numTrials;
    float incNumPucksBy;
    int incNumRobotsBy;
    int steps;
};

class DataWriter{
    private:
        std::fstream out;
    public:
        explicit DataWriter( const std::string& filename ) :
            out( filename.c_str(), std::fstream::out ){
        std::cout << filename << std::endl;     
        }
        ~DataWriter(){ out.close(); }
        bool IsOpen( void ){ return out.is_open(); }
        void WriteToFile( bool endLine, const std::string& msg );
        void WriteLineToFile( const std::vector<float>& line );
};

class Sim{
    public:
        /***
         * Default Simulator
         * PURPOSE: Creates an instance of the default simulator.
         *          Allows the users to specify number of robots and
         *          boxes, but all other criterion are fixed.
         * INPUT:   numPucks (int) - number of pucks in sim
         *          numRobots (int) - number of robots in sim
         * OUPUT:   (None) 
         ***/
        Sim( int numPucks, int numRobots );
        /***
         * Goal File Simulator
         * PURPOSE: Creates an instance of the simulator generated 
         *          by goal files.
         * INPUT:   goalFile (string) - name of the goal file 
         *          (no extension) 
         * OUPUT:   (None)   
         ***/
        Sim( const std::string goalFile );
        ~Sim();
        void Run( void );

    private:
        float timestep;
        SimSettings simSet;
        WorldSettings worldSet;
        GridLightControllerSettings glcSet;
        SimResults results;

        std::string fileName;       // Name of goal file
        std::string fileNameExt;    // Full path to goal file
        std::vector<Goal*> goals;

        /***
         * PURPOSE: Read the goal file, reads in the settings 
         *          for the simulator, world, and grid light
         *          controllers as strings. Populates the "goals"
         *          vectors with Goals.
         * INPUT:   (None) 
         * OUPUT:   (None) 
         ***/
        void ParseFile( void );
        /***
         * PURPOSE: Read in the settings strings ands sets 
         *          SimSettings, WorldSettings, and 
         *          GridLightController Settings values.  
         * INPUT:   index (int) - value to set 
         *          value (string) - string representation of 
         *          setting values
         * OUPUT:   (None) 
         ***/
        void ParseSettings( int index, const std::string& value );
        void ParseResults( void );
};

#endif
