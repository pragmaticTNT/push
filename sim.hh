#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <math.h>
#include <stdio.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "push.hh"

struct SimSettings{
    int controlVal;
    int numTrials;
    float incNumBoxesBy;
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
        Sim( int numBoxes, int numRobots );
        Sim( const std::string goalFile );
        ~Sim();
        void Run( void );

    private:
        float timestep;
        SimSettings simSet;
        WorldSettings worldSet;
        GridLightControllerSettings glcSet;
        SimResults results;

        std::string fileName;
        std::string fileNameExt;
        std::vector<Goal*> goals;

        void ParseFile( void );
        void ParseSettings( int index, const std::string& value );
        void ParseResults( void );
};

#endif
