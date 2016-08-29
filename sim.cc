#include <iostream>
#include <fstream>
#include <sstream>

#include "sim.hh"

// Default Simulator: 
//  May only specify number of boxes and robots all other 
//  parameters are fixed.
Sim::Sim( int numBoxes, int numRobots ) :
    timestep(1.0/30.0)
{
    World::showGui = true;

    worldSet.width = 10;
    worldSet.height = 10;
    worldSet.numBoxes = numBoxes;
    worldSet.boxShape = SHAPE_CIRC;
    worldSet.numRobots = numRobots;
    worldSet.numPatterns = 1;
    worldSet.avoidLuminance = 0.2;
    worldSet.bufferLuminance = 0.4;

    glcSet.goalError = 0;
    glcSet.trialTime = 10e5;
    glcSet.dimGrid = 1;
}

Sim::Sim( const std::string& goalFile ) : 
    timestep(1.0/30.0)
{ 
    ParseFile(goalFile); 
    std::cout << "===> Show GUI? " << World::showGui << "\n";
    // Open file
}

Sim::~Sim(){
    for( Goal* goal : goals )
        delete goal;
    // Close file: you have to be very careful here if something
    //  breaks and you exist without closing the file properly there
    //  might be issues...
}

void Sim::Run( void ){
    if( World::showGui ){
        GuiWorld world( worldSet, glcSet, goals );
        while( !world.RequestShutdown() )
            world.Step( goals, timestep, results );
    } else {
        for( int r = 0; r < simSet.stepsNumRobots; ++r ){
            worldSet.numRobots += simSet.incNumRobotsBy;
            for( int trial = 0; trial < simSet.numTrials; ++trial ){
                GuiWorld world( worldSet, glcSet, goals );

                /* Loop until the user closes the window */
                while( !world.RequestShutdown() and 
                       !world.Step(goals, timestep, results) ){}

                // Processing the data in results
                ParseResults();
            }
        }
    }
}

void Sim::ParseFile( const std::string& fileName ){
    bool setParameters = false;
    int paramIndex = 0;
    int row, col, indexTR, indexTL, indexBL, indexBR;
    float cellWidth, xCoord, yCoord;
    std::string separator = "-"; // deliminates settings and goals

    if( fileName.empty() ){ // DEFAULT
        std::cout << "[WARNING] No file name given. Using DEFAULT.\n";
    } else {                // READ FROM FILE
        std::string goal, x, y, param;
        std::ifstream goalFile(fileName.c_str());
        if( goalFile.is_open() ){
            while( std::getline(goalFile, goal) ){
                std::stringstream ss(goal);
                ss >> x >> y;
                if( goal.size() > 0 and x.at(0) != '#'){
                // If current line is not empty or a comment...
                    if( !setParameters ){ 
                    // If not yet finished setting the parameters...
                        if( x.compare(separator) == 0 ){
                            setParameters = true;
                            cellWidth = worldSet.width / glcSet.dimGrid;
                        } else {
                            ParseSettings(paramIndex, y);
                            ++paramIndex;
                        }
                    } else { 
                    // Finished setting parameters, settings goals...
                        row = std::stoi(x);
                        col = std::stoi(y);
                        indexBL = row*glcSet.dimGrid + col; //Bottom Left 
                        indexTL = indexBL + glcSet.dimGrid;
                        indexTR = indexTL + 1;
                        indexBR = indexBL + 1;
                        xCoord = (col+1)*cellWidth;
                        yCoord = (row+1)*cellWidth; 
                        // printf("TR: %i TL: %i BL: %i BR: %i\n", indexTR, indexTL, indexBL, indexBR);
                        goals.push_back( new Goal( xCoord, yCoord, 
                                        cellWidth, indexTR, indexTL, 
                                        indexBL, indexBR) );
                    }
                }
            }
            goalFile.close();
        } else {
            std::cout << "[ERR] Unable to open file: " << fileName << std::endl;
        }
    }
}

void Sim::ParseSettings( int index, const std::string& value ){
    switch( index ){
        case 0: World::showGui = std::stoi(value) == 1; break;
        case 1: simSet.numTrials = std::stoi(value); break;
        case 2: simSet.incNumRobotsBy = std::stoi(value); break;
        case 3: simSet.stepsNumRobots = std::stoi(value); break;
        case 4: 
            worldSet.avoidLuminance = 0.2;
            worldSet.avoidLuminance = 0.4;
            worldSet.width = std::stof(value); 
            break;
        case 5: worldSet.height = std::stof(value); break; 
        case 6: worldSet.numBoxes = std::stoi(value); break;
        case 7: Box::size = std::stof(value); break;
        case 8:
            switch(value.c_str()[0]){
                case 'C': case 'c':
                    worldSet.boxShape = SHAPE_CIRC;
                    break;
                case 'S': case 's':
                    worldSet.boxShape = SHAPE_RECT
                    break;
                case 'H': case 'h':
                    worldSet.boxShape = SHAPE_HEX;
                    break;
                default:
                    std::cout << "[ERR-World] invalid box shape: " << 
                        value.c_str() << "\n";
                    exit(1);
            } break;
        case 9: worldSet.numRobots = std::stoi(value); break;
        case 10: Robot::size = std::stof(value); break;
        case 11: worldSet.numPatterns = std::stoi(value); break; 
        case 12: glcSet.goalError = std::stof(value); break;
        case 13: glcSet.trialTime = std::stof(value); break;
        case 14: glcSet.dimGrid = std::stoi(value); break;
        default:
            std::cout << "[ERR] SIM: (ParseSettings) index out of bounds.\n";
    }
}

void Sim::ParseResults( void ){

}
