#include <sstream>

#include "sim.hh"

void DataWriter::WriteToFile( bool endLine, const std::string& msg ){
    if( endLine )
        out << msg << "\n";
    else
        out << msg << ", ";
}

void DataWriter::WriteLineToFile( const std::vector<float>& line ){
    for( size_t i = 0; i < line.size(); ++i ){
        if( i != line.size()-1 )
            out << std::to_string(line[i]) << ", ";
        else
            out << std::to_string(line[i]) << "\n";
    }
}

// Default Simulator: 
//  May only specify number of boxes and robots all other 
//  parameters are fixed.
Sim::Sim( int numBoxes, int numRobots ) : 
    timestep(1.0/30.0),
    fileNameExt("")
{
    World::showGui = true;

    simSet.controlVal = 0;

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

Sim::Sim( const std::string goalFile ) : 
    timestep(1.0/30.0),
    fileName(goalFile),
    fileNameExt("templates/" + goalFile)
{ ParseFile(); }

Sim::~Sim(){
    for( Goal* goal : goals )
        delete goal;
}

void Sim::Run( void ){
    if( World::showGui ){
        DataWriter boxDist("data/boxDist_" + fileName);
        std::cout << "===> BEGIN EXPERIMENT...\n";
        GuiWorld world( simSet.controlVal, worldSet, glcSet, goals );
        std::vector<float> boxDistErr;
        while( !world.RequestShutdown() and 
                world.Step(simSet.controlVal, goals, timestep, 
                           results, boxDistErr) ){
            if( simSet.controlVal == 3 and boxDistErr.size() ){
                boxDist.WriteLineToFile( boxDistErr ); 
                boxDistErr.clear();
            }
        }
        boxDist.WriteLineToFile( boxDistErr ); 
        std::cout << "===> FINISH EXPERIMENT!\n";
    } else {
        DataWriter taskData("data/taskTime_" + fileName);
        DataWriter robotsData("data/robotsMove_" + fileName);
        // std::cout << "Task Data is open? " << taskData.IsOpen() << "\n";
        std::cout << "===> BEGIN WRITING TO FILE...\n";
        int indepVar = simSet.controlVal == 4 ? 
            worldSet.numBoxes :
            worldSet.numRobots;
        int incAmount = simSet.controlVal == 4 ?
            simSet.incNumBoxesBy :
            simSet.incNumRobotsBy;
        for( int r = 0; r < simSet.steps; ++r ){
            bool endLine = false;
            std::vector<float> boxDist;
            taskData.WriteToFile( endLine, std::to_string(indepVar) );
            robotsData.WriteToFile( endLine, std::to_string(indepVar) );
            for( int trial = simSet.numTrials; trial > 0; --trial ){
                std::cout << "===  [" << indepVar << "," 
                    << simSet.numTrials-trial+1 << "]\n"; 
                endLine = trial == 1;
                GuiWorld world( simSet.controlVal, worldSet, glcSet, goals );

                /* Loop until the user closes the window */
                while( !world.RequestShutdown() and 
                        world.Step(simSet.controlVal, goals, timestep, 
                                   results, boxDist) ){}
                taskData.WriteToFile( endLine, 
                        std::to_string(results.taskCompletionTime) );
                robotsData.WriteToFile( endLine, 
                        std::to_string(results.robotMoveDistance) );
            }
            if( simSet.controlVal == 4 )
                worldSet.numBoxes += simSet.incNumBoxesBy;
            else
                worldSet.numRobots += simSet.incNumRobotsBy;
            indepVar += incAmount;
        }
        std::cout << "===> FINISH WRITING TO FILE!\n";
    }
}

void Sim::ParseFile( void ){
    bool setParameters = false;
    int paramIndex = 0;
    int row, col, indexTR, indexTL, indexBL, indexBR;
    float cellWidth, xCoord, yCoord;
    std::string separator = "-"; // deliminates settings and goals

    if( !fileNameExt.empty() ){ // DEFAULT
        std::string goal, x, y, param;
        std::ifstream goalFile(fileNameExt.c_str());
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
            SimException e("[ERR]", "---Sim(ParseFile)---",
                    "Unable to open file", fileNameExt );
            throw e;
        }
    } else {                // READ FROM FILE
        SimException e("[ERR]", "---Sim(ParseFile)---",
                "File name is empty", "" );
        throw e;
    }
}

void Sim::ParseSettings( int index, const std::string& value ){
    switch( index ){
        case 0: World::showGui = std::stoi(value) == 1; break;
        case 1: simSet.controlVal = std::stoi(value); break;
        case 2: simSet.numTrials = std::stoi(value); break;
        case 3: simSet.incNumBoxesBy = std::stof(value); break;
        case 4: simSet.incNumRobotsBy = std::stoi(value); break;
        case 5: simSet.steps = std::stoi(value); break;
        case 6: 
            worldSet.avoidLuminance = 0.2;
            worldSet.avoidLuminance = 0.4;
            worldSet.width = std::stof(value); 
            break;
        case 7: worldSet.height = std::stof(value); break; 
        case 8: worldSet.numBoxes = std::stoi(value); break;
        case 9: Box::size = std::stof(value); break;
        case 10:
            switch(value.c_str()[0]){
                case 'C': case 'c':
                    worldSet.boxShape = SHAPE_CIRC;
                    break;
                case 'S': case 's':
                    worldSet.boxShape = SHAPE_RECT;
                    break;
                case 'H': case 'h':
                    worldSet.boxShape = SHAPE_HEX;
                    break;
                default:
                    SimException e( "[ERR]", 
                            "---Sim(ParseSettings)---",
                            "Invalid Box Shape", 
                            value.c_str() );
                    throw e;
            } break;
        case 11: worldSet.numRobots = std::stoi(value); break;
        case 12: Robot::size = std::stof(value); break;
        case 13: worldSet.numPatterns = std::stoi(value); break; 
        case 14: glcSet.goalError = std::stof(value); break;
        case 15: glcSet.trialTime = std::stof(value); break;
        case 16: glcSet.dimGrid = std::stoi(value); break;
        default:
            SimException e( "[ERR]", "---Sim(ParseSettings)---",
                    "Index out of bounds", "" );
            throw e;
    }
}
