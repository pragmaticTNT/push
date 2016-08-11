#include <iostream>

#include "push.hh"

// ===> LIGHTCONTROLLER class methods
LightController::LightController( float avoidIntensity, float bufferIntensity, float radiusSmall ) : 
    avoidIntensity( avoidIntensity ),
    bufferIntensity( bufferIntensity ),
    radiusSmall(radiusSmall),
    scaleFactor(GetScaleFactor(radiusSmall)),
    radiusLarge(GetRadiusLarge()),
    goalError(0),
    isFilled(false)
{ 
    Light::radiusSmall = radiusSmall;
    Light::radiusLarge = radiusLarge;
}

LightController::~LightController(){
    for( auto light : lights )
        delete light;
}
// ===> END LIGHTCONTROLLER class methods


// ===> RADIALLIGHTCONTROLLER class methods
RadialLightController::RadialLightController( const std::vector<Goal*>& goals, std::queue<std::string>& settings, float avoidIntensity, float bufferIntensity, float radiusSmall ) :
    LightController( avoidIntensity, bufferIntensity, radiusSmall ),
    radiusInit(radiusSmall),
    growRate(0),
    timeElapsed(0),
    maxRadius(0),
    startScramble(false),
    scramble(false)
{
    size_t numParams = settings.size();
    
    // When in doubt check the most recent pattern file
    goalError = numParams > 0 ? std::stof(settings.front()) : 0.2;
    settings.pop();
    trialTime = numParams > 1 ? std::stof(settings.front()) : 2000;
    settings.pop();
    growRate = numParams > 2 ? std::stof(settings.front()) : -10e-4;
    settings.pop();
    //TODO: fix this rather annoying hack
    patternTime = numParams > 3 ? std::stof(settings.front()) : 0;
    repeatPattern = numParams > 4 ? std::stoi(settings.back()) == 1 : false;
    for( auto goal : goals ){
        lights.push_back( new Light(goal->GetCenter()) );
    } 
}

float RadialLightController::UpdateGoalsInfo( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes ){
    float minAmongBoxes = INT_MAX, maxAmongGoals = 0;
    for( Goal* goal: goals )
        goal->GetCenter();
    for( Box* box : boxes ){
        box->GetCenter();
        minAmongBoxes = INT_MAX;
        for( Goal* goal : goals ){
            float dist = sqrt(box->SqrDistanceTo(*goal));
            minAmongBoxes = std::min(minAmongBoxes, dist);
            goal->filled = dist < goalError;
        }
        maxAmongGoals = std::max(maxAmongGoals, minAmongBoxes);
    }
    return maxAmongGoals;
}

float RadialLightController::GetIntensity( float x, float y ){
    float brightness = 1.0;
    for( auto light : lights ){
        float intensity = light->GetIntensity(x, y, scaleFactor);
        brightness = std::min(brightness, intensity);
        //brightness *= 1-1.0/(1+scaleFactor*(light->SqrDistanceTo(x,y)));
    }
    return scramble ? 1.0 : brightness;
}

// -> Light off-center to the boxes
void RadialLightController::Update( 
        const std::vector<Goal*>& goals,
        const std::vector<Box*>& boxes)
{ 
    for( int i=0; i<std::min(goals.size(),boxes.size()); ++i ){
        b2Vec2 goal = goals[i]->GetCenter();
        b2Vec2 box = boxes[i]->GetCenter();
        float distBoxToGoal = sqrt(boxes[i]->SqrDistanceTo(*goals[i]));

        if(distBoxToGoal > goalError){
            float x = (goal.x-box.x)*radiusSmall/distBoxToGoal + box.x;
            float y = (goal.y-box.y)*(x-box.x)/(goal.x-box.x) + box.y;
            lights[i]->SetCenter(x,y);
        } else {
            goals[i]->filled = true;
            lights[i]->SetCenter(goal);
            //std::cout << "[GOAL REACHED]" << std::endl;
        }
        //goals[i]->WhereAmI();
        //boxes[i]->WhereAmI();
        //lights[i]->WhereAmI();
    }
}

// -> Light centered at the goal slowly diminishing
void RadialLightController::Update( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes, float timeStep ){
    timeElapsed += timeStep;
    if( !scramble ){
        if( fabs(maxRadius) > EPSILON )
            maxRadius = UpdateGoalsInfo( goals, boxes );
        radiusSmall = std::max(radiusInit, maxRadius + growRate*timeElapsed);
        scaleFactor = GetScaleFactor(radiusSmall);
        radiusLarge = GetRadiusLarge();
        Light::radiusSmall = radiusSmall;
        Light::radiusLarge = radiusLarge; 
    }

    if( timeElapsed > trialTime ){
        timeElapsed = 0;
        maxRadius = UpdateGoalsInfo( goals, boxes );
        if( !(maxRadius < goalError) ){
            std::cout << "Current max error: " << maxRadius << std::endl;
            if( maxRadius < radiusInit || scramble ){
                if( !startScramble ){
                    startScramble = true;
                } else if ( !scramble ){
                    std::cout << "Local minimum reached. Scrambling..." << std::endl;
                    scramble = true; 
                    Light::radiusSmall = 0;
                    Light::radiusLarge = 0;
                } else{
                    startScramble = false;
                    scramble = false;
                }
            }
        } else {
            isFilled = true;
            std::cout << "Goal shapes obtained." << std::endl;
            // May need to keep on checking to maintain shape
        }
    }
}
// ===> END RADIALLIGHTCONTROLLER class methods


// ===> GRID LIGHT CONTROLLER class methods
GridLightController::GridLightController( const std::vector<Goal*>& goals, std::queue<std::string>& settings, float avoidIntensity, float bufferIntensity, float cellWidth ) : 
    LightController( 
            avoidIntensity, 
            bufferIntensity, 
            cellWidth*sqrt(2)/2.0 ),
    dimCell(cellWidth),
    timeElapsed(0)
{
    int goalIndex;
    size_t numParams = settings.size();

    goalError = numParams > 0 ? std::stof(settings.front()) : 0.2;
    settings.pop();
    trialTime = numParams > 1 ? std::stof(settings.front()) : 500;
    settings.pop();
    dimGrid = numParams > 2 ? std::stoi(settings.front()) : 11;
    settings.pop();
    layersActive = 4;
    currentLayer = dimGrid-1;
    
    std::cout << "[GLC] Setting Goals...\n";
    for( int row = 0; row < dimGrid; ++row )
        for( int col = 0; col < dimGrid; ++col )
            lights.push_back( new Light(GetLightCenter(row, col)) );

    // Setting goal lights
    for( auto goal : goals ){
        goalIndex = goal->index;
        lights[goalIndex]->atGoal = true;
    }

    PreprocessLayers(goals);
    SeeLayerDistribution();
    currentLayer = maxLayer;
    std::cout << "Max Layer: " << maxLayer << "\n";
    for( int i = 0; i < layersActive; ++i ){
        Toggle(currentLayer);
        currentLayer = NextLayer(currentLayer);
    }
    currentLayer = maxLayer;
}

// RETURN: index of n-bour if exists otherwise index of cell 
int GridLightController::NeighbourIndex( int cell[2], neighbour_t n ){
    int row, col;
    switch(n){
        case TR: row = -1; col = 1; break; 
        case TC: row = -1; col = 0; break;
        case TL: row = -1; col = -1; break;
        case CL: row = 0; col = -1; break;
        case BL: row = 1; col = -1; break;
        case BC: row = 1; col = 0; break;
        case BR: row = 1; col = 1; break;
        case CR: row = 0; col = 1; break;
    }
    return RowColToIndex(cell[0]+row, cell[1]+col);
}

// NOTE: processed cells are marked by turning the light ON!
void GridLightController::PreprocessLayers( const std::vector<Goal*>& goals ){
    int localMaxLayer = 0;
    for( int layer = 0; layer < dimGrid; ++layer ){
        std::vector<int> newLayer;
        layerIndicies.push_back(newLayer);
        for( auto goal : goals ){
            if( layer > 0 ){
                int nBourIndex = 0;
                int cell[2] = {0, 0};
                bool quadrants[8] = {
                    true, true, true, true, 
                    true, true, true, true 
                };
                IndexToRowCol( goal->index, cell );

                // Mark quadrants if neighbour cells are goal cells
                for( int i = 0; i < NBOUR_NUM; ++i ){
                    neighbour_t n = static_cast<neighbour_t>(i);
                    nBourIndex = NeighbourIndex(cell, n); 
                    if( lights[nBourIndex]->atGoal ) 
                        MarkQuadrants( quadrants, n ); 
                }
                for( int i = 0; i < NBOUR_NUM; ++i ){
                    neighbour_t n = static_cast<neighbour_t>(i);
                    if( quadrants[i] )
                        AddLayerIndicies( layer, cell, n );
                }
            } else {
                layerIndicies[layer].push_back(goal->index);
                lights[goal->index]->on = true;
            }
        } 
    }
    for( auto light: lights ){
        if( light->layer > localMaxLayer )
            localMaxLayer = light->layer;
        light->on = false;
    }
    maxLayer = localMaxLayer;
}

void GridLightController::MarkQuadrants( bool quadrants[8], neighbour_t n ){
    switch(n){
        case TR: quadrants[0] = false; break; 
        case TC: 
                 quadrants[0] = false; 
                 quadrants[1] = false;
                 quadrants[2] = false;
                 break; 
        case TL: quadrants[2] = false; break; 
        case CL:  
                 quadrants[2] = false; 
                 quadrants[3] = false;
                 quadrants[4] = false;
                 break; 
        case BL: quadrants[4] = false; break; 
        case BC: 
                 quadrants[4] = false; 
                 quadrants[5] = false;
                 quadrants[6] = false;
                 break; 
        case BR: quadrants[6] = false; break; 
        case CR: 
                 quadrants[6] = false; 
                 quadrants[7] = false;
                 quadrants[0] = false;
                 break; 
    }
}

void GridLightController::AddLayerIndicies( int layer, int cell[2], neighbour_t n ){
    bool oneCell = true;
    int rowMod = 0, colMod = 0, index = 0;

    // If only oneCell then changes the setting of a cell at a distance
    // "layer" away. Otherwise changes an entire diagonal.
    switch( n ){
        case TR: oneCell = false; rowMod = -1; colMod = 1; break; 
        case TC: oneCell = true; rowMod = -layer; colMod = 0; break;
        case TL: oneCell = false; rowMod = -1; colMod = -1; break;
        case CL: oneCell = true; rowMod = 0; colMod = -layer; break;
        case BL: oneCell = false; rowMod = 1; colMod = 1; break;
        case BC: oneCell = true; rowMod = layer; colMod = 0; break;
        case BR: oneCell = false; rowMod = 1; colMod = -1; break;
        case CR: oneCell = true; rowMod = 0; colMod = layer; break;
    }

    if( oneCell ){
        index = RowColToIndex(cell[0]+rowMod, cell[1]+colMod);
        if( index >= 0 && !lights[index]->on ){
            layerIndicies[layer].push_back(index);
            lights[index]->on = true;
            lights[index]->layer = layer;
        }
    } else{
        for( int i = 1; i < layer; ++i ){
            index = RowColToIndex( 
                    cell[0]+rowMod*i, 
                    cell[1]+colMod*(layer-i) 
                    );  
            if( index >= 0 && !lights[index]->on ){
                layerIndicies[layer].push_back(index);
                lights[index]->on = true;
                lights[index]->layer = layer;
            }
        }
    }
}

bool GridLightController::NoBoxOutside( const std::vector<Box*>& boxes ){
    int lightIndex, layer;
    int cell[2];
    for( Box* box : boxes ){
        b2Vec2 boxCenter = box->GetCenter();
        PointInCell( boxCenter.x, boxCenter.y, cell );
        lightIndex = RowColToIndex( cell[0], cell[1] );
        layer = lights[lightIndex]->layer;
        if( lightIndex >= 0 and layer-1 >= currentLayer-layersActive ){
            std::cout << "Row: " << cell[0] << " Col: " << cell[1] << " lightIndex: " << layer << " currentLayer: " << currentLayer << "\n"; 
            return false;
        } 
    }
    return true;
}

void GridLightController::SeeLayerDistribution( void ){
    std::cout << "\n";
    for( int row = dimGrid-1; row >= 0; --row ){
        for( int col = 0; col < dimGrid; ++col )
            std::cout << lights[RowColToIndex(row, col)]->layer << " ";
        std::cout << "\n";
    }
    std::cout << "\n";
}

void GridLightController::Toggle( int layer ){
    if( layer > 0){
        // std::cout << "Toggling layer: " << layer << std::endl;
        for( int index : layerIndicies[layer] )
            lights[index]->on = !lights[index]->on;
    } else {
        Toggle( layer+dimGrid-1 ); // Modular arithmetics
    }
}

void GridLightController::TurnOffAllLights( void ){
    for( Light* light : lights ){
        light->on = false;
    }
}

bool GridLightController::GoalObtained( const std::vector<Box*>& boxes, const std::vector<Goal*>& goals ){
    bool goalFilled = false;
    for( Goal* goal : goals ){
        for ( Box* box : boxes ){
            // TODO: check which GetCenter() is called
            if( sqrt(box->SqrDistanceTo(*goal)) < goalError )
                goalFilled = true;
        }
        if( not goalFilled )
            return false;
        goalFilled = false;
    }
    return true;
}

float GridLightController::GetIntensity( float x, float y ){
    int cell[2] = {};
    int index;
    neighbour_t nBour;
    std::vector<float> cells;   // All adjacent cells

    PointInCell( x, y, cell );
    int cellIndex = RowColToIndex(cell[0], cell[1]);

    // std::cout << "row: " << cell[0] << " col: " << cell[1] << " index: " << cellIndex << std::endl;
    if( cellIndex >= 0 ){
        cells.push_back(lights[cellIndex]->GetIntensity(x,y,scaleFactor));

        // Check intensities of all 8 adjacent cells
        for( int i = 0; i < NBOUR_NUM; ++i){
            nBour = static_cast<neighbour_t>(i);
            index = NeighbourIndex( cell, nBour );
            if( index >= 0 ){
                cells.push_back(lights[index]->GetIntensity(x,y,scaleFactor));
            }
        }
        return *std::min_element(cells.begin(), cells.end());
    } else {
        return 1.0;
    }
}

void GridLightController::Update( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes, float timeStep ){
    timeElapsed += timeStep;
    if( timeElapsed > trialTime ){
        timeElapsed = 0;
        // std::cout << "(GLC) Changing lights configuration... \n";
        // Check that all boxes in shadow region
        if( GoalObtained(boxes, goals) ){
            std::cout << "[GLC] Goal Obtained!\n";
            TurnOffAllLights();
        } else if( NoBoxOutside(boxes) ){
            Toggle(currentLayer); 
            Toggle(currentLayer-layersActive);
            currentLayer = NextLayer(currentLayer); 
        }
    }
}
// ===> END GRIDLIGHTCONTROLLER clas methods
