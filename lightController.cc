#include <iostream>

#include "push.hh"

// Specifies composition of pixel data:
//      1: GL_RED (element is only red component, GB both 0, alpha is 1)
//      3: GL_RGB (element is RGB triple, alpha set to 1) 
//      4: GL_RGBA (element composed of four components, clamp [0,1])
// FORMAT must match size of DARK and LIGHT arrays
const int FORMAT = 3;
const uint8_t DARK[] = { 211, 211, 211 };
const uint8_t LIGHT[] = { 255, 255, 128 };
const int DELTA[] = { LIGHT[0]-DARK[0], LIGHT[1]-DARK[1], LIGHT[2]-DARK[2] };

// ===> LIGHTCONTROLLER class methods
LightController::LightController() : 
    goalError(0),
    isFilled(false),
    pixels(NULL),
    pRows(0),
    pCols(0)
{ 
    Light::radius = 1.0/sqrt(3);
    Light::cosCritAngle = cos(M_PI/6);
    pixels = new uint8_t[1];
}

LightController::~LightController(){
    for( auto light : lights )
        delete light;
    delete pixels;
}
// ===> END LIGHTCONTROLLER class methods


// ===> GRID LIGHT CONTROLLER class methods
/***
 * Let "n" be dimGrid. The light grid indexed as follows:
 *
 *  -------- ----------       --------
 * | n(n-1) | n(n-1)+1 | ... | n(n)-1 |
 *  -------- ----------       --------
 *                       ... 
 *  -------- ----------       --------
 * |   n    |   n+1    | ... |  2n-1  | 
 *  -------- ----------       --------
 * |   0    |    1     | ... |  n-1   | 
 *  -------- ----------       --------
 *
 ***/
GridLightController::GridLightController( 
        const std::vector<Goal*>& goals, 
        const std::deque<std::string>& settings, 
        float worldWidth ) : 
    LightController(), 
    timeElapsed(0),
    stepElapsed(0)
{
    size_t numParams = settings.size();

    goalError = numParams > 0 ? std::stof(settings[0]) : 0.2;
    trialTime = numParams > 1 ? std::stof(settings[1]) : 500;
    dimGrid = numParams > 2 ? std::stoi(settings[2]) : 11;
    dimWorld = worldWidth;
    dimCell = worldWidth / dimGrid;

    Light::radius = dimCell*(sqrt(2)+1)/4.0;
    Light::cosCritAngle = cos( atan2(Light::radius, Light::HEIGHT) );
 
    std::cout << "[GLC] Setting Goals...\n";
    for( int row = 0; row < dimGrid; ++row )
        for( int col = 0; col < dimGrid; ++col )
            lights.push_back( new Light(GetLightCenter(row, col)) );

    activeLayers = 4;   // Where should we set this?
    SetLayers(goals);
    // Set maxLayers
    for( Light* light: lights ){
        if( light->layer > maxLayers )
            maxLayers = light->layer;
        else if( light->layer < 0 )
            std::cout << "[ERR] (GLC) Layer not set.\n";
    }
    // PrintLayerDistribution();

    currentLayer = maxLayers;
    stepTime = 5*maxLayers;
    // std::cout << "Max Layer: " << maxLayers << "\n";
    InitializeLayers(); // This doesn't do much yet

    pRows = 100;
    pCols = 100;
    // std::cout << pRows << " " << pCols << '\n';
    pixels = new uint8_t[pRows * pCols * FORMAT];   // setup array
    SetPixels();
}

// NOTE: processed cells are marked by turning the light ON!
void GridLightController::SetLayers( const std::vector<Goal*>& goals ){
    // Setting goal lights layer 0
    std::vector<int> goalLayer;
    layerIndicies.push_back(goalLayer);
    for( Goal* goal : goals ){
        layerIndicies[0].push_back(goal->index);
        lights[goal->index]->layer = 0;
    }

    // Setting remaining layers
    for( int currLayer = 1; currLayer < dimGrid; ++currLayer ){
        std::vector<int> newLayer;
        layerIndicies.push_back(newLayer);
        for( Goal* goal : goals ){
            int nBourIndex;
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
                if( lights[nBourIndex]->layer == 0 ) 
                    MarkQuadrants( quadrants, n ); 
            }
            for( int i = 0; i < NBOUR_NUM; ++i ){
                neighbour_t n = static_cast<neighbour_t>(i);
                if( quadrants[i] )
                    AddLayerIndicies( currLayer, cell, n );
            }
        } 
    }
}

// RETURN: index of n-bour if exists otherwise index of cell 
int GridLightController::NeighbourIndex( int cell[2], neighbour_t n ){
    int row, col;
    switch(n){
        case TR: row = 1; col = 1; break; 
        case TC: row = 1; col = 0; break;
        case TL: row = 1; col = -1; break;
        case CL: row = 0; col = -1; break;
        case BL: row = -1; col = -1; break;
        case BC: row = -1; col = 0; break;
        case BR: row = -1; col = 1; break;
        case CR: row = 0; col = 1; break;
    }
    return RowColToIndex(cell[0]+row, cell[1]+col);
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

void GridLightController::AddLayerIndicies( int currLayer, int cell[2], 
        neighbour_t n ){
    bool oneCell = true;
    int rowMod = 0, colMod = 0, index = 0;

    // If only oneCell then changes the setting of a cell at a distance
    // "currLayer" away. Otherwise changes an entire diagonal.
    switch( n ){
        case TR: oneCell = false; rowMod = 1; colMod = 1; break; 
        case TC: oneCell = true; rowMod = currLayer; colMod = 0; break;
        case TL: oneCell = false; rowMod = 1; colMod = -1; break;
        case CL: oneCell = true; rowMod = 0; colMod = -currLayer; break;
        case BL: oneCell = false; rowMod = -1; colMod = -1; break;
        case BC: oneCell = true; rowMod = -currLayer; colMod = 0; break;
        case BR: oneCell = false; rowMod = -1; colMod = 1; break;
        case CR: oneCell = true; rowMod = 0; colMod = currLayer; break;
    }

    if( oneCell ){
        index = RowColToIndex(cell[0]+rowMod, cell[1]+colMod);
        if( index >= 0 and lights[index]->layer < 0 ){ // i.e. not set
            layerIndicies[currLayer].push_back(index);
            lights[index]->layer = currLayer;
        }
    } else {
        for( int i = 1; i < currLayer; ++i ){
            index = RowColToIndex( 
                        cell[0]+rowMod*i, 
                        cell[1]+colMod*(currLayer-i) 
                    );      
            if( index >= 0 and lights[index]->layer < 0 ){
                layerIndicies[currLayer].push_back(index);
                lights[index]->layer = currLayer;
            }
        }
    }
}

void GridLightController::PrintLayerDistribution( void ){
    char space = ' ';
    std::cout << "\n";
    for( int row = dimGrid-1; row >= 0; --row ){
        for( int col = 0; col < dimGrid; ++col )
            std::cout << lights[RowColToIndex(row, col)]->layer << space;
        std::cout << "\n";
    }
    std::cout << "\n";
}

void GridLightController::InitializeLayers( void ){
    int tempLayer = currentLayer;
    // for( int i = 0; i < activeLayers; ++i ){
    //     Toggle(tempLayer);
    //     tempLayer = NextLayer(tempLayer);
    // }
    for( int i = 1; i < 3; ++i ){
        Toggle(i);
    }
}

void GridLightController::SetPixels( void ){
    unsigned int index;
    float x, y, scale;
    float rowWidth = dimWorld / pRows;
    float colWidth = dimWorld / pCols;
    for( int row = 0; row < pRows; ++row ){
        for( int col = 0; col < pCols; ++col ){
            x = std::min( dimWorld-EPSILON, colWidth * col );
            y = std::min( dimWorld-EPSILON, rowWidth * row );
            index = FORMAT*((pRows-row-1)*pRows + col);
            for( int i = 0; i < FORMAT; ++i ){
                pixels[index+i] = DARK[i] + DELTA[i]*GetIntensity(x, y); 
            }
        } 
    }
}

bool GridLightController::BoxOutside( const std::vector<Box*>& boxes ){
    int lightIndex, layer;
    int cell[2];

    for( Box* box : boxes ){
        b2Vec2 boxCenter = box->GetCenter();
        PointInCell( boxCenter.x, boxCenter.y, cell );
        lightIndex = RowColToIndex( cell[0], cell[1] );
        layer = lights[lightIndex]->layer;
        if( lightIndex >= 0 and layer-1 >= currentLayer-activeLayers ){
            // printf("Box center: (%.4f, %.4f)\n", boxCenter.x, boxCenter.y);
            // box->WhereAmI();
            // printf("Row: %i Col: %i Layer: %i CurrentLayer: %i\n", 
            //         cell[0], cell[1], layer, currentLayer);
            return true;
        } 
    }
    return false;
}

void GridLightController::Toggle( int layer ){
    if( layer > 0){
        // std::cout << "Toggling layer: " << layer << std::endl;
        for( int index : layerIndicies[layer] )
            lights[index]->on = !lights[index]->on;
    } else {
        Toggle( layer+dimGrid-1 ); // Layers wrap arround with modular arithmetics
    }
}

void GridLightController::TurnAllLights( bool on ){
    for( Light* light : lights ){
        light->on = on;
    }
}

bool GridLightController::GoalObtained( const std::vector<Box*>& boxes, 
        const std::vector<Goal*>& goals ){
    bool goalFilled = false;
    for( Goal* goal : goals ){
        for ( Box* box : boxes ){
            if( sqrt(box->SqrDistanceTo(*goal)) < goalError )
                goalFilled = true;
        }
        if( not goalFilled ){
            return false;
        }
        goalFilled = false;
    }
    return true;
}

float GridLightController::GetIntensity( float x, float y ){
    int cell[2] = {};
    int index, cellIndex;
    float luminosity;
    neighbour_t nBour;

    PointInCell( x, y, cell );
    cellIndex = RowColToIndex(cell[0], cell[1]);

    // printf("Row: %i Col: %i Index: %i\n", cell[0], cell[1], cellIndex);
    if( cellIndex >= 0 ){
        luminosity = lights[cellIndex]->GetIntensity(x,y);

        // Check intensities of all 8 adjacent cells
        for( int i = 0; i < NBOUR_NUM; ++i){
            nBour = static_cast<neighbour_t>(i);
            index = NeighbourIndex( cell, nBour );
            if( index >= 0 ){
                luminosity += lights[index]->GetIntensity(x,y);
            }
            //std::cout << "N-bour " << i << ": " << luminosity << '\n';
        }
        return luminosity;
    } else {
        return EPSILON;
    }
}

void GridLightController::Update( const std::vector<Goal*>& goals, 
        const std::vector<Box*>& boxes, float timeStep ){
    timeElapsed += timeStep;
    if( timeElapsed > trialTime ){
        bool boxOutside = BoxOutside(boxes); 
        timeElapsed = 0;
        // std::cout << "(GLC) Changing lights configuration... \n";
        // Check that all boxes in shadow region
        if( GoalObtained(boxes, goals) ){
            std::cout << "[GLC] Goal Obtained!\n";
            // TurnAllLights(false);
        } else if( !boxOutside &&  currentLayer-activeLayers > 0 ){
            // Toggle(currentLayer); 
            // Toggle(currentLayer-activeLayers);
            // currentLayer = NextLayer(currentLayer); 
        } else if( !boxOutside ){
            stepElapsed += 1;
            std::cout << "Step: " << stepElapsed << " Box Outside!\n";
        }
        if( stepElapsed > stepTime ){
            stepElapsed = 0;
            TurnAllLights(false);
            currentLayer = maxLayers;
            InitializeLayers();
        }
        SetPixels();
    }
}
// ===> END GRIDLIGHTCONTROLLER clas methods
