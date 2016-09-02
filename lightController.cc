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
GridLightController::GridLightController( int controlVal, 
        GridLightControllerSettings& glcSet, 
        const std::vector<Goal*>& goals, float dimWorld ) : 
    pixels(NULL),
    pRows(0),
    pCols(0),
    dimGrid(glcSet.dimGrid),
    dimWorld(dimWorld),
    dimCell(dimWorld/glcSet.dimGrid),
    maxLayer(0),
    bdLayer(100),
    expand(-1),
    timeElapsed(0),
    totalTime(0),
    glcSet(&glcSet)
{
    Light::radius = dimCell;
    Light::cosCritAngle = cos( atan2(Light::radius, Light::HEIGHT) );
 
    // std::cout << "[GLC] Setting Goals...\n";
    for( int row = 0; row < dimGrid; ++row )
        for( int col = 0; col < dimGrid; ++col )
            lights.push_back( new Light(GetLightCenter(row, col)) );

    SetLayers(goals);
    // printf("[GLC] Layers - Max: %i Bd: %i Active: %i\n", maxLayer, bdLayer, activeLayers);
    if( controlVal == 0 ){
        for( int layer = maxLayer; layer > 0; --layer )
            ToggleLayer(layer);
    } else {
        ToggleLayer( currentLayer );
        --currentLayer;
    }
    // PrintLayerDistribution();

    // if( World::showGui ){
    pRows = 100;
    pCols = 100;
    // std::cout << pRows << " " << pCols << '\n';
    pixels = new uint8_t[pRows * pCols * FORMAT];
    SetPixels();
    // }
}

GridLightController::~GridLightController(){
    for( auto light : lights )
        delete light;
    // if( World::showGui )
    delete pixels;
}

// NOTE: processed cells are marked by turning the light ON!
void GridLightController::SetLayers( const std::vector<Goal*>& goals ){
    // Setting goal lights layer 0
    int nbourCells = 4;
    std::vector<int> goalLayer;
    layerIndicies.push_back(goalLayer);
    for( Goal* goal : goals ){
        for( int i = 0; i < nbourCells; ++i ){
            layerIndicies[0].push_back((goal->index)[i]);
            lights[(goal->index)[i]]->layer = 0;
        }
    }
    for( int i = 0; i < dimGrid; ++i ){
            lights[RowColToIndex(i, 0)]->layer = INT_MAX;
            lights[RowColToIndex(0, i)]->layer = INT_MAX;
            lights[RowColToIndex(i, dimGrid-1)]->layer = INT_MAX;
            lights[RowColToIndex(dimGrid-1, i)]->layer = INT_MAX;
    }

    // Setting remaining layers
    for( int layer = 1; layer < dimGrid; ++layer ){
        std::vector<int> newLayer;
        layerIndicies.push_back(newLayer);
        for( Goal* goal : goals ){
            for( int i = 0; i < nbourCells; ++i ){
                int nBourIndex;
                int cell[2] = {0, 0};
                bool quadrants[8] = {
                    true, true, true, true, 
                    true, true, true, true 
                };
                
                IndexToRowCol( (goal->index)[i], cell );
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
                        AddLayerIndicies( layer, cell, n );
                }
            }
        } 
    }

    // Set maxLayer 
    int layer;
    for( int row = 1; row < dimGrid-1; ++row ){
        for ( int col = 1; col < dimGrid-1; ++col ){
            layer = lights[RowColToIndex(row, col)]->layer;  
            if( layer > maxLayer )
                maxLayer = layer;
            else if( layer < 0 )
                std::cout << "[ERR] (GLC) Layer not set.\n";
            else if( (row == 0 or row == dimGrid-1 or 
                      col == 0 or col == dimGrid-1) and layer < bdLayer )
                bdLayer = layer;
        }
    }

    // Set Edges
    for( int i = 0; i < dimGrid; ++i ){
        int bottom = RowColToIndex(i, 0);
        int top = RowColToIndex(i, dimGrid-1); 
        int left = RowColToIndex(0, i);
        int right = RowColToIndex(dimGrid-1, i);
        lights[bottom]->layer = maxLayer;
        lights[top]->layer = maxLayer;
        lights[left]->layer = maxLayer;
        lights[right]->layer = maxLayer;
        layerIndicies[maxLayer].push_back(bottom);
        layerIndicies[maxLayer].push_back(top);
        layerIndicies[maxLayer].push_back(left);
        layerIndicies[maxLayer].push_back(right);
    }
    currentLayer = maxLayer;

    // Where should we set this?
    // activeLayers = std::min(4, maxLayer-1); 
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

void GridLightController::AddLayerIndicies( int layer, int cell[2], 
        neighbour_t n ){
    bool oneCell = true;
    int rowMod = 0, colMod = 0, index = 0;

    // If only oneCell then changes the setting of a cell at a distance
    // "layer" away. Otherwise changes an entire diagonal.
    switch( n ){
        case TR: oneCell = false; rowMod = 1; colMod = 1; break; 
        case TC: oneCell = true; rowMod = layer; colMod = 0; break;
        case TL: oneCell = false; rowMod = 1; colMod = -1; break;
        case CL: oneCell = true; rowMod = 0; colMod = -layer; break;
        case BL: oneCell = false; rowMod = -1; colMod = -1; break;
        case BC: oneCell = true; rowMod = -layer; colMod = 0; break;
        case BR: oneCell = false; rowMod = -1; colMod = 1; break;
        case CR: oneCell = true; rowMod = 0; colMod = layer; break;
    }

    if( oneCell ){
        index = RowColToIndex(cell[0]+rowMod, cell[1]+colMod);
        if( index >= 0 and lights[index]->layer < 0 ){ // i.e. not set
            layerIndicies[layer].push_back(index);
            lights[index]->layer = layer;
        }
    } else {
        for( int i = 1; i < layer; ++i ){
            index = RowColToIndex( 
                        cell[0]+rowMod*i, 
                        cell[1]+colMod*(layer-i) 
                    );      
            if( index >= 0 and lights[index]->layer < 0 ){
                layerIndicies[layer].push_back(index);
                lights[index]->layer = layer;
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
                // Intensity clamped to [0,1] anything > 1 is precieved as 1
                pixels[index+i] = DARK[i] + 
                    DELTA[i]*std::min(float(1.0),GetIntensity(x, y)); 
            }
        } 
    }
}

int GridLightController::NumBoxesOutside( const std::vector<Box*>& boxes ){
    int lightIndex, layer;
    int cell[2];
    int numBoxes = 0;

    for( Box* box : boxes ){
        b2Vec2 boxCenter = box->GetCenter();
        PointInCell( boxCenter.x, boxCenter.y, cell );
        lightIndex = RowColToIndex( cell[0], cell[1] );
        layer = lights[lightIndex]->layer;
        if( lightIndex >= 0 and layer <= currentLayer ){
            // printf("Box center: (%.4f, %.4f)\n", boxCenter.x, boxCenter.y);
            // box->WhereAmI();
            // printf("Row: %i Col: %i Layer: %i CurrentLayer: %i\n", 
            //         cell[0], cell[1], layer, currentLayer);
            ++numBoxes;
        } 
    }
    return numBoxes;
}

void GridLightController::ToggleLayer( int layer ){
    // Layers wrap arround with modular arithmetics
    layer = layer > 0 ? layer : layer + dimGrid-1;
    // std::cout << "Toggling layer: " << layer << std::endl;
    for( int index : layerIndicies[layer] )
        lights[index]->on = !lights[index]->on;
}

void GridLightController::TurnAllLights( bool on ){
    for( Light* light : lights ){
        light->on = on;
    }
}

bool GridLightController::GoalObtained( const std::vector<Box*>& boxes, 
        const std::vector<Goal*>& goals, int controlVal, 
        std::vector<float>& boxDist ){
    bool goalFilled = false;
    size_t boxesNearGoals = 0;
    int count = 0;
    // TODO: Change this from O(bg) -> O(blog(g)) --- b: NumBoxes,
    //       g: NumGoals --- using voronoi diagrams and preprocessing
    for ( Box* box : boxes ){
        float minErr = 100;
        for( Goal* goal : goals ){
            float err = sqrt(box->SqrDistanceTo(*goal));
            if( err < glcSet->goalError )
                ++boxesNearGoals;
            minErr = std::min(minErr, err);
        }
        if( controlVal == 3 and count % 10 == 0 ){
            boxDist.push_back(minErr);
        }
        ++count;
    }
    for( Goal* goal : goals ){
        float minErr = 100;
        for ( Box* box : boxes ){
            float err = sqrt(box->SqrDistanceTo(*goal));
            if( err < glcSet->goalError ){
                goalFilled = true;
                break;
            }
        }
        if( !goalFilled )
            return false;
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
            if( index >= 0 and lights[index]->on ){
                luminosity += lights[index]->GetIntensity(x,y);
            }
        }
        return luminosity;
    } else {
        return EPSILON;
    }
}

bool GridLightController::Update( int controlVal, 
        const std::vector<Goal*>& goals, 
        const std::vector<Box*>& boxes, 
        float timeStep, std::vector<float>& boxDist){
    totalTime += timeStep;
    timeElapsed += timeStep;
    if( timeElapsed > glcSet->trialTime ){
        timeElapsed = 0;
        // Check that all boxes in shadow region
        if( controlVal == 3 ){
            std::cout << "@ time: " << totalTime << "\n"; 
            boxDist.push_back( totalTime );
        }
        if( GoalObtained(boxes, goals, controlVal, boxDist) ){ 
            // std::cout << "[GLC] Goal Obtained!\n";
            std::cout << "===  Total Time Elapsed: " << totalTime << "\n";
            return false;
        } else if( controlVal != 0 and 
                   NumBoxesOutside(boxes) >= goals.size() and 
                   currentLayer > 0 ){
            ToggleLayer(currentLayer);
            currentLayer += expand;
            // if( currentLayer == 0 ){
            //     expand = 1;
            //     ++currentLayer;
            // } else if( currentLayer == bdLayer ){
            //     expand = -1;
            //     --currentLayer; 
            // }
            // if( World::showGui )
            SetPixels();
        }
    } else if( totalTime > 10000 ){ // Hard Stop
        return false;
    }
    return true;
}
// ===> END GRIDLIGHTCONTROLLER clas methods
