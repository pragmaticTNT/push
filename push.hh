#ifndef PUSH_H
#define PUSH_H

#include <math.h>
#include <stdio.h>

#include <deque>
#include <exception>
#include <string>
#include <vector>

#include <Box2D/Box2D.h>
#include <GLFW/glfw3.h>

static const float EPSILON = 10e-5; // float comparison error
static const float PORTION_INTERIOR = 6.0; // spawn distance away from wall 

// PUCK SHAPES
typedef enum {
    SHAPE_RECT=0,
    SHAPE_CIRC,
    SHAPE_HEX
} puck_shape_t;

// SIMULATOR COLLISION CATEGORIES
enum class _entityCategory{
    PUCKBOUNDARY = 0x1,
    ROBOTBOUNDARY = 0x2,
    ROBOT = 0x4,
    PUCK = 0x8
};

// SETTINGS FOR THE WORLD CLASSES
struct WorldSettings{
    float width;
    float height;
    int numPucks;
    puck_shape_t puckShape;
    int numRobots;
    int numPatterns;
    float avoidLuminance;
    float bufferLuminance;
};

// SETTINGS FOR THE GRID LIGHT CONTROLLER CLASSES
struct GridLightControllerSettings{
    float goalError;
    float trialTime;
    unsigned int dimGrid;
};

// RESULTS FOR A SIMULATION
struct SimResults{
    float taskCompletionTime;
    float robotMoveDistance;
    // This might be more than one number will put else where
    // float minMaxPuckDistance; 
};

// EXCEPTIONS
class SimException : public std::exception {
    public:
        SimException( std::string level, std::string whichClass, 
                std::string msg, std::string value ) : 
            level(level),
            whichClass(whichClass),
            msg(msg),
            value(value){}
        virtual const char* what( void ) const noexcept{
            std::string tmp = level + " " + whichClass + " " + 
                              msg + ": " + value;
            return tmp.c_str();
        }
    private:
        // Levels: WARN, ERR
        std::string level, whichClass, msg, value;
};

class World;

/***
 * Abstract Class - pure virtual function Draw
 * PURPOSE: Base class of objects that appear in the simulation;
 *          these includes Puck, Goal, Light, Robot and Wall. 
 *          Allows the use of abstraction to draw and find the
 *          center of all such objects.
 ***/
class WorldObject{ 
    protected:
        b2Vec2 center;
        void WhereAmI( std::string objectType ){
            center = GetCenter();
            std::cout << objectType << 
                ": (" << center.x << "," << center.y << ")" << std::endl;
        }
        void DrawDisk( b2Vec2 center, float radius );
        void DrawCircle( b2Vec2 center, float radius );
        void DrawBody( b2Body* b, const float color[3] );

    public:
        WorldObject( float x, float y ) : center( b2Vec2(x,y) ){}
        WorldObject( b2Vec2 center ): center( center ){}
        float SqrDistanceTo( WorldObject& other ){
            b2Vec2 otherCenter = other.GetCenter();
            b2Vec2 myCenter = GetCenter();
            return pow(myCenter.x-otherCenter.x,2.0) + 
                   pow(myCenter.y-otherCenter.y,2.0);
        }
        float SqrDistanceTo( float x, float y ){
            b2Vec2 myCenter = GetCenter();
            return pow(myCenter.x-x,2.0) + pow(myCenter.y-y,2.0);
        }
        virtual b2Vec2 GetCenter( void ){ return center; }
        virtual void Draw( void ) = 0;
}; // END WorldObject class

class Puck : public WorldObject {
    public:
        static float size; 
        b2Body* body;
        
        Puck( World& world, float spawnDist, puck_shape_t shape = SHAPE_CIRC );
        b2Vec2 GetCenter( void ){
            center = body->GetWorldCenter();
            return center;
        }
        void WhereAmI( void ){ WorldObject::WhereAmI("Puck"); }
        void Draw( void );
}; // END Puck class

class Goal : public WorldObject {
    public:
        bool filled;    // Is the goal filled?
        int index[4];   // WRT grid Light Controller
        float radius;

        /***
         * QUADRANTS:
         *  1 | 0
         * -- C --
         *  2 | 3
         ***/
        Goal( float x, float y, float size, 
              int index0 = 0, int index1 = 0,
              int index2 = 0, int index3 = 0) : 
            WorldObject(x,y), radius(size/2), filled(false), 
            index{index0, index1, index2, index3} {}
        ~Goal(){}
        void WhereAmI( void ){ WorldObject::WhereAmI("Goal"); }
        void Draw( void );
}; // END Goal class

class Light : public WorldObject {
    public:
        static float cosCritAngle, radius, HEIGHT;
        bool on;
        int layer;

        Light( float x, float y ) : 
            WorldObject(x,y), on(false), layer(-1){}
        Light( b2Vec2 center ) : 
            WorldObject(center), on(false), layer(-1){}
        ~Light(){}

        float GetIntensity( float x, float y ){
            float cosAngle = cos( atan2( sqrt(SqrDistanceTo(x,y)), HEIGHT) );
            return on && (cosAngle > cosCritAngle + EPSILON) ? 
                1.0 - (1.0 - cosAngle)/(1.0 - cosCritAngle) : 
                EPSILON/10;
        }
        void SetCenter( float x, float y ){ center = b2Vec2(x,y); }
        void SetCenter( b2Vec2 here ){ center = here; }
        void WhereAmI( void ){ WorldObject::WhereAmI("Light"); }
        void Draw( void );
}; // END Light class

/***
 * Abstract Class: pure virtual function Update
 ***/
class Robot : public WorldObject{ 
    public:
        static float size;
        float moveAmount;   // Total distance traved by robot
        b2Body *body, *bumper;
        b2PrismaticJoint* joint;
        
        Robot( World& world, float x, float y, float angle);      
        b2Vec2 GetCenter( void ){
            center = body->GetWorldCenter();
            return center;
        }
        void WhereAmI( void ) { WorldObject::WhereAmI("Robot"); }
        void Draw( void );
        virtual void Update( float timestep, World& world ) = 0; 
}; // END Robot class

class Wall : public WorldObject {
    public:
        b2Body* body;
        
        Wall( b2Body* body, b2Vec3& dim );
        b2Vec2 GetCenter( void ){ return center; }
        void Draw( void );
}; // END Wall class

class Pusher : public Robot {
    private:
        typedef enum {
            S_PUSH = 0,
            S_BACKUP,
            S_TURN,
            S_DEAD,
            S_COUNT
        } control_state_t;

        static const float PUSH, BACKUP, TURNMAX, DEAD;
        static const float SPEEDX, SPEEDA;
        static const float maxspeedx, maxspeeda;
        static const b2Vec2 frontLightSensor;
        static const b2Vec2 backLightSensor;

        float timeleft;
        control_state_t state;
        float speedx, speeda;
        int turnRight;

        bool isBumperPressed( void ) const; 
        void SetSpeed( float x, float y, float angle );
        b2Vec2 GetPhotocell( const b2Vec2& lightSensor ) const{
            return body->GetWorldPoint(lightSensor);
        }

    public:
        Pusher( World& world, float spawnDist );
        void Update( float timestep, World& world );
}; // END Pusher class

class GridLightController {
    public:
        float totalTime;
        GridLightControllerSettings* glcSet;

        // Used for drawing the lights with gradation
        unsigned int pRows, pCols;
        uint8_t* pixels; 

        GridLightController( int controlVal,
                GridLightControllerSettings& glcSet, 
                const std::vector<Goal*>& goals, float worldWidth );
        ~GridLightController();
        /***
         * PURPOSE: Get the light intensity in a particular location.
         * INPUT:   x and y location in the grid
         * OUTPUT:  (float) light luminosity
         ***/
        float GetIntensity( float x, float y );
        bool Update( int controlVal, const std::vector<Goal*>& goals, 
                const std::vector<Puck*>& pucks, float timeStep,
                std::vector<float>& puckDist );
    private:
        /***
         *  Enumerates all the neighbours of the cell CC.
         *  Letters specifies the position of the n-bour cell:
         *  first is one of top(T), center(C), bottom(B) and the
         *  second is one of left(L), center(C), right(R). 
         *  Cell numberings start at 0 with TR and goes CCW. 
         *   ---- ---- ----
         *  | TL | TC | TR | <- Start Here
         *   ---- ---- ----
         *  | CL | CC | CR |
         *   ---- ---- ----
         *  | BL | BC | BR |
         *   ---- ---- ----
         ***/
        typedef enum {
           TR=0, TC, TL, CL, BL, BC, BR, CR, NBOUR_NUM
        } neighbour_t;

        int dimGrid;
        float dimWorld, dimCell;
        int maxLayer, bdLayer, activeLayers, currentLayer; 
        int expand;     // How shadow zone changes
        float timeElapsed;

        std::vector<Light*> lights;
        std::vector< std::vector<int> > layerIndicies;

        // Member functions related to grid element access
        int RowColToIndex( int row, int col ){
            // ASSERT: 0 <= index <= dimGrid -1
            return 
                ( row < 0 || row > dimGrid-1 || 
                  col < 0 || col > dimGrid-1 ) ? 
                -1 :
                row*dimGrid + col;
        }
        void IndexToRowCol( int index, int RowCol[2] ){
            RowCol[0] = index / dimGrid;
            RowCol[1] = index % dimGrid;
        }
        b2Vec2 GetLightCenter( int row, int col ){
            float x = col*dimCell + dimCell/2.0;
            float y = row*dimCell + dimCell/2.0;
            return b2Vec2(x, y);
        }
        void PointInCell( float x, float y, int cell[2] ){
            cell[0] = floor(y/dimCell);  
            cell[1] = floor(x/dimCell);
        }
        /***
         * PURPOSE: returns index of n-bour if exists otherwise 
         *          index of cell 
         * INPUT:   cell (array<int>) - row and column index of 
         *                              the cell
         *          n (neighbour_t) - which neighbour to index
         * OUTPUT:  index of neighbour (int)
         ***/
        int NeighbourIndex( int cell[2], neighbour_t n );

        // Member functions related to update
        /***
         * PURPOSE: Goes through each layer outside the goal files,
         *          starting from the closest layer, and determine
         *          which cell are in which layers.
         * INPUT:   goals (vector<Goal*>) - vector of goals 
         * OUTPUT:  (None) 
         ***/
        void SetLayers( const std::vector<Goal*>& goals );
        /***
         * PURPOSE: (Helper Function) used in "SetLayers" to help
         *          determine which quadrants to mark. This depends
         *          on the location of the neighbour goal cells.
         * INPUT:   quadrants (array<boo>) - which neighbour cells are 
         *          occupied by goal cells
         * OUTPUT:  (None)
         ***/
        void MarkQuadrants( bool quadrants[8], neighbour_t n );
        void AddLayerIndicies( int layer, int cell[2], neighbour_t n );
        /***
         * PURPOSE: sets the values for the array used by the 
         *          OpenGL textures 
         * INPUT:   (None)
         * OUTPUT:  (None)
         ***/
        void SetPixels( void );
        void ToggleLayer( int layer );
        void TurnAllLights( bool on );
        int NumPucksOutside( const std::vector<Puck*>& pucks );
        void PrintLayerDistribution( void );
        /***
         * PURPOSE: Determines if the goal has been obtained by
         *          checking the minimum distance from a puck of 
         *          each goal. SIDE EFFECT: records the distribution
         *          of the pucks.
         * INPUT:   pucks (vector<Puck>) - all pucks
         *          goals (vector<Goals>) - all goals
         *          controlVal(INT) - if we the simulator is 
         *                            recording puck distributions
         *                            (controlVal == 3) then 
         *                            store these
         *          puckDist (vector<float>) - where to store the 
         *                                     puck distributions
         * OUTPUT:  (bool) if the goal has been obtained or not
         ***/
        bool GoalObtained( const std::vector<Puck*>& pucks, 
                const std::vector<Goal*>& goals, int controlVal,
                std::vector<float>& puckDist );
}; // END GridLightController class

class World {
    public:  
        static bool showGui;
        WorldSettings* worldSet;
        b2World* b2world;

        World( int controlVal, WorldSettings& worldSet,
               GridLightControllerSettings& glcSet,
               const std::vector<Goal*>& goals );
        ~World();
        float GetLuminance( const b2Vec2& here );
        bool Step( int controlVal, float timestep,
                   const std::vector<Goal*>& goals,
                   SimResults& results,
                   std::vector<float>& puckDist );

    protected:
        std::vector<Puck*> pucks;
        std::vector<Robot*> robots;
        std::vector<Wall*> groundBody;    // Boundary
        GridLightController glc;
        float spawn;    // Distance spaw from boundary

        void AddBoundary( int controlVal );
        void AddRobots( void );
        void AddPucks( void );
        float GetTotalRobotMovement( void );
}; // END World class

class GuiWorld : public World {
    public:
        static bool paused;
        static bool step;
        static int skip;

        GLFWwindow* window;
        int draw_interval;

        GuiWorld( int controlVal, WorldSettings& worldSet, 
                  GridLightControllerSettings& glcSet,
                  const std::vector<Goal*>& goals );
        ~GuiWorld();
        bool Step( int controlVal, float timestep, 
                   const std::vector<Goal*>& goals,
                   SimResults& results,
                   std::vector<float>& puckDist);
        bool RequestShutdown();
    private:
        // Deliminates corners of the world: t-top, b-bottom, r-right, l-left
        float tl[3], tr[3], bl[3], br[3]; 
        void DrawTexture( const uint8_t* pixels, unsigned int cols, 
                unsigned int rows );
}; // END GuiWorld class

#endif
