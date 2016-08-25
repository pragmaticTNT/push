#ifndef PUSH_H
#define PUSH_H

#include <stdio.h>

#include <math.h>
#include <deque>
#include <string>
#include <vector>

#include <Box2D/Box2D.h>
#include <GLFW/glfw3.h>

static const float EPSILON = 10e-5; // float comparison error
static const float SPAWN = 0.5; // spawn distance away from wall 

typedef enum {
    SHAPE_RECT=0,
    SHAPE_CIRC,
    SHAPE_HEX
} box_shape_t;

class World;

// Abstract Class: pure virtual function Draw
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

class Box : public WorldObject {
    public:
        static float size; 
        b2Body* body;
        
        Box( World& world, float spawnDist, box_shape_t shape = SHAPE_CIRC );
        b2Vec2 GetCenter( void ){
            center = body->GetWorldCenter();
            return center;
        }
        void WhereAmI( void ){ WorldObject::WhereAmI("Box"); }
        void Draw( void );
}; // END Box class

class Goal : public WorldObject {
    public:
        bool filled;
        int index[4];  // WRT grid Light Controller
        float radius;

        /***
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

// Abstract Class: pure virtual function Update
class Robot : public WorldObject{ 
    public:
        static float size;
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

class LightController {
    public:
        std::vector<Light*> lights;
        // TODO: move these to GLC(make default) then LC(make interface)
        uint8_t* pixels; 
        unsigned int pRows, pCols;
        
        LightController();
        ~LightController();
        
        virtual float GetIntensity( float x, float y ){ return EPSILON; }
        virtual bool Update( const std::vector<Goal*>& goals, 
                const std::vector<Box*>& boxes, float timeStep ){}
    protected:
        // -> Update Parameters
        float goalError; 
        bool isFilled;
}; // END LightController Class

class GridLightController : public LightController {
    public:
        GridLightController( 
                const std::vector<Goal*>& goals, 
                const std::deque<std::string>& settings, 
                float worldWidth );
        float GetIntensity( float x, float y );
        bool Update( const std::vector<Goal*>& goals, 
                const std::vector<Box*>& boxes, float timeStep );

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

        float dimWorld, dimCell, trialTime, timeElapsed, totalTime;
        int dimGrid, expand;
        int maxLayer, bdLayer, activeLayers, currentLayer; 
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
        int NeighbourIndex( int cell[2], neighbour_t n );

        // Member functions related to update
        void SetLayers( const std::vector<Goal*>& goals );
        void MarkQuadrants( bool quadrants[8], neighbour_t n );
        void AddLayerIndicies( int layer, int cell[2], neighbour_t n );
        void SetPixels( void );
        void ToggleLayer( int layer );
        void TurnAllLights( bool on );
        int NumBoxesOutside( const std::vector<Box*>& boxes );
        void PrintLayerDistribution( void );
        bool GoalObtained( const std::vector<Box*>& boxes, 
                const std::vector<Goal*>& goals );
}; // END GridLightController class

class World {
    public:  
        b2World* b2world;
        float width, height;
        float avoidLuminance, bufferLuminance;

        World( float width, float height, int robotNum, int boxNum,
                box_shape_t boxShape = SHAPE_CIRC );
        World( const std::string& goalFile );
        ~World();
        float GetLuminance( const b2Vec2& here );
        void Step( float timestep );

    protected:
        int numPatterns, numBoxes, numRobots;
        box_shape_t boxShape;
        // std::vector<std::vector<Goal*> > goalList;
        std::vector<Goal*> goals;
        std::vector<Box*> boxes;
        std::vector<Robot*> robots;
        std::vector<Wall*> groundBody;    // Boundary
        LightController* lightController;

        void ParseFile( const std::string& goalFile, 
                std::deque<std::string>& settings );
        int ParseSettings( const std::deque<std::string>& settings );
        void AddBoundary( void );
        void AddRobots( int numRobots );
        void AddBoxes( int numBoxes, box_shape_t shape );
}; // END World class

class GuiWorld : public World {
    public:
        static bool paused;
        static bool step;
        static int skip;

        GLFWwindow* window;
        int draw_interval;

        GuiWorld( float width, float height, int numRobots, 
                int numBoxes, box_shape_t boxShape = SHAPE_CIRC );
        GuiWorld( const std::string& goalFile );
        ~GuiWorld();
        void Step( float timestep );
        bool RequestShutdown();
    private:
        float tl[3], tr[3], bl[3], br[3];
        void DrawWorld( void );
        void DrawTexture( const uint8_t* pixels, const unsigned int cols, 
                          const unsigned int rows );
}; // END GuiWorld class
#endif
