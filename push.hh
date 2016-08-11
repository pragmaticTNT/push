#ifndef PUSH_H
#define PUSH_H

#include <math.h>
#include <queue>
#include <string>
#include <vector>

#include <Box2D/Box2D.h>
#include <GLFW/glfw3.h>

static const float EPSILON = 10e-4; // float comparison error
static const float SPAWN = 0.5; // spawn distance away from wall 

typedef enum {
    SHAPE_RECT=0,
    SHAPE_CIRC,
    SHAPE_HEX
} box_shape_t;

class World;

class WorldObject {
    protected:
        b2Vec2 center;
        void WhereAmI( std::string objectType ){
            std::cout << objectType << ": (" << center.x << "," << center.y << ")" << std::endl;
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
            return pow(myCenter.x-otherCenter.x,2.0) + pow(myCenter.y-otherCenter.y,2.0);
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
            center = (body->GetWorldCenter());
            return center;
        }
        void WhereAmI( void ){ WorldObject::WhereAmI("Box"); }
        void Draw( void );
}; // END Box class

class Goal : public WorldObject {
    public:
        int index;
        float radius;  // r: circle radius or sidelen/2 for square
        bool filled;

        Goal( float x, float y, float radius, int index = 0 ) : WorldObject(x,y), radius(radius), filled(false), index(index){}
        ~Goal(){}
        void WhereAmI( void ){ WorldObject::WhereAmI("Goal"); }
        void Draw( void );
}; // END Goal class

class Light : public WorldObject {
    public:
        static float radiusSmall, radiusLarge;
        int layer;
        bool on, atGoal;

        Light( float x, float y ) : WorldObject(x,y), on(false), atGoal(false), layer(0){}
        Light( b2Vec2 center ) : WorldObject(center), on(false), atGoal(false), layer(0){}
        ~Light(){}

        float GetIntensity( float x, float y, float scaleFactor ){
            float light = 1-1.0/(1+scaleFactor*SqrDistanceTo(x,y)); 
            return on ? 1.0 : light;
        }
        void SetCenter( float x, float y ){ center = b2Vec2(x,y); }
        void SetCenter( b2Vec2 here ){ center = here; }
        void WhereAmI( void ){ WorldObject::WhereAmI("Light"); }
        void Draw( void );
}; // END Light class

class Robot : public WorldObject{
    protected:
        // get sensor data
        bool isBumperPressed( void );
        float lightIntensity;
        
        // send commands
        void SetSpeed( float x, float y, float angle );

    public:
        static float size;
        b2Body *body, *bumper;
        b2PrismaticJoint* joint;
        
        Robot( World& world, float x, float y, float angle);      
        b2Vec2 GetCenter( void ){
            center = (body->GetWorldCenter());
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
            S_BACKUP_LONG,
            S_BACKUP_SHORT,
            S_TURN,
            S_DEAD,
            S_COUNT
        } control_state_t;

        static const float PUSH, BACKUP, TURNMAX;
        static const float SPEEDX, SPEEDA;
        static const float DEAD;
        static const float maxspeedx, maxspeeda;

        float timeleft;
        control_state_t state;
        float speedx, speeda;
        int turnRight;

    public:
    // constructor
    Pusher( World& world, float spawnDist );
    void Update( float timestep, World& world );
}; // END Pusher class

class LightController {
    protected:
        float avoidIntensity;   
        float bufferIntensity;  

        // -> Intensity Related
        float scaleFactor;          // ORDER DEPENDENCY 
        float radiusSmall; 
        float radiusLarge;          // ORDER DEPENDENCY

        // -> Update Parameters
        float goalError; 
        bool isFilled;

        float GetScaleFactor( float radius ){
            return ((1.0/(1-avoidIntensity))-1)/pow(radius,2.0);
        }
        float GetRadiusLarge(){
            return sqrt(((1.0/(1-bufferIntensity))-1)/scaleFactor);
        }

    public:
        std::vector<Light*> lights;
        
        LightController( float avoidIntensity, float bufferIntensity, float radiusSmall );
        ~LightController();
        
        virtual float GetIntensity( float x, float y ){ return 1.0; }
        virtual void Update( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes, float timeStep ){}
}; // END LightController Class

class RadialLightController: public LightController {
    private:
        float radiusInit, timeElapsed;
        float growRate, trialTime, patternTime, maxRadius;
        bool repeatPattern, startScramble, scramble;

        float UpdateGoalsInfo( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes );

    public:
        RadialLightController( const std::vector<Goal*>& goals, std::queue<std::string>& settings, float avoidIntensity = 0.2, float bufferIntensity = 0.4, float radiusSmall = 0.5);
        float GetIntensity( float x, float y );
        void Update( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes );
        void Update( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes, float timeStep );
}; // END RadialLightController class 

class GridLightController : public LightController {
    private:
        float dimCell, trialTime, timeElapsed;
        int dimGrid, maxLayer, layersActive, currentLayer;
        std::vector< std::vector<int> > layerIndicies;

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

        int RowColToIndex( int row, int col ){
            // ASSERT: 0 <= index <= dimGric -1
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
            cell[0] = std::max(round(y/dimCell), 0.0);
            cell[1] = std::max(round(x/dimCell), 0.0);
        }
        int NextLayer( int now ){
            return now > 1 ? now-1 : dimGrid-1; 
        }

        int NeighbourIndex( int cell[2], neighbour_t n );
        bool NoBoxOutside( const std::vector<Box*>& boxes );
        void PreprocessLayers( const std::vector<Goal*>& goals );
        void MarkQuadrants( bool quadrants[8], neighbour_t n );
        void AddLayerIndicies( int layer, int cell[2], neighbour_t n );
        void Toggle( int layer );
        void SeeLayerDistribution( void );
        void TurnOffAllLights( void );
        bool GoalObtained( const std::vector<Box*>& boxes, const std::vector<Goal*>& goals );

    public:
        GridLightController( const std::vector<Goal*>& goals, std::queue<std::string>& settings, float avoidIntensity, float bufferIntensity, float cellWidth );
        float GetIntensity( float x, float y );
        void Update( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes, float timeStep );
}; // END GridLightController class

class World {
    private:
        typedef enum {
            LC_RADIAL=0,
            LC_GRID
        } light_controller_t;

    protected:
        int numPatterns;
        // std::vector<std::vector<Goal*> > goalList;
        std::vector<Goal*> goals;
        std::vector<Box*> boxes;
        std::vector<Robot*> robots;
        std::vector<Wall*> groundBody;    // Boundary
        LightController* lightController;

        void ParseFile( const std::string& goalFile, std::queue<std::string>& settings );
        void AddBoundary( void );
        void AddRobots( int numRobots );
        void AddBoxes( int numBoxes, box_shape_t shape );

    public:  
        b2World* b2world;
        float width, height;
        float lightAvoidIntensity, lightBufferIntensity; // Small and Large circle radii respectively 

        World( float width, float height, int robotNum, int boxNum, box_shape_t boxShape = SHAPE_CIRC );
        World( const std::string& goalFile );
        ~World();
        float GetLightIntensity ( const b2Vec2& here );
        void Step( float timestep );
}; // END World class

class GuiWorld : public World {
    private:
        void DrawWorld( void );
    public:
        static bool paused;
        static bool step;
        static int skip;

        GLFWwindow* window;
        int draw_interval;

        GuiWorld( float width, float height, int numRobots, int numBoxes, box_shape_t boxShape = SHAPE_CIRC );
        GuiWorld( const std::string& goalFile );
        ~GuiWorld();
        void Step( float timestep );
        bool RequestShutdown();
}; // END GuiWorld class
#endif
