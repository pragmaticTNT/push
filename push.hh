#ifndef PUSH_H
#define PUSH_H

#include <math.h>
#include <string>
#include <vector>

#include <Box2D/Box2D.h>
#include <GLFW/glfw3.h>

typedef float XCoord;
typedef float YCoord;
typedef float Angle;
typedef float Radius;
typedef float Intensity;
typedef float Epsilon;  // Error tolerance
typedef b2Vec2 Center;
typedef enum {
    SHAPE_RECT=0,
    SHAPE_CIRC,
    SHAPE_HEX
} box_shape_t;

class World;

class WorldObject {
    protected:
        Center center;
        void WhereAmI(std::string objectType){
            std::cout << objectType << ": (" << center.x << "," << center.y << ")" << std::endl;
        }
    public:
        WorldObject( XCoord x, YCoord y ) : center( b2Vec2(x,y) ){}
        WorldObject( Center center ): center( center ){}
        float SqrDistanceTo( const WorldObject& other ){
            return pow(center.x-other.center.x,2) + pow(center.y-other.center.y,2);             
        }
        float SqrDistanceTo( XCoord x, YCoord y ){
            return pow(center.x-x,2) + pow(center.y-y,2);              
        }
        Center GetCenter(){ return center; }
}; // END WorldObject class

class Goal : public WorldObject {
    public:
        Radius radius;  // r: radius for circle or sidelen/2 for square
        bool filled;

        Goal ( XCoord x, YCoord y, Radius radius ): 
            WorldObject(x,y), radius(radius), filled(false){}
        ~Goal(){}
        void WhereAmI(){ WorldObject::WhereAmI("Goal"); }
}; // END Goal class

class Box : public WorldObject {
    public:
        static float size; 
        b2Body* body;
        
        Box( World& world, Epsilon spawnDist, box_shape_t shape = SHAPE_RECT );
        Center GetCenter(){
            center = (body->GetWorldCenter());
            return center;
        }
        void WhereAmI(){ WorldObject::WhereAmI("Box"); }
}; // END Box class

class Light : public WorldObject {
    public:
        Light( XCoord x, YCoord y ) : WorldObject(x,y){}
        Light( Center center ) : WorldObject(center){}
        ~Light(){}

        void SetCenter( XCoord x, YCoord y ){ center = b2Vec2(x,y); }
        void SetCenter( Center here ){ center = here; }
        void WhereAmI(){ WorldObject::WhereAmI("Light"); }
}; // END Light class

class LightController {
    private:
        float scaleFactor;          // ORDER DEPENDENCY 
        float timeElapsed;          
        Epsilon goalError;          
        Intensity avoidIntensity;   
        Intensity bufferIntensity;  

        float GetScaleFactor( Radius radius ){
            return ((1.0/(1-avoidIntensity))-1)/pow(radius,2.0);
        }
        Radius GetRadiusLarge(){
            return sqrt(((1.0/(1-bufferIntensity))-1)/scaleFactor);
        }

    public:
        std::vector<Light*> lights;
        Radius radiusInit;
        Radius radiusSmall; 
        Radius radiusLarge;         // ORDER DEPENDENCY

        LightController( Intensity avoidIntensity = 0.2, Intensity bufferIntensity = 0.4, Radius radiusSmall = 1.0, Epsilon goalError = 0.1 );
        ~LightController();
        Intensity GetIntensity( XCoord x, YCoord y );
        void SetGoals( const std::vector<Goal*>& goals );
        void Update( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes );
        void Update( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes, float timeStep );
        //void PairGoalsAndBoxes( void );
}; // END LightController Class

class Robot : public WorldObject{
    protected:
        // get sensor data
        bool isBumperPressed( void );
        Intensity lightIntensity;
        
        // send commands
        void SetSpeed( XCoord x, YCoord y, Angle angle );

    public:
        static float size;
        b2Body *body, *bumper;
        b2PrismaticJoint* joint;
        
        Robot( World& world, XCoord x, YCoord y, Angle angle);      
        Center GetCenter(){
            center = (body->GetWorldCenter());
            return center;
        }
        void WhereAmI() { WorldObject::WhereAmI("Robot"); }
        virtual void Update( float timestep, World& world ) = 0; 
}; // END Robot class

class World {
    protected:
        std::vector<Goal*> goals;
        std::vector<Box*> boxes;
        std::vector<Robot*> robots;
        std::vector<b2Body*> groundBody;    // Boundary
        LightController lightCTRL;
        Epsilon spawnDist;                  // Spawn object away from wall

        void AddBoundary( void );
        void AddGoals( const std::string& goalFile = "" );
        void AddRobots( size_t numRobots );
        void AddBoxes( size_t numBoxes, box_shape_t shape );

    public:  
        b2World* b2world;
        float width, height, boxDiam;
        Intensity lightAvoidIntensity, lightBufferIntensity; // Small and Large circle radii respectively 

        World( float width, float height, float boxDiam, size_t numRobots, size_t numBoxes, const std::string& fileName, box_shape_t shape = SHAPE_CIRC ); 
        Intensity GetLightIntensity ( const Center& here );
        void Step( float timestep );
}; // END World class

class GuiWorld : public World {
        // -> Helper Functions
        void DrawDisk( Center center, Radius radius );
        void DrawCircle( Center center, Radius radius );
        void DrawBody( b2Body* b, const float color[3] );
        void Draw( const LightController& lightCTRL, const float color[3] );
        void Draw( const std::vector<Box*>& bodies, const float color[3] );
        void Draw( const std::vector<Robot*>& robots, const float color[3] );
        void Draw( const std::vector<b2Body*>& walls, const float color[3] );
        void Draw( const std::vector<Goal*>& goals, const float color[3] );

    public:
        static bool paused;
        static bool step;
        static int skip;

        GLFWwindow* window;
        int draw_interval;

        GuiWorld( float width, float height, float boxDiam, size_t numRobots, size_t numBoxes, const std::string& fileName );
        ~GuiWorld();
        void Step( float timestep );
        bool RequestShutdown();
}; // END GuiWorld class

class Pusher : public Robot {
    private:
        typedef enum {
            S_PUSH = 0,
            S_BACKUP_LONG,
            S_BACKUP_SHORT,
            S_TURN,
            S_COUNT
        } control_state_t;

        static const float PUSH, BACKUP, TURNMAX;
        static const float SPEEDX, SPEEDA;
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

#endif
