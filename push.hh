#ifndef PUSH_H
#define PUSH_H

#include <math.h>
#include <string>
#include <vector>

#include <Box2D/Box2D.h>
#include <GLFW/glfw3.h>

static const float EPSILON = 10e-4;
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
        float SqrDistanceTo( const WorldObject& other ){
            return pow(center.x-other.center.x,2.0) + pow(center.y-other.center.y,2.0);
        }
        float SqrDistanceTo( float x, float y ){
            return pow(center.x-x,2.0) + pow(center.y-y,2.0);
        }
        b2Vec2 GetCenter( void ){ return center; }
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
        float radius;  // r: circle radius or sidelen/2 for square
        bool filled;

        Goal ( float x, float y, float radius ) : 
            WorldObject(x,y), radius(radius), filled(false){}
        ~Goal(){}
        void WhereAmI( void ){ WorldObject::WhereAmI("Goal"); }
        void Draw( void );
}; // END Goal class

class Light : public WorldObject {
    public:
        static float radiusSmall, radiusLarge;

        Light( float x, float y ) : WorldObject(x,y){}
        Light( b2Vec2 center ) : WorldObject(center){}
        ~Light(){}

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

class LightController {
    private:
        float scaleFactor;          // ORDER DEPENDENCY 
        float avoidIntensity;   
        float bufferIntensity;  

        // Update parameters
        float goalError, growRate, trialTime, patternTime;
        float timeElapsed, maxRadius;
        bool isFilled, repeatPattern;
        bool startScramble, scramble;

        float GetScaleFactor( float radius ){
            return ((1.0/(1-avoidIntensity))-1)/pow(radius,2.0);
        }
        float GetRadiusLarge(){
            return sqrt(((1.0/(1-bufferIntensity))-1)/scaleFactor);
        }
        float UpdateGoalsInfo( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes );

    public:
        std::vector<Light*> lights;
        // TODO(Lily): Why are these public?
        float radiusInit;
        float radiusSmall; 
        float radiusLarge;         // ORDER DEPENDENCY

        LightController( float avoidIntensity = 0.2, float bufferIntensity = 0.4, float radiusSmall = 0.5);
        ~LightController();
        float GetIntensity( float x, float y );
        void SetGoals( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes, float goalError, float growRate, float trialWaitTime, float patternWaitTime = 0, bool repeatPattern = false);
        void Update( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes );
        void Update( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes, float timeStep );
}; // END LightController Class

class World {
    protected:
        std::vector<std::vector<Goal*> > goalList;
        std::vector<Goal*> goals;
        std::vector<Box*> boxes;
        std::vector<Robot*> robots;
        std::vector<Wall*> groundBody;    // Boundary
        LightController lightCTRL;
        float spawnDist;    // Spawn object away from wall
        size_t numBoxes, numPatterns;

        void AddBoundary( void );
        void AddGoals( const std::string& goalFile, std::vector<std::string>& parameters );
        void AddRobots( size_t numRobots );
        void AddBoxes( size_t numBoxes, box_shape_t shape );

    public:  
        b2World* b2world;
        float width, height;
        float lightAvoidIntensity, lightBufferIntensity; // Small and Large circle radii respectively 

        World( float worldWidth, float worldHeight, float spawnDist, size_t robotNum, size_t boxNum, const std::string& goalFile, box_shape_t boxShape = SHAPE_CIRC, float lightAvoidIntensity = 0.2, float lightBufferIntensity = 0.4, float lightSmallRadius = 0.5 );
        ~World();
        float GetLightIntensity ( const b2Vec2& here );
        void Step( float timestep );
}; // END World class

class GuiWorld : public World {
        void DrawObjects( const std::vector<WorldObject*>& objects );
    public:
        static bool paused;
        static bool step;
        static int skip;

        GLFWwindow* window;
        int draw_interval;

        GuiWorld( float width, float height, float boxDiam, size_t numRobots, size_t numBoxes, const std::string& fileName, box_shape_t boxShape = SHAPE_CIRC, float lightAvoidIntensity = 0.2, float lightBufferIntensity = 0.4, float lightSmallRadius = 0.5 );
        ~GuiWorld();
        void Step( float timestep );
        bool RequestShutdown();
}; // END GuiWorld class
#endif
