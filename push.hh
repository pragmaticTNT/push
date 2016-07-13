#ifndef PUSH_H
#define PUSH_H

#include <math.h>
#include <string>
#include <vector>

#include <Box2D/Box2D.h>
#include <GLFW/glfw3.h>

class World;

class WorldObject {
    protected:
        b2Vec2 center;
    public:
        WorldObject( float x, float y ) : center( b2Vec2(x,y) ){}
        float DistanceTo( const WorldObject& other ){
            return sqrt(pow(center.x-other.center.x,2) + pow(center.y-other.center.y,2));             
        }
        b2Vec2 GetCenter(){ return center; }
        void WhereAmI(std::string objectType){
            std::cout << objectType << ": (" << center.x << "," << center.y << ")" << std::endl;
        }
}; // END WorldObject class

class Goal : public WorldObject {
    public:
        float r;  // r: radius for circle or sidelen/2 for square
        bool filled;
        Goal (float x, float y, float r): 
            WorldObject(x,y), r(r), filled(false){}
        ~Goal(){}
        void WhereAmI(){ WorldObject::WhereAmI("Goal"); }
}; // END Goal class

class Box : public WorldObject {
    public:
        static float size; 
        b2Body* body;
        
        Box( World& world );
        b2Vec2 GetCenter(){
            center = (body->GetWorldCenter());
            return center;
        }
        void WhereAmI(){ WorldObject::WhereAmI("Box"); }
}; // END Box class

class Light : public WorldObject {
    public:
        Light( float x, float y ) : WorldObject(x,y){}
        ~Light(){}
        float GetIntensity( float dx, float dy ){
            return 1 - 1.0/(1+pow(center.x-dx,2)+pow(center.y-dy,2)); 
        }
        void SetCenter( float x, float y ){ center = b2Vec2(x,y); }
        void SetCenter( b2Vec2 here ){ center = here; }
        void WhereAmI(){ WorldObject::WhereAmI("Light"); }
}; // END Light class

class LightController {
    private:
        float goalError;    // Acceptable distance from goal
        float lightSmall;   // Light intensity corresponding to small radius
        float lightLarge;   // Light intensity corresponding to large radius
        float RadiusToIntensity( float radius ){
            return 1 - 1.0/(1 + pow(radius, 2));
        }

    public:
        std::vector<Light*> lights;
        float radiusSmall;  // Radius at which robot will not enter 
        float radiusLarge;  // Robot will backup out of this radius

        LightController( float goalError = 0.1, float radiusSmall = 1.0, float radiusLarge = 1.5);
        ~LightController();
        float GetSmallLight( void ){ return lightSmall; }
        float GetLargeLight( void ){ return lightLarge; }
        float GetIntensity( float x, float y );
        void SetGoals( std::vector<Goal*>& goals );
        void Update( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes );
        //void PairGoalsAndBoxes( void );
}; // END LightController Class

class Robot : public WorldObject{
    public:
        static float size;
        static LightController* lightCTRL; 

        b2Body *body, *bumper;
        b2PrismaticJoint* joint;
        
        Robot( World& world, float x, float y, float a);      
        void WhereAmI() { WorldObject::WhereAmI("Robot"); }
        virtual void Update( float timestep ) = 0; // pure 

    protected:
        // get sensor data
        bool GetBumperPressed( void );
        float lightIntensity;
        
        // send commands
        void SetSpeed( float x, float y, float a );
}; // END Robot class

class World {
    protected:
        std::vector<Goal*> goals;
        std::vector<Box*> boxes;
        std::vector<Robot*> robots;
        std::vector<b2Body*> groundBody;    // Walls
        LightController lightCTRL;

        void BuildWalls( void ); 
        void AddGoals( void );
        void AddRobots( int numRobots );
        void AddBoxes( int numBoxes );

    public:  
        b2World* b2world;
        float width, height, boxDiam;

        World( float width, float height, float boxDiam, int numRobots, int numBoxes ); 
        void Step( float timestep );
}; // END World class

class GuiWorld : public World {
        // -> Helper Functions
        void DrawDisk( float cx, float cy, float r );
        void DrawCircle( float cx, float cy, float cr );
        void DrawBody( b2Body* b, const float color[3] );
        void Draw( LightController& lightCTRL, const float color[3] );
        void Draw( const std::vector<Box*>& bodies, const float color[3] );
        void Draw( const std::vector<Robot*>&robots, const float color[3] );
        void Draw( const std::vector<b2Body*>& walls, const float color[3] );
        void Draw( const std::vector<Goal*>& goals, const float color[3] );

    public:
        static bool paused;
        static bool step;
        static int skip;

        GLFWwindow* window;
        int draw_interval;

        GuiWorld( float width, float height, float boxDiam, int numRobots, int numBoxes );
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
        float epsilon;   // spawn away from wall
        int turnRight;

    public:
    // constructor
    Pusher( World& world, float epsilon );
    void Update( float timestep );
}; // END Pusher class

#endif
