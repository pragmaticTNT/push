#include <Box2D/Box2D.h>
#include <GLFW/glfw3.h>

#include <math.h>
#include <vector>

class Robot;

class World {
    public:    
        b2World* b2world;
        std::vector<b2Body*> groundBody;
        float width, height, boxDiam;
        
        World( float width, float height, float boxDiam ) :
            width(width),
            height(height),
            boxDiam(boxDiam),
            b2world( new b2World( b2Vec2( 0,0 ))) // gravity 
        {
            b2BodyDef groundBodyDef;
            groundBodyDef.type = b2_staticBody;
            b2PolygonShape groundBox;
            groundBox.SetAsBox( width/2.0, 0.05f );    
            
            int walls = 8;
            //b2Body* groundBody[walls];
            for( int i = 0; i < walls; i++ ) {
                b2Body* wall = b2world->CreateBody(&groundBodyDef);	
                wall->CreateFixture(&groundBox, 0.05f); // Second parameter is density
                groundBody.push_back(wall);
            }
            
            groundBody[0]->SetTransform( 
                    b2Vec2( width/2, 0 ), 
                    0 );    
            groundBody[1]->SetTransform( 
                    b2Vec2( width/2, height ), 
                    0 );    
            groundBody[2]->SetTransform( 
                    b2Vec2( 0, height/2 ), 
                    M_PI/2.0 );    
            groundBody[3]->SetTransform( 
                    b2Vec2( width, height/2 ), 
                    M_PI/2.0 );    
            groundBody[4]->SetTransform( 
                    b2Vec2( boxDiam, boxDiam ), 
                    3*M_PI/4.0 );    
            groundBody[5]->SetTransform( 
                    b2Vec2( width-boxDiam, boxDiam ), 
                    M_PI/4.0 );    
            groundBody[6]->SetTransform( 
                    b2Vec2( width-boxDiam, height-boxDiam ), 
                    3*M_PI/4.0 );    
            groundBody[7]->SetTransform( 
                    b2Vec2( boxDiam, height-boxDiam ), 
                    M_PI/4.0 );    
        }
        
        void Step( float timestep ){
            const int32 velocityIterations = 6;
            const int32 positionIterations = 2;
            
            // Instruct the world to perform a single step of simulation.
            // It is generally best to keep the time step and iterations fixed.
            b2world->Step( timestep, velocityIterations, positionIterations);	
        }
}; // END World class

class Goal {
    public:
        float x, y, r;  // r: radius for circle or sidelen/2 for square
        bool filled;
        Goal (float x, float y, float r):
            x(x), y(y), r(r), filled(false) {}
        ~Goal(){}
}; // END Goal class

class Box {
    public:
        static float size; 
        b2Body* body;
        
        Box( World& world ) : 
            body(NULL) {
            b2PolygonShape dynamicBox;
            dynamicBox.SetAsBox( size/2.0, size/2.0 );
            b2CircleShape dynamicCircle;
            dynamicCircle.m_p.Set(0,0);
            dynamicCircle.m_radius = size/2.0f;
            
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &dynamicCircle;
            fixtureDef.density = 2.0;
            fixtureDef.friction = 1.0;
            fixtureDef.restitution = 0.1;
            
            b2BodyDef bodyDef;
            bodyDef.type = b2_dynamicBody;
            
            body = world.b2world->CreateBody(&bodyDef);    
            body->SetLinearDamping( 10.0 );
            body->SetAngularDamping( 10.0 );
            body->SetTransform( b2Vec2( world.width * drand48(), world.height * drand48()), 0 );	    	    
              
            body->CreateFixture(&fixtureDef);
        }
}; // END Box class

class Light {
    public:
        float x, y; // location

    Light( float x, float y ) : x(x), y(y){}
    ~Light(){}
    float GetIntensity( float dx, float dy ){
        return 1 - 1.0/(1 + pow(x-dx,2) + pow(y-dy,2)); 
    }
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

        LightController( 
                std::vector<Goal*>& goals,
                float goalError = 0.1, 
                float radiusSmall = 1.0,
                float radiusLarge = 1.5) :
            goalError(goalError),
            radiusSmall(radiusSmall),
            radiusLarge(radiusLarge){

            lightSmall = RadiusToIntensity( radiusSmall );
            lightLarge = RadiusToIntensity( radiusLarge );
            for( int i = 0; i < goals.size(); i++){
                lights.push_back( 
                        new Light( goals[i]->x, goals[i]->y) );
            }
        }
        ~LightController(){
            for( int i = 0; i < lights.size(); i++)
                delete lights[i];
        }
        float GetSmallLight( void ){ return lightSmall; }
        float GetLargeLight( void ){ return lightLarge; }
        float GetIntensity( float x, float y );
        void Update( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes );
        //void PairGoalsAndBoxes( void );
}; // END LightController Class

class Robot {
    public:
        static float size;
        static LightController* lightCTRL; 

        b2Body *body, *bumper;
        b2PrismaticJoint* joint;
        
        Robot( World& world, float x, float y, float a);      
        virtual void Update( float timestep ) = 0; // pure 

    protected:
        // get sensor data
        bool GetBumperPressed( void );
        float lightIntensity;
        
        // send commands
        void SetSpeed( float x, float y, float a );
}; // END Robot class

class GuiWorld : public World {
    public:
        static bool paused;
        static bool step;
        static int skip;

        GLFWwindow* window;
        int draw_interval;
        LightController lightCTRL;

        GuiWorld( float width, float height, float boxDiam, LightController& lightCTRL );
        ~GuiWorld();

        virtual void Step( float timestep,
                 const std::vector<Robot*>& robots, 
                 const std::vector<Box*>& bodies, 
                 const std::vector<Goal*>& goals);

        bool RequestShutdown();
}; // END GuiWorld class
