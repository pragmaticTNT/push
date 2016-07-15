#include <iostream>
#include <fstream>
#include <sstream>

#include "push.hh"

// ===> Set Static Variables
float Box::size;
float Robot::size;

// ===> WORLD class methods
World::World( float width, float height, float boxDiam, size_t numRobots, size_t numBoxes, const std::string& goalFile ) :
    width(width),
    height(height),
    boxDiam(boxDiam),
    lightCTRL(LightController( 0.2, 0.4, 0.5, 0.1 )),
    b2world( new b2World( b2Vec2( 0,0 ))) // gravity 
{
    lightAvoidIntensity = 0.2;
    lightBufferIntensity = 0.4;
    std::cout << "Starting " << width << "x" << height << " World... robots: " << numRobots << " boxes: " << numBoxes << std::endl;
    BuildWalls();
    AddRobots(numRobots);
    AddBoxes(numBoxes, Box::SHAPE_CIRC);
    AddGoals(goalFile);
    lightCTRL.SetGoals(goals, numBoxes);
}

void World::BuildWalls( void ){
    b2BodyDef groundBodyDef;
    groundBodyDef.type = b2_staticBody;
    b2PolygonShape groundBox;
    groundBox.SetAsBox( width/2.0, 0.05f );    
    
    int walls = 8;
    for( int i = 0; i < walls; i++ ) {
        b2Body* wall = b2world->CreateBody(&groundBodyDef); 
        wall->CreateFixture(&groundBox, 0.05f); // Second parameter is density
        groundBody.push_back(wall);
    }
    
    // -> Sets the wall arrangements
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

void World::AddGoals( const std::string& fileName ){
    if( fileName.empty() ){
        goals.push_back( new Goal(3,3,Box::size/2) );
        //goals.push_back( new Goal(1,2,Box::size/2) );
    } else { // read from file
        std::string goal, x, y;
        std::ifstream goalFile(fileName.c_str());
        if( goalFile.is_open() ){
            while( std::getline(goalFile, goal) ){
                std::stringstream ss(goal);
                ss >> x >> y;
                goals.push_back( new Goal(std::stof(x), std::stof(y), Box::size/2) );
            }
            goalFile.close();
        } else {
            std::cout << "[ERR] Unable to open file: " << fileName << std::endl;
        }
    }
}

void World::AddRobots( size_t numRobots ){
    for( int i=0; i<numRobots; i++ )
        robots.push_back( new Pusher( *this, Box::size ) );
}

void World::AddBoxes( size_t numBoxes, Box::box_shape_t shape ){
    for( int i=0; i<numBoxes; i++ )
        boxes.push_back( new Box(*this, shape) );
}

float World::GetLightIntensity( const b2Vec2& here ){
    return lightCTRL.GetIntensity( here.x, here.y );
}

void World::Step( float timestep ){
    const int32 velocityIterations = 6;
    const int32 positionIterations = 2;

    for( int i=0; i<robots.size(); ++i){
        robots[i]->Update( timestep, *this );
    }
    lightCTRL.Update(goals, boxes, timestep); 

    // Instruct the world to perform a single step of simulation.
    // It is generally best to keep the time step and iterations fixed.
    b2world->Step( timestep, velocityIterations, positionIterations);   
}

// ===> BOX class methods
Box::Box( World& world, box_shape_t shape ) :
    WorldObject(0,0),
    body(NULL) 
{
    b2FixtureDef fixtureDef;
    b2PolygonShape dynamicPolygon;
    b2CircleShape dynamicCirc;

    switch( shape ){
        case SHAPE_RECT:
            dynamicPolygon.SetAsBox( size/2.0, size/2.0 );
            fixtureDef.shape = &dynamicPolygon;
            break;
        case SHAPE_CIRC:
            dynamicCirc.m_p.Set(0,0);
            dynamicCirc.m_radius = size/2.0f;
            fixtureDef.shape = &dynamicCirc;
            break;
        case SHAPE_HEX:
            {
                b2Vec2 verts[6];
                for (int i=0; i<6; ++i){
                    verts[i].x = size/2.0 * cos(2.0*M_PI*i/6.0);
                    verts[i].y = size/2.0 * sin(2.0*M_PI*i/6.0);
                }
                dynamicPolygon.Set(verts, 6);
                fixtureDef.shape = &dynamicPolygon;
                break;
            }
        default:
            std::cout << "Invalid shape number " << shape << std::endl;
            break;
    }

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
    center = (body->GetWorldCenter());
}

// ===> ROBOT class methods
Robot::Robot( World& world, const float x, const float y, const float a ) : 
    WorldObject(0,0),
    body( NULL ),
    joint( NULL ) 
{
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    body = world.b2world->CreateBody(&bodyDef);
    bumper = world.b2world->CreateBody(&bodyDef);

    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox( size/2.0, size/2.0 );

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;    
    fixtureDef.density = 10;
    fixtureDef.friction = 1.0;
    body->CreateFixture(&fixtureDef);

    // bumper has same settings the body but different size
    dynamicBox.SetAsBox( size/10.0, size/2.0 );
    bumper->CreateFixture(&fixtureDef);

    b2PrismaticJointDef jointDef;

    jointDef.Initialize( body, 
            bumper, 
            body->GetWorldCenter(), 
            b2Vec2( 1.0f, 0.0f )
    ); 

    jointDef.lowerTranslation = 0;//-0.2;
    jointDef.upperTranslation = 0.04f;
    jointDef.enableLimit = true;
    jointDef.maxMotorForce = 0.8f;
    jointDef.motorSpeed = 1.0f;
    jointDef.localAnchorA.Set( size/2.0, 0); // on the nose
    jointDef.localAnchorB.Set( 0,0 );

    jointDef.enableMotor = true;
    //jointDef.collideConnected = true;

    joint = (b2PrismaticJoint*)world.b2world->CreateJoint( &jointDef );

    // place assembled robot in the world
    body->SetTransform( b2Vec2( x, y ), a );	
    bumper->SetTransform( body->GetWorldPoint( b2Vec2( size/2,0) ), a );	
    center = (body->GetWorldCenter());
}

bool Robot::GetBumperPressed( void ) {
    return( joint->GetJointTranslation() < 0.01 );
}

// set body speed in body-local coordinate frame
void Robot::SetSpeed( float x, float y, float a ) {  
    body->SetLinearVelocity( body->GetWorldVector(b2Vec2( x, y )));
    body->SetAngularVelocity( a );
}

// ===> LIGHT CONTROLLER class methods
// -> PUBLIC
LightController::LightController( float avoidIntensity, float bufferIntensity, float radiusSmall, float goalError ) :
    timeElapsed(0),
    goalError(goalError),
    radiusInit(radiusSmall),
    radiusSmall(radiusSmall),
    avoidIntensity(avoidIntensity),
    bufferIntensity(bufferIntensity)
{
    scaleFactor = GetScaleFactor(radiusSmall);
    radiusLarge = GetRadiusLarge();
}

LightController::~LightController(){
    for( int i = 0; i < lights.size(); i++)
        delete lights[i];
}

float LightController::GetIntensity( float x, float y ){
    // integrate brightness over all light sources
    float brightness = 1.0;
    for( std::vector<Light*>::iterator it = lights.begin(); it != lights.end(); ++it ){
        b2Vec2 center = (*it)->GetCenter();
        float light = 1-1.0/(1+scaleFactor*(pow(center.x-x,2)+pow(center.y-y,2)));
        brightness = std::min(brightness, light);
    }
    return brightness;
}

void LightController::SetGoals( std::vector<Goal*>& goals, size_t numBoxes ){
    for( int i = 0; i < std::min(goals.size(), numBoxes); ++i){
        b2Vec2 goalCenter = goals[i]->GetCenter();
        lights.push_back( 
                new Light( goalCenter.x, goalCenter.y) );
    }
}

// -> Light off-center to the boxes
void LightController::Update( 
        const std::vector<Goal*>& goals,
        const std::vector<Box*>& boxes){ 
    for( int i=0; i<std::min(goals.size(),boxes.size()); ++i ){
        b2Vec2 goal = goals[i]->GetCenter();
        b2Vec2 box = boxes[i]->GetCenter();
        float distBoxToGoal = boxes[i]->DistanceTo(*goals[i]);

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
void LightController::Update(
        const std::vector<Goal*>& goals,
        const std::vector<Box*>& boxes,
        float timeStep ){
    static float maxRadius = 4.0;
    static float growRate = -10e-4;
    timeElapsed += timeStep;
    for( size_t i=0; i<goals.size(); ++i ){
        if( !goals[i]->filled ){
            radiusSmall = std::max(radiusInit, maxRadius + growRate*timeElapsed);
            scaleFactor = GetScaleFactor(radiusSmall);
            radiusLarge = GetRadiusLarge();
        }
    }
    // std::cout << "Time Elapsed: " << timeElapsed << std::endl;
    if( radiusInit/maxRadius > maxRadius + growRate*timeElapsed ){
        timeElapsed = 0;
        maxRadius *= 0.9;
        for( size_t g=0; g<goals.size(); ++g ){
            if( ! goals[g]->filled ){
                for( size_t b=0; b<boxes.size(); ++b ){ 
                    if( goals[g]->DistanceTo(*boxes[b]) < goalError )
                        goals[g]->filled = true;
                }
            }
        }
    }
}

// ===> PUSHER class methods
Pusher::Pusher( World& world, float epsilon ) : 
    epsilon(epsilon),
    Robot( world, 
        drand48() * (world.width - 2*epsilon) + epsilon,
        drand48() * (world.height - 2*epsilon) + epsilon, 
        -M_PI + drand48() * 2.0*M_PI), 
    state( S_TURN ),
    timeleft( drand48() * TURNMAX ),
    speedx( 0 ),
    speeda( 0 ) 
{
    turnRight = drand48() < 0.5 ? 1 : -1; 
}

void Pusher::Update( float timestep, World& world ) {
    // ===> IMPLEMENT ROBOT BEHAVIOUR WITH A LITTLE STATE MACHINE
    center = (body->GetWorldCenter());
    float currentLightIntensity = world.GetLightIntensity(center);
    // std::cout << "Current light intensity: " << currentLightIntensity << std::endl;
    int backupRatio = 10;

    // count down to changing control state
    timeleft -= timestep;

    // force a change of control state
    if( state == S_PUSH && 
        (currentLightIntensity < world.lightAvoidIntensity || GetBumperPressed()) ){
        timeleft = 0.0; // end pushing right now
        speedx = 0;
        speeda = 0;
    }

    if( timeleft <= 0 ) // time to change to another behaviour
        switch( state ) {
            case S_PUSH:
                //std::cout << "In state: BACKUP << std::endl;
                state = currentLightIntensity < world.lightAvoidIntensity ? 
                    S_BACKUP_LONG : S_BACKUP_SHORT;
                timeleft = currentLightIntensity < world.lightAvoidIntensity ?
                    0 : BACKUP;
                speedx = -SPEEDX;
                speeda = 0;           
                break;
            case S_BACKUP_LONG: 
                if( currentLightIntensity > world.lightBufferIntensity || fabs(timeleft) > backupRatio * BACKUP )
                    state = S_BACKUP_SHORT;
                else if( currentLightIntensity < lightIntensity )
                    speedx = -speedx;
                break;
            case S_BACKUP_SHORT: 
                //std::cout << "In state: TURN" << std::endl;
                state = S_TURN;
                timeleft = drand48() * TURNMAX;
                speedx = 0;
                speeda = turnRight * SPEEDA;        
                break;
            case S_TURN: 
                //std::cout << "In state: PUSH" << std::endl;
                state = S_PUSH;
                timeleft = PUSH;
                speedx = SPEEDX;
                speeda = 0;  
                break;
            default:
                std::cout << "invalid control state: " << state << std::endl;
                exit(1);
        }
    SetSpeed( speedx, 0, speeda );
    lightIntensity = currentLightIntensity;
}

    // ===> THE FOLLOWING HAS BEEN MOVED TO: gui.cc
    //
    // const float c_yellow[3] = {1.0, 1.0, 0.0 };
    // const float c_red[3] = {1.0, 0.0, 0.0 };
    // const float c_darkred[3] = {0.8, 0.0, 0.0 };
    // const float c_tan[3] = { 0.8, 0.6, 0.5};
    // const float c_gray[3] = { 0.9, 0.9, 1.0 };
    // 
    // bool GuiWorld::paused = true;
    // bool GuiWorld::step = false;
    // int GuiWorld::skip = 1;

    // void checkmouse( GLFWwindow* win, double x, double y) 
    // {
    //   //std::cout << x << ' ' << y << std::endl;
    //   mousex = x/10.0;
    //   mousey = -y/10.0;
    // }

    // void key_callback( GLFWwindow* window, 
    // 		   int key, int scancode, int action, int mods)
    // {
    //   if(action == GLFW_PRESS)
    //     switch( key )
    //       {
    //       case GLFW_KEY_SPACE:
    // 	GuiWorld::paused = !GuiWorld::paused;
    // 	break;
    // 
    //       case GLFW_KEY_S:	
    // 	GuiWorld::paused = true;
    // 	GuiWorld::step = true; //!GuiWorld::step;
    // 	break;
    // 
    //       case GLFW_KEY_LEFT_BRACKET:
    // 	if( mods & GLFW_MOD_SHIFT )
    // 	  GuiWorld::skip = 0;
    // 	else
    // 	  GuiWorld::skip  = std::max( 0, --GuiWorld::skip );
    // 	break;
    //       case GLFW_KEY_RIGHT_BRACKET:
    // 	if( mods & GLFW_MOD_SHIFT )
    // 	  GuiWorld::skip = 500;
    // 	else
    // 	  GuiWorld::skip++;
    // 	break;
    //       default:
    // 	break;
    //       }
    // }
    // 
    // 
    // void DrawBody( b2Body* b, const float color[3] )
    // {
    //   for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext()) 
    //     {
    //       switch( f->GetType() )
    // 	{
    // 	case b2Shape::e_circle:
    // 	  {
    // 	    b2CircleShape* circle = (b2CircleShape*)f->GetShape();
    // 	  }
    // 	  break;
    // 	case b2Shape::e_polygon:
    // 	  {
    // 	    b2PolygonShape* poly = (b2PolygonShape*)f->GetShape();
    // 	    
    // 	    const int count = poly->GetVertexCount();
    // 	    
    // 	    glColor3fv( color );
    // 	    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL );
    // 	    glBegin( GL_POLYGON );	
    // 	    
    // 	    for( int i = 0; i < count; i++ )
    // 	      {
    // 		const b2Vec2 w = b->GetWorldPoint( poly->GetVertex( i ));		
    // 		glVertex2f( w.x, w.y );
    // 	      }
    // 	    glEnd();		  
    // 	    
    // 	    glLineWidth( 2.0 );
    // 	    glColor3f( color[0]/5, color[1]/5, color[2]/5 );
    // 	    //glColor3fv( color );
    // 	    //glColor3f( 0,0,0 );
    // 	    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // 	    glBegin( GL_POLYGON );	
    // 	    
    // 	    for( int i = 0; i < count; i++ )
    // 	       {
    // 		 const b2Vec2& v = poly->GetVertex( i );		 
    // 	         const b2Vec2 w = b->GetWorldPoint( v );		 
    // 	         glVertex2f( w.x, w.y );
    // 	       }
    // 	    glEnd();		  
    // 	  }
    // 	  break;
    // 	default:
    // 	  break;
    // 	} 
    //     }
    // }
    // 
    // 
    // void DrawDisk(float cx, float cy, float r ) 
    // { 
    //   const int num_segments = 32.0 * sqrtf( r );
    //   
    //   const float theta = 2 * M_PI / float(num_segments); 
    //   const float c = cosf(theta);//precalculate the sine and cosine
    //   const float s = sinf(theta);
    //   float t;
    //   
    //   float x = r; //we start at angle = 0 
    //   float y = 0; 
    //   
    //   //glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    //   glBegin(GL_TRIANGLE_STRIP); 
    //   for(int ii = 0; ii < num_segments; ii++) 
    //     { 
    //       glVertex2f( x + cx, y + cy);//output vertex 
    //       glVertex2f( cx, cy );//output vertex 
    //       
    //       //apply the rotation matrix
    //       t = x;
    //       x = c * x - s * y;
    //       y = s * t + c * y;
    //     } 
    // 
    //   glVertex2f( r + cx, 0 + cy); // first point again to close disk
    // 
    //   glEnd(); 
    // }

    // GuiWorld::GuiWorld( float width, float height ) :
    //   World( width, height ),
    //   window(NULL),
    //   draw_interval( skip )
    //   {
    //     srand48( time(NULL) );  
    // 
    //     /* Initialize the gui library */
    //     if (!glfwInit())
    //       {
    // 	std::cout << "Failed glfwInit()" << std::endl;
    // 	exit(1);
    //       }
    //   
    //     /* Create a windowed mode window and its OpenGL context */
    //     window = glfwCreateWindow(800, 800, "S3", NULL, NULL);
    //     if (!window)
    //       {
    // 	glfwTerminate();
    // 	exit(2);
    //       }
    //   
    //     /* Make the window's context current */
    //     glfwMakeContextCurrent(window);
    //     
    //     glEnable (GL_BLEND);
    //     glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //     
    //     // scale the drawing to fit the whole world in the window, origin
    //     // at bottom left
    //     glScalef( 2.0 / width, 2.0 / height, 1.0 );
    //     glTranslatef( -width/2.0, -height/2.0, 0 );
    //     
    //     // get mouse/pointer events
    //     //glfwSetCursorPosCallback( window, checkmouse );
    //     
    //     // get key events
    //     glfwSetKeyCallback (window, key_callback);
    // }
    // 
    // void GuiWorld::Step( float timestep, 
    // 		const std::vector<Robot*>& robots, 
    // 		const std::vector<Box*>& bodies ) 
    // {
    //   if( !paused || step)
    //     {
    //       World::Step( timestep );
    // 
    //       step = false;
    //       //	paused = true;
    //     }
    // 
    //   if( --draw_interval < 1 )
    //     {
    //       draw_interval = skip;
    // 
    //       glClearColor( 0.8, 0.8, 0.8, 1.0 ); 
    //       glClear(GL_COLOR_BUFFER_BIT);	
    //       
    //       for( int i=0; i<bodies.size(); i++ )
    // 	DrawBody( bodies[i]->body, c_gray );
    //       
    //       for( int i=0; i<robots.size(); i++ )
    // 	{
    // 	  DrawBody( robots[i]->body, c_red );
    // 	  DrawBody( robots[i]->bumper, c_darkred );
    // 	}
    //       
    //       // draw a nose on the robot
    //       glColor3f( 1,1,1 );
    //       glPointSize( 12 );
    //       glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    //       glBegin( GL_TRIANGLES );
    //       for( int i=0; i<robots.size(); i++ )
    // 	{      
    // 	  const b2Transform& t = robots[i]->body->GetTransform();
    // 	  const float a = t.q.GetAngle();
    // 	  
    // 	  glVertex2f( t.p.x + Robot::size/2.0 * cos(a),
    // 		      t.p.y + Robot::size/2.0 * sin(a) );		  
    // 	  glVertex2f( t.p.x + Robot::size/3.0 * cos(a+0.5),
    // 		      t.p.y + Robot::size/3.0 * sin(a+0.5) );		  
    // 	  glVertex2f( t.p.x + Robot::size/3.0 * cos(a-0.5),
    // 		      t.p.y + Robot::size/3.0 * sin(a-0.5) );		  
    // 	}
    //       glEnd();
    //       
    //       glBegin( GL_LINES );
    //       glVertex2f( 0,0 );
    //       glVertex2f( width,0 );
    //       glVertex2f( 0,0 );
    //       glVertex2f( 0,height );
    //       glEnd();
    //       
    //       // draw the light sources  
    //       glColor4f( 1,1,0,0.2 );
    //       for( std::vector<Light>::iterator it = Robot::lights.begin(); 
    // 	   it != Robot::lights.end(); 
    // 	   it++ )
    // 	{
    // 	  DrawDisk( it->x, it->y, sqrt( it->intensity ) );
    // 	}
    //       
    //       /* Swap front and back buffers */
    //       glfwSwapBuffers(window);
    //       
    //       /* Poll for and process events */
    //       glfwPollEvents();
    //     }
    // }
    // 
    // GuiWorld::~GuiWorld( void )
    // {
    //   glfwTerminate();
    // }
    // 
    // bool GuiWorld::RequestShutdown( void )
    // {
    //   return glfwWindowShouldClose(window);
    // }
