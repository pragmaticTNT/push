#include <iostream>

#include "push.hh"

// ===> Set Static Variables
LightController* Robot::lightCTRL;
float Box::size;
float Robot::size;

// ===> WORLD class methods
World::World( float width, float height, float boxDiam, int numRobots, int numBoxes ) :
    width(width),
    height(height),
    boxDiam(boxDiam),
    lightCTRL(LightController( 0.1, 1.0, 1.2 )),
    b2world( new b2World( b2Vec2( 0,0 ))) // gravity 
{
    std::cout << "Starting " << width << "x" << height << " World... robots: " << numRobots << " boxes: " << numBoxes << std::endl;
    BuildWalls();
    AddGoals();
    AddRobots(numRobots);
    AddBoxes(numBoxes);
    lightCTRL.SetGoals(goals);
    Robot::lightCTRL = &lightCTRL;
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

void World::AddGoals( void ){
    goals.push_back( new Goal(3,3,Box::size/2) );
    goals.push_back( new Goal(1,2,Box::size/2) );
}

void World::AddRobots( int numRobots ){
    for( int i=0; i<numRobots; i++ )
        robots.push_back( new Pusher( *this, Box::size ) );
}

void World::AddBoxes( int numBoxes ){
    for( int i=0; i<numBoxes; i++ )
        boxes.push_back( new Box( *this ) );
}

void World::Step( float timestep ){
    const int32 velocityIterations = 6;
    const int32 positionIterations = 2;

    for( int i=0; i<robots.size(); ++i){
        robots[i]->Update( timestep );
    }
    lightCTRL.Update(goals, boxes); 

    // Instruct the world to perform a single step of simulation.
    // It is generally best to keep the time step and iterations fixed.
    b2world->Step( timestep, velocityIterations, positionIterations);   
}

// ===> BOX class methods
Box::Box( World& world ) :
    WorldObject(0,0),
    body(NULL) 
{
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
LightController::LightController( float goalError, float radiusSmall, float radiusLarge ) :
    goalError(goalError),
    radiusSmall(radiusSmall),
    radiusLarge(radiusLarge)
{
    lightSmall = RadiusToIntensity( radiusSmall );
    lightLarge = RadiusToIntensity( radiusLarge );
}

LightController::~LightController(){
    for( int i = 0; i < lights.size(); i++)
        delete lights[i];
}

float LightController::GetIntensity( float x, float y ){
    // integrate brightness over all light sources
    float brightness = 1.0;
    for( std::vector<Light*>::iterator it = lights.begin(); it != lights.end(); ++it ){
        brightness = std::min(brightness,(*it)->GetIntensity(x,y));
    }
    return brightness;
}

void LightController::SetGoals( std::vector<Goal*>& goals ){
    for( int i = 0; i < goals.size(); i++){
        b2Vec2 goalCenter = goals[i]->GetCenter();
        lights.push_back( 
                new Light( goalCenter.x, goalCenter.y) );
    }
}

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
            std::cout << "[GOAL REACHED]" << std::endl;
        }
        //goals[i]->WhereAmI();
        //boxes[i]->WhereAmI();
        //lights[i]->WhereAmI();
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

void Pusher::Update( float timestep ) {
    // ===> IMPLEMENT ROBOT BEHAVIOUR WITH A LITTLE STATE MACHINE
    b2Vec2 here = body->GetWorldCenter();
    float currentLightIntensity = lightCTRL->GetIntensity(here.x, here.y);
    // count down to changing control state
    timeleft -= timestep;

    // force a change of control state
    if( state == S_PUSH && 
        (currentLightIntensity < lightCTRL->GetSmallLight() || GetBumperPressed()) ){
        timeleft = 0.0; // end pushing right now
        speedx = 0;
        speeda = 0;
    }

    if( timeleft <= 0 ) // time to change to another behaviour
        switch( state ) {
            case S_PUSH:
                //std::cout << "In state: BACKUP << std::endl;
                state = currentLightIntensity < lightCTRL->GetSmallLight() ? 
                    S_BACKUP_LONG : S_BACKUP_SHORT;
                timeleft = currentLightIntensity < lightCTRL->GetSmallLight() ?
                    0 : BACKUP;
                speedx = -SPEEDX;
                speeda = 0;           
                break;
            case S_BACKUP_LONG: 
                if( currentLightIntensity > lightCTRL->GetLargeLight() || fabs(timeleft) > 10 * BACKUP )
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
