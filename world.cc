#include <iostream>
#include <fstream>
#include <sstream>

#include "push.hh"

double mousex = 0;
double mousey = 0;

bool World::showGui = true;
bool GuiWorld::paused = true;
bool GuiWorld::step = false;
int GuiWorld::skip = 1;

// ===> WORLD class methods
World::World( int controlVal, WorldSettings& worldSet,
        GridLightControllerSettings& glcSet,
        const std::vector<Goal*>& goals ):
    worldSet(&worldSet),
    spawn(1/PORTION_INTERIOR * worldSet.width),
    glc( GridLightController(controlVal, glcSet, goals, worldSet.width) ),
    b2world( new b2World( b2Vec2( 0,0 )) )
{
    AddBoundary( controlVal );
    AddRobots();
    AddPucks();

    if( World::showGui )
        printf("Starting %.2f x %.2f world. Robots: %i Pucks: %i\n", 
            worldSet.width, worldSet.height, 
            worldSet.numRobots, worldSet.numPucks);
}

World::~World(){
    for( auto puck : pucks )
        delete puck;
    for( auto robot : robots )
        delete robot;
    for( auto wall : groundBody )
        delete wall;
}

void World::AddBoundary( int controlVal ){
    float width = worldSet->width;
    float height = worldSet->height;
    b2BodyDef groundBodyDef;
    groundBodyDef.type = b2_staticBody;
    
    // set interior puck container
    b2PolygonShape interiorBox;
    b2FixtureDef interiorFixture;
    interiorBox.SetAsBox( width/3.0, 0.05f );     
    interiorFixture.shape = &interiorBox;     
    // prevent collision with puck-retaining strings 
    interiorFixture.filter.categoryBits = 
        static_cast<uint16_t>(_entityCategory::PUCKBOUNDARY);
    interiorFixture.filter.maskBits = 
        static_cast<uint16_t>(_entityCategory::PUCK); // only pucks

    // Set exterior puck container
    b2PolygonShape exteriorBox;
    b2FixtureDef exteriorFixture;
    exteriorBox.SetAsBox( width/2.0, 0.05f );    
    exteriorFixture.shape = &exteriorBox;
    exteriorFixture.filter.categoryBits = 
        static_cast<uint16_t>(_entityCategory::ROBOTBOUNDARY);
    exteriorFixture.filter.maskBits = 
        static_cast<uint16_t>(_entityCategory::ROBOT) ||
        static_cast<uint16_t>(_entityCategory::PUCK); // contain everything

    // Set exterior container edges: not required due to string
    /***
    b2PolygonShape edge;
    b2FixtureDef edgeFixture;
    edge.SetAsBox( width/2.0, spawn*sqrt(2)/2.0 + EPSILON );
    edgeFixture.shape = &edge;
    edgeFixture.filter.categoryBits = 
        static_cast<uint16_t>(_entityCategory::ROBOTBOUNDARY);
    edgeFixture.filter.maskBits = 
        static_cast<uint16_t>(_entityCategory::ROBOT) || 
        static_cast<uint16_t>(_entityCategory::PUCK); // contain everything
    ***/
 
    float halfSpawn = spawn/2.0;

    // -> Sets the wall arrangements
    // Elements of b2Vec3 cooresponds to:
    // x: center x coordinate
    // y: center y coordinate
    // z: angle

    std::vector<b2Vec3> boundary = {
        b2Vec3( width/2, height/PORTION_INTERIOR, 0 ),    
        b2Vec3( width/2, height-height/PORTION_INTERIOR, 0 ),    
        b2Vec3( width/PORTION_INTERIOR, height/2, M_PI/2.0 ),    
        b2Vec3( width-width/PORTION_INTERIOR, height/2, M_PI/2.0 ),

        b2Vec3( width/2, 0, 0 ),
        b2Vec3( width/2, height, 0 ), 
        b2Vec3( 0, height/2, M_PI/2.0 ), 
        b2Vec3( width, height/2, M_PI/2.0 ), 

        // b2Vec3( halfSpawn, halfSpawn, 3*M_PI/4.0 ), 
        // b2Vec3( width-halfSpawn, halfSpawn, M_PI/4.0 ), 
        // b2Vec3( width-halfSpawn, height-halfSpawn, 3*M_PI/4.0 ), 
        // b2Vec3( halfSpawn, height-halfSpawn, M_PI/4.0 ),
    };

    for( size_t i = 0; i < boundary.size(); ++i ){
        b2Body* wallBody = b2world->CreateBody(&groundBodyDef); 
        if( i < boundary.size()/2 ){
            wallBody->CreateFixture(&interiorFixture);
        } else {
            wallBody->CreateFixture(&exteriorFixture);
            // wallBody->CreateFixture(&edgeFixture);
        }
        groundBody.push_back( new Wall(wallBody, boundary[i]) ); 
    }
}

void World::AddRobots( void ){
    for( int i = 0; i < worldSet->numRobots; ++i )
        robots.push_back( new Pusher( *this, spawn ) );
}

void World::AddPucks( void ){
    for( int i = 0; i < worldSet->numPucks; ++i )
        pucks.push_back( new Puck(*this, spawn, worldSet->puckShape) );
}

float World::GetTotalRobotMovement( void ){
    float totalMovement = 0;
    for( Robot* robot : robots ){
        totalMovement += robot->moveAmount;
    }
    return totalMovement;
}

float World::GetLuminance( const b2Vec2& here ){
    return glc.GetIntensity( here.x, here.y );
}

bool World::Step( int controlVal, float timestep,  
        const std::vector<Goal*>& goals, 
        SimResults& results, 
        std::vector<float>& puckDist ){
    const int32 velocityIterations = 6;
    const int32 positionIterations = 2;

    for( int i=0; i<robots.size(); ++i){
        robots[i]->Update( timestep, *this );
    }

    // Instruct the world to perform a single step of simulation.
    // It is generally best to keep the time step and iterations fixed.
    b2world->Step( timestep, velocityIterations, 
            positionIterations);   

    if( !glc.Update(controlVal, goals, pucks, timestep, puckDist) ){
        results.taskCompletionTime = glc.totalTime;
        results.robotMoveDistance = GetTotalRobotMovement();
        return false;
    } 
    return true;
}
// ===> END WORLD class methods


// ===> GUIWORLD class methods
// void checkmouse( GLFWwindow* win, double x, double y) 
// {
//   //std::cout << x << ' ' << y << std::endl;
//   mousex = x/10.0;
//   mousey = -y/10.0;
// }

void key_callback( GLFWwindow* window, int key, int scancode, int action, 
        int mods) {
    if(action == GLFW_PRESS)
        switch( key ) {
            case GLFW_KEY_SPACE:
                GuiWorld::paused = !GuiWorld::paused; break;

            case GLFW_KEY_S:	
            GuiWorld::paused = true;
            GuiWorld::step = true; //!GuiWorld::step;
            break;

            case GLFW_KEY_LEFT_BRACKET:
                if( mods & GLFW_MOD_SHIFT )
                    GuiWorld::skip = 0;
                else
                    GuiWorld::skip  = std::max( 0, --GuiWorld::skip );
                break;
            case GLFW_KEY_RIGHT_BRACKET:
                if( mods & GLFW_MOD_SHIFT )
                    GuiWorld::skip = 500;
                else
                    GuiWorld::skip++;
                break;
            default:
                break;
        }
}

GuiWorld::GuiWorld( int controlVal, WorldSettings& worldSet, 
        GridLightControllerSettings& glcSet,
        const std::vector<Goal*>& goals ):
    World( controlVal, worldSet, glcSet, goals ),
    window(NULL),
    draw_interval( skip )
{ 
    srand48( time(NULL) );  
    if( World::showGui ){
        /* Initialize the gui library */
        if (!glfwInit()) {
            SimException e("[ERR]", "---GuiWorld(CONSTRUCTOR)---",
                    "Failed", "glfwInit()" );
            throw e;
        }

        /* Create a windowed mode window and its OpenGL context */
        window = glfwCreateWindow(1000, 1000, "Robot Sim", NULL, NULL);
        if (!window) {
            SimException e("[ERR]", "---GuiWorld(CONSTRUCTOR)---",
                    "Failed", "glfwTerminate()" );
            throw e;
        }

        /* Make the window's context current */
        glfwMakeContextCurrent(window);

        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // scale the drawing to fit the whole world in the window, origin
        // at bottom left
        glScalef( 2.0/worldSet.width, 2.0/worldSet.height, 1.0 );
        glTranslatef( -worldSet.width/2.0, -worldSet.height/2.0, 0 );

        tl[0] = 0;      tl[1] = worldSet.height; tl[2] = 0;
        tr[0] = worldSet.width;  tr[1] = worldSet.height; tr[2] = 0;
        bl[0] = 0;      bl[1] = 0;      bl[2] = 0;
        br[0] = worldSet.width;  br[1] = 0;      br[2] = 0;

        // get mouse/pointer events
        //glfwSetCursorPosCallback( window, checkmouse );

        // get key events
        glfwSetKeyCallback( window, key_callback );
    }
}

GuiWorld::~GuiWorld( void ){
    if( World::showGui )
        glfwTerminate();
}

void GuiWorld::DrawTexture( const uint8_t* pixels, 
        unsigned int cols, unsigned int rows ){
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    glEnable(GL_TEXTURE_2D);
     

    glTexImage2D(GL_TEXTURE_2D, 0, 4, cols, rows, 0, 
           GL_RGB,  GL_UNSIGNED_BYTE, pixels );

    glColor3f(1,1,1); // white

    glBegin(GL_QUADS);

    glTexCoord2f(0.0, 0.0);
    glVertex3fv( tl );

    glTexCoord2f(0.0, 1.0);
    glVertex3fv( bl );

    glTexCoord2f(1.0, 1.0);
    glVertex3fv( br );

    glTexCoord2f(1.0, 0.0);
    glVertex3fv( tr );

    glEnd();

    glDisable(GL_TEXTURE_2D);
}

bool GuiWorld::Step( int controlVal, float timestep,  
        const std::vector<Goal*>& goals, 
        SimResults& results,
        std::vector<float>& puckDist){
    if( !paused || step || !World::showGui ) {
        if( !World::Step(controlVal, timestep, goals, results, puckDist) ){
            return false;
        }
        step = false;
        //	paused = true;
    }

    if( World::showGui and --draw_interval < 1 ) {
        draw_interval = skip;

        // ===> DRAW: background color
        glClearColor( 0.8, 0.8, 0.8, 1 ); 
        glClear(GL_COLOR_BUFFER_BIT);	

        glPolygonOffset( 1.0, 1.0 ); 
        DrawTexture( glc.pixels, glc.pRows, glc.pCols );
        for( Goal* goal : goals )
            goal->Draw();

        // ===> DRAW: field objects
        for ( Puck* puck : pucks )
            puck->Draw();
        for ( Robot* robot : robots )
            robot->Draw();
        for ( Wall* wall : groundBody ){ 
            wall->Draw();
        }

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }
    return true;
}

bool GuiWorld::RequestShutdown( void ){
    return World::showGui ? glfwWindowShouldClose(window) : false;
}
// ===> END GUIWORLD class methods
