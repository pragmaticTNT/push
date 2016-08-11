#include <iostream>
#include <fstream>
#include <sstream>

#include "push.hh"

double mousex = 0;
double mousey = 0;

bool GuiWorld::paused = true;
bool GuiWorld::step = false;
int GuiWorld::skip = 1;

// ===> WORLD class methods
World::World( float width, float height, int robotNum, int boxNum, box_shape_t boxShape ) :
    width(width),
    height(height),
    lightAvoidIntensity(0.2),
    lightBufferIntensity(0.4),
    lightController(NULL),
    b2world( new b2World( b2Vec2( 0,0 )) ) // gravity 
{
    AddBoundary();
    AddRobots(robotNum);
    AddBoxes(boxNum, boxShape);
    lightController = new LightController( lightAvoidIntensity, lightBufferIntensity, 1.0);
}

World::World( const std::string& goalFile ):
    width(0),
    height(0),
    lightAvoidIntensity(0.2),
    lightBufferIntensity(0.4),
    lightController(NULL),
    b2world( new b2World( b2Vec2( 0,0 )) )
{
    std::queue<std::string> settings;
    int numBoxes, numRobots;
    float cellWidth;
    light_controller_t controllerType;
    box_shape_t boxShape;

    ParseFile(goalFile, settings);     // modify settings & adds goals

    switch(settings.front().c_str()[0]){
        case 'R': case 'r':
            controllerType = LC_RADIAL;
            break;
        case 'G': case 'g':
            controllerType = LC_GRID;
            break;
        default:
            std::cout << "[ERR-W] invalid lightcontroller type: " << settings.front() << "\n";
            exit(1);
    }
    settings.pop();
    width = std::stof(settings.front());
    settings.pop();
    height = std::stof(settings.front());
    settings.pop();
    numBoxes = std::stoi(settings.front());
    settings.pop();
    Box::size = std::stof(settings.front());
    settings.pop();
    switch(settings.front().c_str()[0]){
        case 'C': case 'c':
            boxShape = SHAPE_CIRC;
            break;
        case 'S': case 's':
            boxShape = SHAPE_RECT;
            break;
        case 'H': case 'h':
            boxShape = SHAPE_HEX;
            break;
        default:
            std::cout << "[ERR-W] invalid box shape: " << settings.front() << "\n";
            exit(1);
    }
    settings.pop();
    numRobots = std::stoi(settings.front());
    settings.pop();
    Robot::size = std::stof(settings.front());
    settings.pop();
    numPatterns = std::stoi(settings.front());
    settings.pop();

    AddBoundary();
    AddRobots(numRobots);
    AddBoxes(numBoxes, boxShape);
    std::cout << "Starting " << width << "x" << height << " World... robots: " << numRobots << " boxes: " << numBoxes << "\n";
    
    switch( controllerType ){
        case LC_RADIAL:
            lightController = new RadialLightController(goals, settings, lightAvoidIntensity, lightBufferIntensity);
            break;
        case LC_GRID:
            if( settings.size() != 3 ){
                std::cout << "[Err-W] Number of parameters for LCG goal file is incorrect. Fields: " << settings.size() << "\n";
                exit(1);
            }
            cellWidth = width/std::stof(settings.back());
            lightController = new GridLightController(goals, settings, lightAvoidIntensity, lightBufferIntensity, cellWidth);
    }
}

World::~World(){
    delete lightController;
    for( auto goal : goals )
        delete goal;
    for( auto box : boxes )
        delete box;
    for( auto robot : robots )
        delete robot;
    for( auto wall : groundBody )
        delete wall;
}

void World::AddBoundary( void ){
    b2BodyDef groundBodyDef;
    groundBodyDef.type = b2_staticBody;
    b2PolygonShape groundBox;
    groundBox.SetAsBox( width/2.0, 0.05f );    
    
    // -> Sets the wall arrangements
    // Elements of b2Vec3 cooresponds to:
    // x: center x coordinate
    // y: center y coordinate
    // z: angle

    // TODO: set a proper octagonal boundary and "line" solution/
    std::vector<b2Vec3> boundary = {
        b2Vec3( width/2, 0, 0 ),
        b2Vec3( width/2, height, 0 ), 
        b2Vec3( 0, height/2, M_PI/2.0 ), 
        b2Vec3( width, height/2, M_PI/2.0 ), 
        b2Vec3( SPAWN, SPAWN, 3*M_PI/4.0 ), 
        b2Vec3( width-SPAWN, SPAWN, M_PI/4.0 ), 
        b2Vec3( width-SPAWN, height-SPAWN, 3*M_PI/4.0 ), 
        b2Vec3( SPAWN, height-SPAWN, M_PI/4.0 ) 
    };

    for( b2Vec3 wall : boundary ){
        b2Body* wallBody = b2world->CreateBody(&groundBodyDef); 
        wallBody->CreateFixture(&groundBox, 0.05f); // Param2 is density
        groundBody.push_back( new Wall(wallBody, wall) );
    }
    
    // TODO: set the no spawn boundary property based on wall size
    // SPAWN = 0; // Do some calculations
}

void World::AddRobots( int numRobots ){
    for( int i=0; i<numRobots; i++ )
        robots.push_back( new Pusher( *this, SPAWN ) );
}

void World::AddBoxes( int numBoxes, box_shape_t shape ){
    for( int i=0; i<numBoxes; i++ )
        boxes.push_back( new Box(*this, SPAWN, shape) );
}

void World::ParseFile( const std::string& fileName, std::queue<std::string>& settings ){
    bool setParameters = false;
    float radius = 1.0;
    float cellWidth = 1.0;
    int row, col, index;
    float xCoord, yCoord;
    char controller;
    bool radialController;

    if( fileName.empty() ){ // DEFAULT
        std::cout << "[WARNING] No file name given. Using DEFAULT.";
        goals.push_back( new Goal(3,3,Box::size/2) );
    } else {                // READ FROM FILE
        std::string goal, x, y, param;
        std::ifstream goalFile(fileName.c_str());
        if( goalFile.is_open() ){
            while( std::getline(goalFile, goal) ){
                std::stringstream ss(goal);
                ss >> x >> y;
                if( x.at(0) != '#' && goal.size() > 0 ){
                    if( !setParameters ){ 
                        settings.push(x);
                        settings.push(y);
                        while( ss >> param ){ 
                            settings.push(param);
                        }
                        setParameters = true;
                        controller = toupper(settings.front().c_str()[0]);
                        radialController = controller == 'R'; 
                    } else { 
                        if( radialController ){
                            goals.push_back( new Goal(std::stof(x), std::stof(y), radius) );
                        } else {
                            row = std::stoi(x);
                            col = std::stoi(y);
                            index = row*std::stoi(settings.back())+col; 
                            xCoord = col*cellWidth + cellWidth/2.0;
                            yCoord = row*cellWidth + cellWidth/2.0; 
                            goals.push_back( new Goal(xCoord, yCoord, radius, index) );
                        }
                    }
                }
            }
            goalFile.close();
        } else {
            std::cout << "[ERR] Unable to open file: " << fileName << std::endl;
        }
    }
}

float World::GetLightIntensity( const b2Vec2& here ){
    return lightController->GetIntensity( here.x, here.y );
}

void World::Step( float timestep ){
    const int32 velocityIterations = 6;
    const int32 positionIterations = 2;

    for( int i=0; i<robots.size(); ++i){
        robots[i]->Update( timestep, *this );
    }
    lightController->Update(goals, boxes, timestep); 

    // Instruct the world to perform a single step of simulation.
    // It is generally best to keep the time step and iterations fixed.
    b2world->Step( timestep, velocityIterations, positionIterations);   
}
// ===> END WORLD class methods


// ===> GUIWORLD class methods
// void checkmouse( GLFWwindow* win, double x, double y) 
// {
//   //std::cout << x << ' ' << y << std::endl;
//   mousex = x/10.0;
//   mousey = -y/10.0;
// }

void key_callback( GLFWwindow* window, int key, int scancode, int action, int mods) {
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

GuiWorld::GuiWorld( float width, float height, int numRobots, int numBoxes, box_shape_t boxShape ) : 
    World( width, height, numRobots, numBoxes, boxShape ),
    window(NULL),
    draw_interval( skip ) 
{ DrawWorld(); }

GuiWorld::GuiWorld( const std::string& goalFile ):
    World( goalFile ),
    window(NULL),
    draw_interval( skip )
{ DrawWorld(); }

GuiWorld::~GuiWorld( void ){
    glfwTerminate();
}

void GuiWorld::DrawWorld( void ){
    srand48( time(NULL) );  
    /* Initialize the gui library */
    if (!glfwInit()) {
        std::cout << "Failed glfwInit()" << std::endl;
        exit(1);
    }

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(1000, 1000, "Robot Sim", NULL, NULL);
    if (!window) {
        glfwTerminate();
        exit(2);
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);

    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // scale the drawing to fit the whole world in the window, origin
    // at bottom left
    glScalef( 2.0/width, 2.0/height, 1.0 );
    glTranslatef( -width/2.0, -height/2.0, 0 );

    // get mouse/pointer events
    //glfwSetCursorPosCallback( window, checkmouse );

    // get key events
    glfwSetKeyCallback( window, key_callback );
}

void GuiWorld::Step( float timestep ){
    if( !paused || step) {
        World::Step( timestep );
        step = false;
        //	paused = true;
    }

    if( --draw_interval < 1 ) {
        draw_interval = skip;

        // ===> DRAW: background color
        glClearColor( 1, 1, 0.6, 1 ); 
        glClear(GL_COLOR_BUFFER_BIT);	

        glPolygonOffset( 1.0, 1.0 ); 
        for( auto light : lightController->lights )
            light->Draw();
        for( auto goal : goals )
            goal->Draw();

        // ===> DRAW: field objects
        // glPolygonOffset( 0, 0 ); 
        for ( auto box : boxes )
            box->Draw();
        for ( auto robot : robots )
            robot->Draw();
        for ( auto wall : groundBody ){ 
            wall->Draw();
        }

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }
}

bool GuiWorld::RequestShutdown( void ){
    return glfwWindowShouldClose(window);
}
// ===> END GUIWORLD class methods
