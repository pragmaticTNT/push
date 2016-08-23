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
World::World( float width, float height, int robotNum, int boxNum, 
        box_shape_t boxShape ) :
    width(width),
    height(height),
    avoidLuminance(0.2),
    bufferLuminance(0.4),
    lightController(new LightController()),
    b2world( new b2World( b2Vec2( 0,0 )) ) // gravity 
{
    AddBoundary();
    AddRobots(robotNum);
    AddBoxes(boxNum, boxShape);
}

World::World( const std::string& goalFile ):
    width(0),
    height(0),
    avoidLuminance(0.2),
    bufferLuminance(0.4),
    lightController(NULL),
    b2world( new b2World( b2Vec2( 0,0 )) )
{
    std::deque<std::string> settings;
    float cellWidth;

    ParseFile(goalFile, settings);     // modify settings & adds goals

    AddBoundary();
    AddRobots(numRobots);
    AddBoxes(numBoxes, boxShape);
    printf("Starting %.2f x %.2f world. Robots: %i Boxes: %i\n", 
            width, height, numRobots, numBoxes);
    
    if( settings.size() != 3 ){
        std::cout << "[Err-World] Incorrect Number of parameters for LCG goal file. Fields: " << settings.size() << "\n";
        exit(1);
    }
    lightController = new GridLightController( goals, settings, width );
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
    b2PolygonShape edgeBox;
    edgeBox.SetAsBox( width/2.0, SPAWN*sqrt(2)/2.0 + EPSILON );

    float halfSpawn = SPAWN/2.0;
    
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
        b2Vec3( halfSpawn, halfSpawn, 3*M_PI/4.0 ), 
        b2Vec3( width-halfSpawn, halfSpawn, M_PI/4.0 ), 
        b2Vec3( width-halfSpawn, height-halfSpawn, 3*M_PI/4.0 ), 
        b2Vec3( halfSpawn, height-halfSpawn, M_PI/4.0 ) 
    };

    for( size_t i = 0; i < boundary.size(); ++i ){
        b2Body* wallBody = b2world->CreateBody(&groundBodyDef); 
        if( i < boundary.size()/2 )
            wallBody->CreateFixture(&groundBox, 0.05f); // Param2 is density
        else
            wallBody->CreateFixture(&edgeBox, 0.05f); // Param2 is density
        groundBody.push_back( new Wall(wallBody, boundary[i]) );
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

void World::ParseFile( const std::string& fileName, 
        std::deque<std::string>& settings ){
    bool setParameters = false;
    int row, col, index;
    float cellWidth, xCoord, yCoord;

    if( fileName.empty() ){ // DEFAULT
        std::cout << "[WARNING] No file name given. Using DEFAULT.\n";
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
                        settings.push_back(x);
                        settings.push_back(y);
                        while( ss >> param ){ 
                            settings.push_back(param);
                        }
                        setParameters = true;
                        index = ParseSettings(settings);
                        for( int i = 0; i < index; ++i ){
                            settings.pop_front();
                        }
                        cellWidth = width/std::stof(settings.back());
                    } else { 
                        row = std::stoi(x);
                        col = std::stoi(y);
                        index = row*std::stoi(settings.back())+col; 
                        xCoord = col*cellWidth + cellWidth/2.0;
                        yCoord = row*cellWidth + cellWidth/2.0; 
                        goals.push_back( new Goal( xCoord, yCoord, 
                                        cellWidth, index) );
                    }
                }
            }
            goalFile.close();
        } else {
            std::cout << "[ERR] Unable to open file: " << fileName << std::endl;
        }
    }
}

int World::ParseSettings( const std::deque<std::string>& settings ){
    width = std::stof(settings[1]);
    height = std::stof(settings[2]);
    numBoxes = std::stoi(settings[3]);
    Box::size = std::stof(settings[4]);
    switch(settings[5].c_str()[0]){
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
            printf("[ERR-World] invalid box shape: %s\n", 
                    settings[5].c_str());
            exit(1);
    }
    numRobots = std::stoi(settings[6]);
    Robot::size = std::stof(settings[7]);
    numPatterns = std::stoi(settings[8]);
    // std::cout << width << "x" << height << '\n';
    return 9;   // Number of elements used
}

float World::GetLuminance( const b2Vec2& here ){
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

GuiWorld::GuiWorld( float width, float height, int numRobots, int numBoxes,
        box_shape_t boxShape ) : 
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

    tl[0] = 0;      tl[1] = height; tl[2] = 0;
    tr[0] = width;  tr[1] = height; tr[2] = 0;
    bl[0] = 0;      bl[1] = 0;      bl[2] = 0;
    br[0] = width;  br[1] = 0;      br[2] = 0;

    // get mouse/pointer events
    //glfwSetCursorPosCallback( window, checkmouse );

    // get key events
    glfwSetKeyCallback( window, key_callback );
}

void GuiWorld::DrawTexture( const uint8_t* pixels,
          const unsigned int cols,
          const unsigned int rows )
{
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

void GuiWorld::Step( float timestep ){
    if( !paused || step) {
        World::Step( timestep );
        step = false;
        //	paused = true;
    }

    if( --draw_interval < 1 ) {
        draw_interval = skip;

        // ===> DRAW: background color
        glClearColor( 0.8, 0.8, 0.8, 1 ); 
        glClear(GL_COLOR_BUFFER_BIT);	

        glPolygonOffset( 1.0, 1.0 ); 
        DrawTexture( lightController->pixels, lightController->pRows, 
                     lightController->pCols );
        // for( auto light : lightController->lights )
        //     light->Draw();
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
