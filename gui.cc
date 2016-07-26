#include <vector>
#include <iostream>

#include "push.hh"

double mousex = 0;
double mousey = 0;

bool GuiWorld::paused = true;
bool GuiWorld::step = false;
int GuiWorld::skip = 1;

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

GuiWorld::GuiWorld( float width, float height, float boxDiam, size_t numRobots, size_t numBoxes, const std::string& fileName ) : 
    World( width, height, boxDiam, numRobots, numBoxes, fileName ),
    window(NULL),
    draw_interval( skip ) {

    srand48( time(NULL) );  
    /* Initialize the gui library */
    if (!glfwInit()) {
        std::cout << "Failed glfwInit()" << std::endl;
        exit(1);
    }

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(800, 800, "Robot Sim", NULL, NULL);
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
    glScalef( 2.0 / width, 2.0 / height, 1.0 );
    glTranslatef( -width/2.0, -height/2.0, 0 );

    // get mouse/pointer events
    //glfwSetCursorPosCallback( window, checkmouse );

    // get key events
    glfwSetKeyCallback (window, key_callback);
}

void GuiWorld::DrawObjects( const std::vector<WorldObject*>& objects ){
    for( auto obj : objects )
        obj->Draw();
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
        // DrawObjects( static_cast<std::vector<WorldObject*> >(lightCTRL.lights) );
        // DrawObjects( goals );
        for( auto light : lightCTRL.lights )
            light->Draw();
        for( auto goal : goals )
            goal->Draw();

        // ===> DRAW: field objects
        glPolygonOffset( 0, 0 ); 
        // DrawObjects( boxes );
        // DrawObjects( robots );
        // DrawObjects( groundBody );
        for ( auto box : boxes )
            box->Draw();
        for ( auto robot : robots )
            robot->Draw();
        for ( auto wall : groundBody )
            wall->Draw();

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }
}

GuiWorld::~GuiWorld( void ){
    glfwTerminate();
}

bool GuiWorld::RequestShutdown( void ){
    return glfwWindowShouldClose(window);
}
