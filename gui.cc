#include <vector>
#include <iostream>

#include "push.hh"

double mousex = 0;
double mousey = 0;

// ===> Specified colors
const float c_yellow[3]     = {1.0, 1.0, 0.0};
const float c_light[3]      = {0.8, 0.8, 0.8};
const float c_red[3]        = {1.0, 0.0, 0.0};
const float c_blue[3]       = {0.0, 1.0, 0.0};
const float c_green[3]      = {0.0, 0.0, 1.0};
const float c_darkred[3]    = {0.8, 0.0, 0.0};
const float c_tan[3]        = {0.8, 0.6, 0.5};
const float c_gray[3]       = {0.9, 0.9, 1.0};
const float c_black[3]      = {0.0, 0.0, 0.0};

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

void GuiWorld::DrawDisk(float cx, float cy, float r ) { 
    const int num_segments = 32.0 * sqrtf( r );
    const float theta = 2 * M_PI / float(num_segments); 
    const float c = cosf(theta);//precalculate the sine and cosine
    const float s = sinf(theta);
    float t;

    float x = r; //we start at angle = 0 
    float y = 0; 

    //glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    glBegin(GL_TRIANGLE_STRIP); 
        for(int ii = 0; ii < num_segments; ii++) { 
            glVertex2f( x + cx, y + cy);//output vertex 
            glVertex2f( cx, cy );//output vertex 

            //apply the rotation matrix
            t = x;
            x = c * x - s * y;
            y = s * t + c * y;
        }
    glVertex2f( r + cx, 0 + cy); // first point again to close disk
    glEnd();
}

void GuiWorld::DrawCircle(float cx, float cy, float cr) {
    const int lineAmount = 32.0 * sqrtf(cr);
    float twicePi = M_PI * 2.0f;

    glBegin(GL_LINE_LOOP);
        for( int i = 0; i < lineAmount; i++ ){
            glVertex2f(
                    cx + (cr * cosf(i*twicePi/lineAmount)),
                    cy + (cr * sinf(i*twicePi/lineAmount))
            );
        }
    glEnd();
}

void GuiWorld::DrawBody( b2Body* b, const float color[3] ) {
    for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext()){
        switch( f->GetType() ){
            case b2Shape::e_circle:
                {
                b2CircleShape* circle = (b2CircleShape*)f->GetShape();
                b2Vec2 position = b->GetWorldCenter();
                glColor3fv( color );
                DrawDisk( position.x, position.y, circle->m_radius ); 
                }
                break;
            case b2Shape::e_polygon:
                {
                b2PolygonShape* poly = (b2PolygonShape*)f->GetShape();
                const int count = poly->GetVertexCount();

                glColor3fv( color );
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL );
                glBegin( GL_POLYGON );	
                for( int i = 0; i < count; i++ ){
                    const b2Vec2 w = b->GetWorldPoint( poly->GetVertex( i ));		
                    glVertex2f( w.x, w.y );
                }
                glEnd();		  

                glLineWidth( 3.0 );
                // glColor3f( color[0]/5, color[1]/5, color[2]/5 );
                // // TODO: this line in particular is related to the circles not being properly filled in... I wonder why
                // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                // glBegin( GL_POLYGON );	
                // for( int i = 0; i < count; i++ ){
                //     const b2Vec2& v = poly->GetVertex( i );		 
                //     const b2Vec2 w = b->GetWorldPoint( v );		 
                //     glVertex2f( w.x, w.y );
                // }
                // glEnd();		  
                }
                break;
            default:
                break;
        }
    } 
}

void GuiWorld::Draw( LightController& lightCTRL, const float color[3] ){
    glColor4f( color[0], color[1], color[2], 0.5 );
    for( std::vector<Light*>::iterator it = lightCTRL.lights.begin(); it != lightCTRL.lights.end(); it++ ){
        b2Vec2 light = (*it)->GetCenter();
        DrawDisk( light.x, light.y, lightCTRL.radiusSmall );
        DrawDisk( light.x, light.y, lightCTRL.radiusLarge );
    }
}

void GuiWorld::Draw( const std::vector<Box*>& bodies, const float color[3] ){
    for( int i=0; i<bodies.size(); i++ )
        DrawBody( bodies[i]->body, color );
}

void GuiWorld::Draw( const std::vector<Robot*>& robots, const float color[3] ){
    for( int i=0; i<robots.size(); i++ ) {
        DrawBody( robots[i]->body, color );
        DrawBody( robots[i]->bumper, c_darkred );
    }
    // draw a nose on the robot
    glColor3f( 1,1,1 );
    glPointSize( 12 );
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    glBegin( GL_TRIANGLES );
    for( int i=0; i<robots.size(); i++ ) {      
        const b2Transform& t = robots[i]->body->GetTransform();
        const float a = t.q.GetAngle();

        glVertex2f( t.p.x + Robot::size/2.0 * cos(a),
          t.p.y + Robot::size/2.0 * sin(a) );		  
        glVertex2f( t.p.x + Robot::size/3.0 * cos(a+0.5),
          t.p.y + Robot::size/3.0 * sin(a+0.5) );		  
        glVertex2f( t.p.x + Robot::size/3.0 * cos(a-0.5),
          t.p.y + Robot::size/3.0 * sin(a-0.5) );		  
    }
    glEnd();
}

void GuiWorld::Draw( const std::vector<b2Body*>& walls, const float color[3] ){
    for( int i=0; i<walls.size(); i++ )
        DrawBody( walls[i], color );
}

void GuiWorld::Draw( const std::vector<Goal*>& goals, const float color[3] ){
    glColor3fv( color );
    for (int i=0; i<goals.size(); ++i){
        b2Vec2 goal = goals[i]->GetCenter();
        DrawCircle(goal.x, goal.y, goals[i]->r);
    }
}

GuiWorld::GuiWorld( float width, float height, float boxDiam, int numRobots, int numBoxes ) : 
    World( width, height, boxDiam, numRobots, numBoxes ),
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
        Draw( lightCTRL, c_light );
        Draw( goals, c_green );

        // ===> DRAW: field objects
        glPolygonOffset( 0, 0 ); 
        Draw( boxes, c_gray );
        Draw( robots, c_red );
        Draw( groundBody, c_black );

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
