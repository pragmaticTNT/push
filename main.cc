#include <stdio.h>  // printf

#include <iostream>
#include <getopt.h> // getopt_long: reading in cmd line args
#include <unistd.h> // usleep(3)

#include "push.hh"

//class Pusher : public Robot {
//    private:
//        typedef enum {
//            S_PUSH = 0,
//            S_BACKUP_LONG,
//            S_BACKUP_SHORT,
//            S_TURN,
//            S_COUNT
//        } control_state_t;
//
//        static const float PUSH, BACKUP, TURNMAX;
//        static const float SPEEDX, SPEEDA;
//        static const float maxspeedx, maxspeeda;
//
//        float timeleft;
//        control_state_t state;
//        float speedx, speeda;
//        float epsilon;   // spawn away from wall
//        int turnRight;
//
//    public:
//    // constructor
//    Pusher( World& world, float epsilon ) : 
//        epsilon(epsilon),
//        Robot( world, 
//            drand48() * (world.width - 2*epsilon) + epsilon,
//            drand48() * (world.height - 2*epsilon) + epsilon, 
//            -M_PI + drand48() * 2.0*M_PI), 
//        state( S_TURN ),
//        timeleft( drand48() * TURNMAX ),
//        speedx( 0 ),
//        speeda( 0 ) {
//        turnRight = drand48() < 0.5 ? 1 : -1; 
//        }
//
//    virtual void Update( float timestep ) {
//        // ===> IMPLEMENT ROBOT BEHAVIOUR WITH A LITTLE STATE MACHINE
//        b2Vec2 here = body->GetWorldCenter();
//        float currentLightIntensity = lightCTRL->GetIntensity(here.x, here.y);
//        // count down to changing control state
//        timeleft -= timestep;
//
//        // force a change of control state
//        if( state == S_PUSH && 
//            (currentLightIntensity < lightCTRL->GetSmallLight() || GetBumperPressed()) ){
//            timeleft = 0.0; // end pushing right now
//            speedx = 0;
//            speeda = 0;
//        }
//
//        if( timeleft <= 0 ) // time to change to another behaviour
//            switch( state ) {
//                case S_PUSH:
//                    //std::cout << "In state: BACKUP << std::endl;
//                    state = currentLightIntensity < lightCTRL->GetSmallLight() ? 
//                        S_BACKUP_LONG : S_BACKUP_SHORT;
//                    timeleft = currentLightIntensity < lightCTRL->GetSmallLight() ?
//                        0 : BACKUP;
//                    speedx = -SPEEDX;
//                    speeda = 0;	          
//                    break;
//                case S_BACKUP_LONG: 
//                    if( currentLightIntensity > lightCTRL->GetLargeLight() || fabs(timeleft) > 10 * BACKUP )
//                        state = S_BACKUP_SHORT;
//                    else if( currentLightIntensity < lightIntensity )
//                        speedx = -speedx;
//                    break;
//                case S_BACKUP_SHORT: 
//                    //std::cout << "In state: TURN" << std::endl;
//                    state = S_TURN;
//                    timeleft = drand48() * TURNMAX;
//                    speedx = 0;
//                    speeda = turnRight * SPEEDA;	    
//                    break;
//                case S_TURN: 
//                    //std::cout << "In state: PUSH" << std::endl;
//                    state = S_PUSH;
//                    timeleft = PUSH;
//                    speedx = SPEEDX;
//                    speeda = 0;	 
//                    break;
//                default:
//                    std::cout << "invalid control state: " << state << std::endl;
//                    exit(1);
//            }
//        SetSpeed( speedx, 0, speeda );
//        lightIntensity = currentLightIntensity;
//    }
//}; // class Pusher

// static members
const float Pusher::PUSH = 10.0; // seconds
const float Pusher::BACKUP = 0.1;
const float Pusher::TURNMAX = 1.5;
const float Pusher::SPEEDX = 0.5;
const float Pusher::SPEEDA = M_PI/2.0;
const float Pusher::maxspeedx = 0.5;
const float Pusher::maxspeeda = M_PI/2.0;

int main( int argc, char* argv[] ){
    float WIDTH = 5;
    float HEIGHT = 5;
    size_t ROBOTS = 16;
    size_t BOXES = 128;
    float32 timeStep = 1.0 / 30.0;

    /* options descriptor */
    static struct option longopts[] = {
        { "robots",  required_argument,   NULL,  'r' },
        { "boxes",  required_argument,   NULL,  'b' },
        { "robotsize",  required_argument,   NULL,  'z' },
        { "boxsize",  required_argument,   NULL,  's' },
        //	{ "help",  optional_argument,   NULL,  'h' },
        { NULL, 0, NULL, 0 }
    };

    int ch=0, optindex=0;  
    while ((ch = getopt_long(argc, argv, "r:b:s:z:", longopts, &optindex)) != -1) {
        switch( ch ) {
            case 0: // long option given
                printf( "option %s given", longopts[optindex].name );
                if (optarg)
                printf (" with arg %s", optarg);
                printf ("\n");
                break;
            case 'r':
                ROBOTS = atoi( optarg ); break;
            case 'b':
                BOXES = atoi( optarg ); break;
            case 'z':
                Robot::size = atof( optarg ); break;
            case 's':
                Box::size = atof( optarg ); break;
                // case 'h':  
                // case '?':  
                //   puts( USAGE );
                //   exit(0);
                //   break;
            default:
                printf("unhandled option %c\n", ch );
                //puts( USAGE );
                exit(0);
        }
    }

//    std::vector<Goal*> goals;
//    std::vector<Box*> boxes;    
//    std::vector<Robot*> robots;
//
//    // One time sort the goals by x --- then y --- coordinate please
//    goals.push_back( new Goal(3, 3, Box::size/2) );        
//    goals.push_back( new Goal(1, 2, Box::size/2) );        
//    //goals.push_back( new Goal(2, 2, Box::size/2) );        
//    //goals.push_back( new Goal(4, 4, Box::size/2) );        
//    //goals.push_back( new Goal(4, 1, Box::size/2) );        
//
//    LightController lightCTRL(goals, 0.1, 0.5, 0.6);
//
//    GuiWorld world( WIDTH, HEIGHT, Robot::size, lightCTRL );
//    std::cout << "Starting " << WIDTH << "x" << HEIGHT << " World... robots: " << ROBOTS << " boxes: " << BOXES << std::endl;
//
//    for( int i=0; i<BOXES; i++ )
//        boxes.push_back( new Box( world ) );
//
//    for( int i=0; i<ROBOTS; i++ )
//        robots.push_back( new Pusher( world, Box::size ) );
//    Robot::lightCTRL = &lightCTRL;

    GuiWorld world( WIDTH, HEIGHT, Robot::size, ROBOTS, BOXES);
    /* Loop until the user closes the window */
    while( !world.RequestShutdown() ) {
        // if( ! GuiWorld::paused )
        // for( int i=0; i<ROBOTS; i++ )
        //     robots[i]->Update( timeStep );
        // lightCTRL.Update(goals, boxes);
        world.Step( timeStep );
    }

    return 0;
}
