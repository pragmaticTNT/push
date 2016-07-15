#include <stdio.h>  // printf

#include <getopt.h> // getopt_long: reading in cmd line args
#include <iostream>
#include <string>
#include <unistd.h> // usleep(3)

#include "push.hh"

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
    std::string goalFile = "goals.txt";
    char* gValue = NULL;
    float32 timeStep = 1.0 / 30.0;

    /* options descriptor */
    static struct option longopts[] = {
        //  { "width",  optional_argument,   NULL,  'w' },
        //  { "height",  optional_argument,   NULL,  'h' },
        { "robots",  required_argument,   NULL,  'r' },
        { "boxes",  required_argument,   NULL,  'b' },
        { "robotsize",  required_argument,   NULL,  'z' },
        { "boxsize",  required_argument,   NULL,  's' },
        //  { "goalFileName",  optional_argument,   NULL,  'g' },
        //	{ "help",  optional_argument,   NULL,  'h' },
        { NULL, 0, NULL, 0 }
    };

    int ch=0, optindex=0;  
    while ((ch = getopt_long(argc, argv, "w:h:r:b:s:z:g:", longopts, &optindex)) != -1) {
        switch( ch ) {
            case 0: // long option given
                printf( "option %s given", longopts[optindex].name );
                if (optarg)
                printf (" with arg %s", optarg);
                printf ("\n");
                break;
            case 'w':
                WIDTH = atoi( optarg ); break;
            case 'h':
                HEIGHT = atoi( optarg ); break;
            case 'r':
                ROBOTS = atoi( optarg ); break;
            case 'b':
                BOXES = atoi( optarg ); break;
            case 'z':
                Robot::size = atof( optarg ); break;
            case 's':
                Box::size = atof( optarg ); break;
            case 'g':
                goalFile = std::string(optarg);
                break;
                // case 'h':  
            case '?':  
                if( optopt == 'c' )
                    std::cout << "[ERR] option -c requires an argument\n";
                else
                    std::cout << "[ERR] unhandled option: " << ch << std::endl;
                exit(0);
        }
    }

    GuiWorld world( WIDTH, HEIGHT, Robot::size, ROBOTS, BOXES, goalFile);
    /* Loop until the user closes the window */
    while( !world.RequestShutdown() ) {
        world.Step( timeStep );
    }

    return 0;
}
