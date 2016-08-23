#include <stdio.h>  // printf

#include <getopt.h> // getopt_long: reading in cmd line args
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h> // usleep(3)

#include "push.hh"

/* Long Options Descriptor */
static struct option longopts[] = {
    { "width",  required_argument,   NULL,  'w' },
    { "length",  required_argument,   NULL,  'l' },
    { "robots",  required_argument,   NULL,  'r' },
    { "boxes",  required_argument,   NULL,  'b' },
    { "robotsize",  required_argument,   NULL,  'z' },
    { "boxsize",  required_argument,   NULL,  's' },
    { "boxshape", required_argument, NULL, 'p' },
    { "goalFileName",  required_argument,   NULL,  'g' },
    { "help",  optional_argument,   NULL,  'h' },
    { NULL, 0, NULL, 0 }
};

void PrintReadMe( const char* filename ){
    std::cout << '\n';
    std::ifstream readme( filename );
    std::string line;
    if( readme.is_open() ){
        while( std::getline(readme, line) )
            std::cout << line << '\n';
        readme.close();
    }
    std::cout << '\n';
}

int main( int argc, char* argv[] ){
    char shapeid;
    std::string goalfile;

    float width = 5, height = 5;
    int robots = 20, boxes = 10;
    box_shape_t shape = SHAPE_CIRC;
    float32 timestep = 1.0 / 30.0;

    int ch=0, optindex=0; 
    while( (ch = getopt_long(argc, argv, "w:l:r:b:s:z:g:p:h", 
                    longopts, &optindex)) != -1 ){
        switch( ch ) {
            case 0: // long option given
                printf( "option %s given", longopts[optindex].name );
                if (optarg)
                    printf (" with arg %s\n", optarg);
                break;
            case 'w':
                width = atoi( optarg ); break;
            case 'l':
                height = atoi( optarg ); break;
            case 'r':
                robots = atoi( optarg ); break;
            case 'b':
                boxes = atoi( optarg ); break;
            case 'z':
                Robot::size = atof( optarg ); break;
            case 's':
                Box::size = atof( optarg ); break;
            case 'g':
                goalfile = std::string(optarg);
                break;
            case 'p':
                shapeid = toupper(optarg[0]);
                if( shapeid == 'S')
                    shape = SHAPE_RECT; 
                else if( shapeid == 'H')
                    shape = SHAPE_HEX;
                break;
            case 'h':
                PrintReadMe("README.md");
                exit(0);
            case '?':  
                std::cout << "[ERR] unhandled option: " << ch << std::endl;
                exit(1);
        }
    }
    GuiWorld world = argc < 4 and argc > 1 ? 
        GuiWorld( "templates/" + goalfile ) :
        GuiWorld( width, height, robots, boxes, shape );

    /* Loop until the user closes the window */
    while( !world.RequestShutdown() ) {
        world.Step( timestep );
    }

    return 0;
}
