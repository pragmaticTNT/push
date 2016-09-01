#include <stdio.h>  // printf

#include <getopt.h> // getopt_long: reading in cmd line args
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h> // usleep(3)

#include "sim.hh"

/* Long Options Descriptor */
static struct option longopts[] = {
    { "boxes",  required_argument,   NULL,  'b' },
    { "robots",  required_argument,   NULL,  'r' },
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
    std::string goalfile = "";
    int boxes = 0, robots = 0;
    int ch=0, optindex=0; 

    try {
        while( (ch = getopt_long(argc, argv, "b:r:g:h", 
                        longopts, &optindex)) != -1 ){
            switch( ch ) {
                case 0: // long option given
                    printf( "option %s given", longopts[optindex].name );
                    if (optarg)
                        printf (" with arg %s\n", optarg);
                    break;
                case 'b': boxes = atoi( optarg ); break;
                case 'r': robots = atoi( optarg ); break;
                case 'g': goalfile = std::string(optarg); break;
                case 'h': PrintReadMe("README.md"); break;
                case '?': throw "[ERR] unhandled option: " + ch;
            }
        }
        
        Sim simulator = goalfile.compare("") == 0 ?
            Sim( boxes, robots ) :
            Sim( goalfile );

        simulator.Run();
    } catch (const std::exception& e){
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    } catch (const char* msg){
        std::cerr << msg << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
