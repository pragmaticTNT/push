#include <iostream>

#include "push.hh"

// ===> PUSHER class static variables
const float Pusher::PUSH = 10.0; // seconds
const float Pusher::BACKUP = 0.1;
const float Pusher::TURNMAX = 1.5;
const float Pusher::SPEEDX = 0.5;
const float Pusher::SPEEDA = M_PI/2.0;
const float Pusher::DEAD = 0.1;
const float Pusher::maxspeedx = 0.5;
const float Pusher::maxspeeda = M_PI/2.0;

// ===> PUSHER class methods
Pusher::Pusher( World& world, float spawnDist ) : 
    Robot( world, 
        drand48() * (world.width-2*spawnDist) + spawnDist,
        drand48() * (world.height-2*spawnDist) + spawnDist, 
        -M_PI + drand48() * 2.0*M_PI), 
    state( S_TURN ),
    timeleft( drand48() * TURNMAX ),
    speedx( 0 ),
    speeda( 0 ),
    turnRight(drand48() < 0.5 ? 1 : -1){}

void Pusher::Update( float timestep, World& world ) {
    // ===> IMPLEMENT ROBOT BEHAVIOUR WITH A STATE MACHINE
    center = GetCenter();
    float currentLightIntensity = world.GetLightIntensity(center);
    // std::cout << "Current light intensity: " << currentLightIntensity << std::endl;
    int backupRatio = 10;   // How much to backup

    // count down to changing control state
    timeleft -= timestep;

    // force a change of control state
    if( state == S_PUSH && 
        (currentLightIntensity < world.lightAvoidIntensity || isBumperPressed()) ){
        timeleft = 0.0; // end pushing right now
        speedx = 0;
        speeda = 0;
    }

    // if( currentLightIntensity < DEAD ){
    //     timeleft = 0.0;
    //     state = S_DEAD;
    // }

    if( timeleft <= 0 ) // time to change to another behaviour
        switch( state ) {
            case S_PUSH:
                //std::cout << "In state: BACKUP << std::endl;
                state = currentLightIntensity < world.lightAvoidIntensity ? 
                    S_BACKUP_LONG : S_BACKUP_SHORT;
                timeleft = currentLightIntensity < world.lightAvoidIntensity ?
                    0 : BACKUP;
                speedx = -SPEEDX;
                speeda = 0;           
                break;
            case S_BACKUP_LONG: 
                if( currentLightIntensity > world.lightBufferIntensity || fabs(timeleft) > backupRatio * BACKUP )
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
            case S_DEAD:
                speedx = 0;
                speeda = 0;
                if( currentLightIntensity > DEAD ){
                    state = S_TURN;
                }
                break;
            default:
                std::cout << "invalid control state: " << state << std::endl;
                exit(1);
        }
    SetSpeed( speedx, 0, speeda );
    lightIntensity = currentLightIntensity;
}
// ===> END PUSHER class methods
