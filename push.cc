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
        -M_PI + drand48() * 2.0*M_PI ), 
    state( S_TURN ),
    timeleft( drand48() * TURNMAX ),
    speedx( 0 ),
    speeda( 0 ),
    turnRight( drand48() < 0.5 ? 1 : -1 ){}

void Pusher::Update( float timestep, World& world ) {
    // ===> IMPLEMENT ROBOT BEHAVIOUR WITH A STATE MACHINE
    center = GetCenter();
    float currentLuminance = world.GetLuminance(center);
    // printf("Current light intensity: %.4f\n", currentLuminance); 
    int backupRatio = 10;   // How much BACKUP_LONG differs from BACKUP_SHORT

    // count down to changing control state
    timeleft -= timestep;

    // force a change of control state
    if( state == S_PUSH && 
        (currentLuminance < world.avoidLuminance || isBumperPressed()) ){
        timeleft = 0.0; // end pushing right now
        speedx = 0;
        speeda = 0;
    }

    // // run out of batteries
    // if( currentLuminance < DEAD ){
    //     timeleft = 0.0;
    //     state = S_DEAD;
    // }

    if( timeleft <= 0 ) // change to another behaviour
        switch( state ) {
            case S_PUSH:
                state = currentLuminance < world.avoidLuminance ? 
                    S_BACKUP_LONG : S_BACKUP_SHORT;
                timeleft = currentLuminance < world.avoidLuminance ?
                    0 : BACKUP; // stop pushing because it got too dark
                speedx = -SPEEDX;
                speeda = 0; 
                break;
            case S_BACKUP_LONG: 
                if( currentLuminance > world.bufferLuminance || 
                    fabs(timeleft) > backupRatio * BACKUP )
                    state = S_BACKUP_SHORT;
                else if( currentLuminance < luminance )
                    speedx = -speedx;
                break;
            case S_BACKUP_SHORT: 
                state = S_TURN;
                timeleft = drand48() * TURNMAX;
                turnRight = drand48() < 0.5 ? 1 : -1;
                speedx = 0;
                speeda = turnRight * SPEEDA;        
                break;
            case S_TURN: 
                state = S_PUSH;
                timeleft = PUSH;
                speedx = SPEEDX;
                speeda = 0;  
                break;
            case S_DEAD:
                speedx = 0;
                speeda = 0;
                if( currentLuminance > DEAD ){
                    state = S_TURN;
                }
                break;
            default:
                printf("invalid control state: %i\n", state); 
                exit(1);
        }
    SetSpeed( speedx, 0, speeda );
    luminance = currentLuminance;
}
// ===> END PUSHER class methods
