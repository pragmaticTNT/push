#include <iostream>

#include "push.hh"

// ===> PUSHER class static variables
const float Pusher::PUSH = 10.0; // seconds
const float Pusher::BACKUP = 0.5;
const float Pusher::TURNMAX = 1.5;
const float Pusher::DEAD = 10*EPSILON;
const float Pusher::SPEEDX = 0.5;
const float Pusher::SPEEDA = M_PI/2.0;
const float Pusher::maxspeedx = 0.5;
const float Pusher::maxspeeda = M_PI/2.0;
const b2Vec2 Pusher::frontLightSensor = b2Vec2(Robot::size/4, 0);
const b2Vec2 Pusher::backLightSensor = b2Vec2(-Robot::size/4, 0);

// ===> PUSHER class methods
Pusher::Pusher( World& world, float spawnDist ) : 
    Robot( world, 
        drand48() * ((world.worldSet)->width-2*spawnDist) + spawnDist,
        drand48() * ((world.worldSet)->height-2*spawnDist) + spawnDist, 
        -M_PI + drand48() * 2.0*M_PI ), 
    state( S_PUSH ),
    timeleft( drand48() * TURNMAX ),
    speedx( 0 ),
    speeda( 0 ),
    turnRight( drand48() < 0.5 ? 1 : -1 ){}

bool Pusher::isBumperPressed( void ) const {
    return joint->GetJointTranslation() < 0.01;
}

// set body speed in body-local coordinate frame
void Pusher::SetSpeed( float x, float y, float angle ) {  
    body->SetLinearVelocity( body->GetWorldVector(b2Vec2(x,y)) );
    body->SetAngularVelocity( angle );
}

void Pusher::Update( float timestep, World& world ) {
    // ===> IMPLEMENT ROBOT BEHAVIOUR WITH A STATE MACHINE
    bool bright, facingLight;
    b2Vec2 frontPhotocell = GetPhotocell(Pusher::frontLightSensor);
    b2Vec2 backPhotocell = GetPhotocell(Pusher::backLightSensor);
    float luminanceFront = world.GetLuminance(frontPhotocell);
    float luminanceBack = world.GetLuminance(backPhotocell);

    // printf("Luminance front: %.4f back:%.4f\n", luminanceFront, luminanceBack); 
    
    // count down to changing control state
    timeleft -= timestep;

    if( luminanceFront < DEAD and luminanceBack < DEAD ){
        state = S_BACKUP;
        speedx = -SPEEDX;
        speeda = 0;
    }

    // printf( "State: %i Speedx: %.2f Speeda: %.2f\n", state, speedx, speeda );
    switch( state ) {
        case S_PUSH: // no time dependency
            if( isBumperPressed() or 
                luminanceFront < (world.worldSet)->avoidLuminance ){
                state = S_BACKUP;
                timeleft = BACKUP;
                speedx = -SPEEDX;
                speeda = 0; 
            }
            break;
        case S_BACKUP:
            if( luminanceFront > (world.worldSet)->avoidLuminance and
                    (timeleft < 0 or
                luminanceBack < (world.worldSet)->avoidLuminance) ){
                state = S_TURN;
                timeleft = drand48() * TURNMAX;
                speedx = 0;
                speeda = SPEEDA*turnRight;
            }
            break;
        case S_TURN:
            bright = luminanceFront > (world.worldSet)->avoidLuminance and
                luminanceBack > (world.worldSet)->avoidLuminance;
            facingLight = fabs(luminanceFront - luminanceBack) > 
                    (world.worldSet)->bufferLuminance and 
                    luminanceFront > luminanceBack;
            if( (bright or facingLight) and 
                !isBumperPressed() and timeleft < 0){ 
                state = S_PUSH; 
                speedx = SPEEDX;
                speeda = 0;
            }
            break;
        case S_DEAD:
            if( luminanceFront > DEAD or luminanceBack > DEAD ){
                state = S_TURN;
                timeleft = drand48() * TURNMAX;
                speedx = 0;
                speeda = SPEEDA*turnRight;
            }
            break;
        default:
            printf("invalid control state: %i\n", state); 
            exit(1);
    }
    if( state == S_PUSH or state == S_BACKUP )
        moveAmount += fabs(speedx)*timestep; 
    // printf("Current move amount: %.4f\n", moveAmount);
    SetSpeed( speedx, 0, speeda );
}
// ===> END PUSHER class methods
