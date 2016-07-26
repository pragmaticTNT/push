#include <iostream>
#include <fstream>
#include <sstream>

#include "push.hh"

// ===> Set Static Variables
float Box::size;
float Robot::size;
float Light::radiusSmall;
float Light::radiusLarge;

// ===> PUSHER class static variables
const float Pusher::PUSH = 10.0; // seconds
const float Pusher::BACKUP = 0.1;
const float Pusher::TURNMAX = 1.5;
const float Pusher::SPEEDX = 0.5;
const float Pusher::SPEEDA = M_PI/2.0;
const float Pusher::maxspeedx = 0.5;
const float Pusher::maxspeeda = M_PI/2.0;

// ===> COLORS
const float c_yellow[3]     = {1.0, 1.0, 0.0};
const float c_light[3]      = {0.8, 0.8, 0.8};
const float c_red[3]        = {1.0, 0.0, 0.0};
const float c_blue[3]       = {0.0, 1.0, 0.0};
const float c_green[3]      = {0.0, 0.0, 1.0};
const float c_darkred[3]    = {0.8, 0.0, 0.0};
const float c_tan[3]        = {0.8, 0.6, 0.5};
const float c_gray[3]       = {0.9, 0.9, 1.0};
const float c_black[3]      = {0.0, 0.0, 0.0};


// ===> WORLDOBJECT class methods
void WorldObject::DrawDisk( b2Vec2 center, float radius ) { 
    if( radius > EPSILON ){
        const int num_segments = 32.0 * sqrtf(radius);
        const float theta = 2 * M_PI / float(num_segments); 
        const float c = cosf(theta); 
        const float s = sinf(theta);
        float t;

        float x = radius; //we start at angle = 0 
        float y = 0; 

        //glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
        glBegin(GL_TRIANGLE_STRIP); 
            for(int ii = 0; ii < num_segments; ii++) { 
                glVertex2f( x + center.x, y + center.y); 
                glVertex2f( center.x, center.y ); 

                //apply the rotation matrix
                t = x;
                x = c * x - s * y;
                y = s * t + c * y;
            }
        glVertex2f( radius+center.x, 0+center.y ); // first point again to close disk
        glEnd();
    }
}

void WorldObject::DrawCircle( b2Vec2 center, float radius) {
    const int lineAmount = 32.0 * sqrtf(radius);
    float twicePi = M_PI * 2.0f;

    glBegin(GL_LINE_LOOP);
        for( int i = 0; i < lineAmount; i++ ){
            glVertex2f(
                center.x + (radius*cosf(i*twicePi/lineAmount)),
                center.y + (radius*sinf(i*twicePi/lineAmount))
            );
        }
    glEnd();
}

void WorldObject::DrawBody( b2Body* b, const float color[3] ) {
    for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext()){
        switch( f->GetType() ){
            case b2Shape::e_circle:
                {
                b2CircleShape* circle = (b2CircleShape*)f->GetShape();
                b2Vec2 position = b->GetWorldCenter();
                glColor3fv( color );
                DrawDisk( position, circle->m_radius ); 
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
// ===> END WORLDOBJECT class methods


// ===> WORLD class methods
World::World( float worldWidth, float worldHeight, float spawnDist, size_t robotNum, size_t boxNum, const std::string& goalFile, box_shape_t boxShape, float lightAvoidIntensity, float lightBufferIntensity, float lightSmallRadius ) :
    width(worldWidth),
    height(worldHeight),
    numBoxes(boxNum),
    spawnDist(spawnDist),
    lightAvoidIntensity(lightAvoidIntensity),
    lightBufferIntensity(lightBufferIntensity),
    lightCTRL(LightController(lightAvoidIntensity, lightBufferIntensity, lightSmallRadius)),
    b2world( new b2World( b2Vec2( 0,0 ))) // gravity 
{
    std::vector<std::string> params;
    AddGoals(goalFile, params);     // modifies params

    AddBoundary();
    AddRobots(robotNum);
    AddBoxes(numBoxes, boxShape);

    //TODO: add comment listing the input parameters of the goal files
    if( params.size() >= 4 ){
        float goalError, growRate, trialTime; 
        float patternTime = -1;
        bool repeat = false;
        numPatterns = (size_t) std::stoi(params[0]);
        trialTime = std::stof(params[1]);
        goalError = std::stof(params[2]);
        growRate = std::stof(params[3]);
        if( params.size() >= 6 && numPatterns > 1){
            patternTime = std::stof(params[4]);
            repeat = std::stoi(params[5]) == 0 ? false : true;
        }
        lightCTRL.SetGoals(goals, boxes, goalError, growRate, trialTime, patternTime, repeat);
    } else {
        std::cout << "[ERR] Goal file does not have enough parameters." << std::endl;
    }

    std::cout << "Starting " << worldWidth << "x" << worldHeight << " World... robots: " << robotNum << " boxes: " << boxNum << std::endl;
}

void World::AddBoundary( void ){
    b2BodyDef groundBodyDef;
    groundBodyDef.type = b2_staticBody;
    b2PolygonShape groundBox;
    groundBox.SetAsBox( width/2.0, 0.05f );    
    
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
        b2Vec3( spawnDist, spawnDist, 3*M_PI/4.0 ), 
        b2Vec3( width-spawnDist, spawnDist, M_PI/4.0 ), 
        b2Vec3( width-spawnDist, height-spawnDist, 3*M_PI/4.0 ), 
        b2Vec3( spawnDist, height-spawnDist, M_PI/4.0 ) 
    };

    for( b2Vec3 wall : boundary ){
        b2Body* wallBody = b2world->CreateBody(&groundBodyDef); 
        wallBody->CreateFixture(&groundBox, 0.05f); // Param2 is density
        groundBody.push_back( new Wall(wallBody, wall) );
    }
    
    // TODO: set the no spawn boundary property based on wall size
    // spawnDist = 0; // Do some calculations
}

void World::AddRobots( size_t numRobots ){
    for( int i=0; i<numRobots; i++ )
        robots.push_back( new Pusher( *this, spawnDist ) );
}

void World::AddBoxes( size_t numBoxes, box_shape_t shape ){
    for( int i=0; i<numBoxes; i++ )
        boxes.push_back( new Box(*this, spawnDist, shape) );
}

void World::AddGoals( const std::string& fileName, std::vector<std::string>& parameters ){
    bool setParameters = false;
    if( fileName.empty() ){ // DEFAULT
        std::cout << "[WARNING] No file name given. Using DEFAULT.";
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
                        numBoxes = (size_t) std::stoi(x);
                        Box::size = std::stof(y);
                        while( ss >> param ){
                            parameters.push_back(param);
                        }
                        setParameters = true;
                    } else { 
                        goals.push_back( new Goal( std::stof(x), std::stof(y), Box::size/2) );
                    }
                }
            }
            goalFile.close();
        } else {
            std::cout << "[ERR] Unable to open file: " << fileName << std::endl;
        }
    }
}

World::~World(){
    for( auto goal : goals )
        delete goal;
    for( auto box : boxes )
        delete box;
    for( auto robot : robots )
        delete robot;
    for( auto wall : groundBody )
        delete wall;
}

float World::GetLightIntensity( const b2Vec2& here ){
    return lightCTRL.GetIntensity( here.x, here.y );
}

void World::Step( float timestep ){
    const int32 velocityIterations = 6;
    const int32 positionIterations = 2;

    for( int i=0; i<robots.size(); ++i){
        robots[i]->Update( timestep, *this );
    }
    lightCTRL.Update(goals, boxes, timestep); 

    // Instruct the world to perform a single step of simulation.
    // It is generally best to keep the time step and iterations fixed.
    b2world->Step( timestep, velocityIterations, positionIterations);   
}
// ===> END WORLD class methods


// ===> WALL class methods
Wall::Wall( b2Body* body, b2Vec3& dim ) :
    WorldObject(dim.x, dim.y),
    body(body)
{
    if (body){
        body->SetTransform( b2Vec2( dim.x, dim.y ), dim.z );
    } else {
        std::cout << "[ERR] WALL: body parameter is NULL.";
    }
}

void Wall::Draw( void ){
    DrawBody( body, c_black );
}
// ===> END WALL class methods


// ===> GOAL class methods
void Goal::Draw( void ){
    glColor3fv( c_green );
    DrawCircle( GetCenter(), radius );
}
// ===> END GOAL class methods


// ===> BOX class methods
Box::Box( World& world, float spawnDist, box_shape_t shape ) :
    WorldObject(0,0),
    body(NULL) 
{
    b2FixtureDef fixtureDef;
    b2PolygonShape dynamicPolygon;
    b2CircleShape dynamicCirc;

    switch( shape ){
        case SHAPE_RECT:
            dynamicPolygon.SetAsBox( size/2.0, size/2.0 );
            fixtureDef.shape = &dynamicPolygon;
            break;
        case SHAPE_CIRC:
            dynamicCirc.m_p.Set(0,0);
            dynamicCirc.m_radius = size/2.0f;
            fixtureDef.shape = &dynamicCirc;
            break;
        case SHAPE_HEX:
            {
                b2Vec2 verts[6];
                for (int i=0; i<6; ++i){
                    verts[i].x = size/2.0 * cos(2.0*M_PI*i/6.0);
                    verts[i].y = size/2.0 * sin(2.0*M_PI*i/6.0);
                }
                dynamicPolygon.Set(verts, 6);
                fixtureDef.shape = &dynamicPolygon;
                break;
            }
        default:
            std::cout << "Invalid shape number " << shape << std::endl;
            break;
    }

    fixtureDef.density = 2.0;
    fixtureDef.friction = 1.0;
    fixtureDef.restitution = 0.1; 
    
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    
    body = world.b2world->CreateBody(&bodyDef);    
    body->SetLinearDamping( 10.0 );
    body->SetAngularDamping( 10.0 );
    body->SetTransform( b2Vec2( drand48()*(world.width-2*spawnDist) + spawnDist, drand48()*(world.height-2*spawnDist) + spawnDist ), 0 ); 
      
    body->CreateFixture(&fixtureDef);
    center = (body->GetWorldCenter());
}

void Box::Draw( void ){
    GetCenter();
    DrawBody( body, c_gray );
}
// ===> END BOX class methods


// ===> LIGHT class methods
void Light::Draw( void ){
    glColor4f( c_light[0], c_light[1], c_light[2], 0.5 );
    DrawDisk( GetCenter(), radiusSmall );
    DrawDisk( GetCenter(), radiusLarge );
}
// ===> END LIGHT class methods


// ===> ROBOT class methods
Robot::Robot( World& world, float x, float y, float angle ) : 
    WorldObject(0,0),
    body( NULL ),
    joint( NULL ) 
{
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    body = world.b2world->CreateBody(&bodyDef);
    bumper = world.b2world->CreateBody(&bodyDef);

    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox( size/2.0, size/2.0 );

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;    
    fixtureDef.density = 10;
    fixtureDef.friction = 1.0;
    body->CreateFixture(&fixtureDef);

    // bumper has same settings the body but different size
    dynamicBox.SetAsBox( size/10.0, size/2.0 );
    bumper->CreateFixture(&fixtureDef);

    b2PrismaticJointDef jointDef;

    jointDef.Initialize( 
        body, 
        bumper, 
        body->GetWorldCenter(), 
        b2Vec2( 1.0f, 0.0f )
    ); 

    jointDef.lowerTranslation = 0;//-0.2;
    jointDef.upperTranslation = 0.04f;
    jointDef.enableLimit = true;
    jointDef.maxMotorForce = 0.8f;
    jointDef.motorSpeed = 1.0f;
    jointDef.localAnchorA.Set( size/2.0, 0); // on the nose
    jointDef.localAnchorB.Set( 0,0 );

    jointDef.enableMotor = true;
    //jointDef.collideConnected = true;

    joint = (b2PrismaticJoint*)world.b2world->CreateJoint( &jointDef );

    // place assembled robot in the world
    body->SetTransform( b2Vec2( x, y ), angle );	
    bumper->SetTransform( body->GetWorldPoint( b2Vec2( size/2,0) ), angle );	
    GetCenter();
}

bool Robot::isBumperPressed( void ) {
    return joint->GetJointTranslation() < 0.01;
}

// set body speed in body-local coordinate frame
void Robot::SetSpeed( float x, float y, float angle ) {  
    body->SetLinearVelocity( body->GetWorldVector(b2Vec2(x,y)) );
    body->SetAngularVelocity( angle );
}

void Robot::Draw( void ){
    DrawBody( body, c_red );
    DrawBody( bumper, c_darkred );

    // draw a nose on the robot
    glColor3f( 1,1,1 );
    glPointSize( 12 );
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

    glBegin( GL_TRIANGLES );
        const b2Transform& t = body->GetTransform();
        const float a = t.q.GetAngle();

        glVertex2f( t.p.x + Robot::size/2.0 * cos(a),
          t.p.y + Robot::size/2.0 * sin(a) );		  
        glVertex2f( t.p.x + Robot::size/3.0 * cos(a+0.5),
          t.p.y + Robot::size/3.0 * sin(a+0.5) );		  
        glVertex2f( t.p.x + Robot::size/3.0 * cos(a-0.5),
          t.p.y + Robot::size/3.0 * sin(a-0.5) );		  
    glEnd();
}
// ===> END ROBOT class methods


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
            default:
                std::cout << "invalid control state: " << state << std::endl;
                exit(1);
        }
    SetSpeed( speedx, 0, speeda );
    lightIntensity = currentLightIntensity;
}
// ===> END PUSHER class methods


// ===> LIGHT CONTROLLER class methods
LightController::LightController( float avoidIntensity, float bufferIntensity, float radiusSmall ) :
    avoidIntensity(avoidIntensity),
    bufferIntensity(bufferIntensity),
    radiusSmall(radiusSmall),
    radiusInit(radiusSmall),
    scaleFactor(GetScaleFactor(radiusSmall)),
    radiusLarge(GetRadiusLarge()),
    goalError(0),
    growRate(0),
    timeElapsed(0),
    maxRadius(0),
    isFilled(false),
    startScramble(false),
    scramble(false)
{
    Light::radiusSmall = radiusSmall;
    Light::radiusLarge = radiusLarge;
}

LightController::~LightController(){
    for( auto light : lights )
        delete light;
}

float LightController::GetIntensity( float x, float y ){
    float brightness = 1.0;
    for( auto light : lights ){
        float intensity = 1-1.0/(1+scaleFactor*(light->SqrDistanceTo(x,y)));
        brightness = std::min(brightness, intensity);
        //brightness *= 1-1.0/(1+scaleFactor*(light->SqrDistanceTo(x,y)));
    }
    return scramble ? 1.0 : brightness;
}

void LightController::SetGoals( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes, float goalError, float growRate, float trialWaitTime, float patternWaitTime, bool repeatPattern ){
    this->goalError = goalError;
    this->growRate = growRate;
    this->trialTime = trialWaitTime;
    this->patternTime = patternWaitTime;
    this->repeatPattern = repeatPattern;
    maxRadius = UpdateGoalsInfo( goals, boxes );
    for( auto goal : goals ){
        lights.push_back( new Light(goal->GetCenter()) );
    }
}

float LightController::UpdateGoalsInfo( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes ){
    float minAmongBoxes = INT_MAX, maxAmongGoals = 0;
    for( Goal* goal: goals )
        goal->GetCenter();
    for( Box* box : boxes ){
        box->GetCenter();
        minAmongBoxes = INT_MAX;
        for( Goal* goal : goals ){
            float dist = sqrt(box->SqrDistanceTo(*goal));
            minAmongBoxes = std::min(minAmongBoxes, dist);
            goal->filled = dist < goalError;
        }
        maxAmongGoals = std::max(maxAmongGoals, minAmongBoxes);
    }
    return maxAmongGoals;
}

// -> Light off-center to the boxes
void LightController::Update( 
        const std::vector<Goal*>& goals,
        const std::vector<Box*>& boxes){ 
    for( int i=0; i<std::min(goals.size(),boxes.size()); ++i ){
        b2Vec2 goal = goals[i]->GetCenter();
        b2Vec2 box = boxes[i]->GetCenter();
        float distBoxToGoal = sqrt(boxes[i]->SqrDistanceTo(*goals[i]));

        if(distBoxToGoal > goalError){
            float x = (goal.x-box.x)*radiusSmall/distBoxToGoal + box.x;
            float y = (goal.y-box.y)*(x-box.x)/(goal.x-box.x) + box.y;
            lights[i]->SetCenter(x,y);
        } else {
            goals[i]->filled = true;
            lights[i]->SetCenter(goal);
            //std::cout << "[GOAL REACHED]" << std::endl;
        }
        //goals[i]->WhereAmI();
        //boxes[i]->WhereAmI();
        //lights[i]->WhereAmI();
    }
}

// -> Light centered at the goal slowly diminishing
void LightController::Update(
        const std::vector<Goal*>& goals,
        const std::vector<Box*>& boxes,
        float timeStep ){

    timeElapsed += timeStep;
    if( !scramble ){
        radiusSmall = std::max(radiusInit, maxRadius + growRate*timeElapsed);
        scaleFactor = GetScaleFactor(radiusSmall);
        radiusLarge = GetRadiusLarge();
        Light::radiusSmall = radiusSmall;
        Light::radiusLarge = radiusLarge; 
    }

    if( timeElapsed > trialTime ){
        timeElapsed = 0;
        maxRadius = UpdateGoalsInfo( goals, boxes );
        if( !(maxRadius < goalError) ){
            std::cout << "Current max error: " << maxRadius << std::endl;
            if( maxRadius < radiusInit || scramble ){
                if( !startScramble ){
                    startScramble = true;
                } else if ( !scramble ){
                    std::cout << "Local minimum reached. Scrambling..." << std::endl;
                    scramble = true; 
                    Light::radiusSmall = 0;
                    Light::radiusLarge = 0;
                } else{
                    startScramble = false;
                    scramble = false;
                }
            }
        } else {
            isFilled = true;
            std::cout << "Goal shapes obtained." << std::endl;
            // May need to keep on checking to maintain shape
        }
    }
}
// ===> END LIGHTCONTROLLER class methods
