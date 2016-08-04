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
const float c_green[3]      = {0.0, 1.0, 0.0};
const float c_lightblue[3]  = {0.5, 0.5, 1.0};
const float c_blue[3]       = {0.0, 0.0, 1.0};
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
World::World( float worldWidth, float worldHeight, float spawnDist, size_t robotNum, size_t boxNum, const std::string& goalFile, box_shape_t boxShape ) :
    width(worldWidth),
    height(worldHeight),
    numBoxes(boxNum),
    spawnDist(spawnDist),
    lightAvoidIntensity(0.2),
    lightBufferIntensity(0.4),
    lightCTRL(NULL),
    b2world( new b2World( b2Vec2( 0,0 )) ) // gravity 
{
    float lightSmallRadius = 0.5;
    std::vector<float> settings;

    ParseFile(goalFile, settings);     // modify settings & adds goals
    AddBoundary();
    AddRobots(robotNum);
    AddBoxes(numBoxes, boxShape);
    std::cout << "Starting " << worldWidth << "x" << worldHeight << " World... robots: " << robotNum << " boxes: " << boxNum << std::endl;
    
    // lightCTRL = new RadialLightController(goals, boxes, settings, lightAvoidIntensity, lightBufferIntensity, lightSmallRadius);
    if( settings.size() < 4 ){
        std::cout << "[Err] World Constructor: grid dimension not added\n";
        return;
    }
    lightSmallRadius = settings.back()*sqrt(2)/2.0;
    lightCTRL = new GridLightController(goals, settings, lightAvoidIntensity, lightBufferIntensity, lightSmallRadius);
}

World::~World(){
    delete lightCTRL;
    for( auto goal : goals )
        delete goal;
    for( auto box : boxes )
        delete box;
    for( auto robot : robots )
        delete robot;
    for( auto wall : groundBody )
        delete wall;
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

void World::ParseFile( const std::string& fileName, std::vector<float>& settings ){
    bool setParameters = false;
    float radius = 1.0;
    float cellWidth = 1.0;
    size_t row, col, index;
    float xCoord, yCoord;

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
                        width = std::stof(x);
                        height = std::stof(y);
                        ss >> x >> y;
                        numBoxes = (size_t) std::stoi(x);
                        Box::size = std::stof(y);
                        while( ss >> param ){
                            settings.push_back(std::stof(param));
                        }
                        setParameters = true;
                        numPatterns = settings.back();
                        settings.pop_back();
                        cellWidth = width/settings.back();
                        radius = cellWidth/2.0;
                        // std::cout << "Cell width is: " << cellWidth << std::endl;
                        settings.push_back(cellWidth);
                    } else { 
                        row = std::stoi(x);
                        col = std::stoi(y);
                        index = row*settings[settings.size()-2] + col; 
                        xCoord = col*cellWidth + cellWidth/2.0;
                        yCoord = row*cellWidth + cellWidth/2.0; 
                        goals.push_back( new Goal(xCoord, yCoord, radius, index) );
                    }
                }
            }
            goalFile.close();
        } else {
            std::cout << "[ERR] Unable to open file: " << fileName << std::endl;
        }
    }
}

float World::GetLightIntensity( const b2Vec2& here ){
    return lightCTRL->GetIntensity( here.x, here.y );
}

void World::Step( float timestep ){
    const int32 velocityIterations = 6;
    const int32 positionIterations = 2;

    for( int i=0; i<robots.size(); ++i){
        robots[i]->Update( timestep, *this );
    }
    lightCTRL->Update(goals, boxes, timestep); 

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
    glColor3fv( c_lightblue );
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
    DrawBody( body, c_blue );
}
// ===> END BOX class methods


// ===> LIGHT class methods
void Light::Draw( void ){
    if( !on ){
        glColor4f( c_light[0], c_light[1], c_light[2], 0.7 );
        DrawDisk( GetCenter(), radiusSmall );
        DrawDisk( GetCenter(), radiusLarge );
    }
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


// ===> LIGHTCONTROLLER class methods
LightController::LightController( float avoidIntensity, float bufferIntensity, float radiusSmall ) : 
    avoidIntensity( avoidIntensity ),
    bufferIntensity( bufferIntensity ),
    radiusSmall(radiusSmall),
    scaleFactor(GetScaleFactor(radiusSmall)),
    radiusLarge(GetRadiusLarge()),
    goalError(0),
    isFilled(false)
{ 
    Light::radiusSmall = radiusSmall;
    Light::radiusLarge = radiusLarge;
}

LightController::~LightController(){
    for( auto light : lights )
        delete light;
}
// ===> END LIGHTCONTROLLER class methods


// ===> RADIALLIGHTCONTROLLER class methods
RadialLightController::RadialLightController( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes, const std::vector<float>& settings, float avoidIntensity, float bufferIntensity, float radiusSmall ) :
    LightController( avoidIntensity, bufferIntensity, radiusSmall ),
    radiusInit(radiusSmall),
    growRate(0),
    timeElapsed(0),
    maxRadius(0),
    startScramble(false),
    scramble(false)
{
    size_t numParams = settings.size();
    
    // When in doubt check the most recent pattern file
    goalError = numParams > 0 ? settings[0] : 0.2;
    trialTime = numParams > 1 ? settings[1] : 2000;
    growRate = numParams > 2 ? settings[2] : -10e-4;
    patternTime = numParams > 3 ? settings[3] : 0;
    repeatPattern = numParams > 4 ? fabs(settings[4]-1)<EPSILON : false;
    maxRadius = UpdateGoalsInfo( goals, boxes );
    for( auto goal : goals ){
        lights.push_back( new Light(goal->GetCenter()) );
    } 
}

float RadialLightController::UpdateGoalsInfo( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes ){
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

float RadialLightController::GetIntensity( float x, float y ){
    float brightness = 1.0;
    for( auto light : lights ){
        float intensity = light->GetIntensity(x, y, scaleFactor);
        brightness = std::min(brightness, intensity);
        //brightness *= 1-1.0/(1+scaleFactor*(light->SqrDistanceTo(x,y)));
    }
    return scramble ? 1.0 : brightness;
}

// -> Light off-center to the boxes
void RadialLightController::Update( 
        const std::vector<Goal*>& goals,
        const std::vector<Box*>& boxes)
{ 
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
void RadialLightController::Update( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes, float timeStep ){
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
// ===> END RADIALLIGHTCONTROLLER class methods


// ===> GRID LIGHT CONTROLLER class methods
GridLightController::GridLightController( const std::vector<Goal*>& goals, const std::vector<float>& settings, float avoidIntensity, float bufferIntensity, float cellWidth ) : 
    LightController( 
            avoidIntensity, 
            bufferIntensity, 
            cellWidth*sqrt(2)/2.0 ),
    dimCell(settings.back()),
    timeElapsed(0)
{
    int goalIndex;
    size_t numParams = settings.size();

    goalError = numParams > 0 ? settings[0] : 0.2;
    trialTime = numParams > 1 ? settings[1] : 500;
    dimGrid = numParams > 2 ? settings[2] : 11;
    dimWorld = numParams > 3 ? settings[3] : 10;
    layersActive = 4;
    currentLayer = dimGrid-1;
    
    std::cout << "[GLC] Setting Goals...\n";
    for( int row = 0; row < dimGrid; ++row )
        for( int col = 0; col < dimGrid; ++col )
            lights.push_back( new Light(GetLightCenter(row, col)) );

    // Setting goal lights
    for( auto goal : goals ){
        goalIndex = goal->index;
        lights[goalIndex]->atGoal = true;
    }

    PreprocessLayers(goals);
    SeeLayerDistribution();
    for( int i = 0; i < layersActive; ++i ){
        Toggle(currentLayer);
        currentLayer = NextLayer(currentLayer);
    }
    currentLayer = dimGrid-1;
}

// RETURN: index of n-bour if exists otherwise index of cell 
int GridLightController::NeighbourIndex( int cell[2], neighbour_t n ){
    int row, col;
    switch(n){
        case TR: row = -1; col = 1; break; 
        case TC: row = -1; col = 0; break;
        case TL: row = -1; col = -1; break;
        case CL: row = 0; col = -1; break;
        case BL: row = 1; col = -1; break;
        case BC: row = 1; col = 0; break;
        case BR: row = 1; col = 1; break;
        case CR: row = 0; col = 1; break;
    }
    return RowColToIndex(cell[0]+row, cell[1]+col);
}

// NOTE: processed cells are marked by turning the light ON!
void GridLightController::PreprocessLayers( const std::vector<Goal*>& goals ){
    for( int layer = 0; layer < dimGrid; ++layer ){
        std::vector<int> newLayer;
        layerIndicies.push_back(newLayer);
        for( auto goal : goals ){
            if( layer > 0 ){
                int nBourIndex = 0;
                int cell[2] = {0, 0};
                bool quadrants[8] = {
                    true, true, true, true, 
                    true, true, true, true 
                };
                IndexToRowCol( goal->index, cell );

                // Mark quadrants if neighbour cells are goal cells
                for( int i = 0; i < NBOUR_NUM; ++i ){
                    neighbour_t n = static_cast<neighbour_t>(i);
                    nBourIndex = NeighbourIndex(cell, n); 
                    if( lights[nBourIndex]->atGoal ) 
                        MarkQuadrants( quadrants, n ); 
                }
                for( int i = 0; i < NBOUR_NUM; ++i ){
                    neighbour_t n = static_cast<neighbour_t>(i);
                    if( quadrants[i] )
                        AddLayerIndicies( layer, cell, n );
                }
            } else {
                layerIndicies[layer].push_back(goal->index);
                lights[goal->index]->on = true;
            }
        } 
    }
    for( auto light: lights ){
        light->on = false;
    }
}

void GridLightController::MarkQuadrants( bool quadrants[8], neighbour_t n ){
    switch(n){
        case TR: quadrants[0] = false; break; 
        case TC: 
                 quadrants[0] = false; 
                 quadrants[1] = false;
                 quadrants[2] = false;
                 break; 
        case TL: quadrants[2] = false; break; 
        case CL:  
                 quadrants[2] = false; 
                 quadrants[3] = false;
                 quadrants[4] = false;
                 break; 
        case BL: quadrants[4] = false; break; 
        case BC: 
                 quadrants[4] = false; 
                 quadrants[5] = false;
                 quadrants[6] = false;
                 break; 
        case BR: quadrants[6] = false; break; 
        case CR: 
                 quadrants[6] = false; 
                 quadrants[7] = false;
                 quadrants[0] = false;
                 break; 
    }
}

void GridLightController::AddLayerIndicies( int layer, int cell[2], neighbour_t n ){
    bool oneCell = true;
    int rowMod = 0, colMod = 0, index = 0;

    // If only oneCell then changes the setting of a cell at a distance
    // "layer" away. Otherwise changes an entire diagonal.
    switch( n ){
        case TR: oneCell = false; rowMod = -1; colMod = 1; break; 
        case TC: oneCell = true; rowMod = -layer; colMod = 0; break;
        case TL: oneCell = false; rowMod = -1; colMod = -1; break;
        case CL: oneCell = true; rowMod = 0; colMod = -layer; break;
        case BL: oneCell = false; rowMod = 1; colMod = 1; break;
        case BC: oneCell = true; rowMod = layer; colMod = 0; break;
        case BR: oneCell = false; rowMod = 1; colMod = -1; break;
        case CR: oneCell = true; rowMod = 0; colMod = layer; break;
    }

    if( oneCell ){
        index = RowColToIndex(cell[0]+rowMod, cell[1]+colMod);
        if( index >= 0 && !lights[index]->on ){
            layerIndicies[layer].push_back(index);
            lights[index]->on = true;
            lights[index]->layer = layer;
        }
    } else{
        for( int i = 1; i < layer; ++i ){
            index = RowColToIndex( 
                    cell[0]+rowMod*i, 
                    cell[1]+colMod*(layer-i) 
                    );  
            if( index >= 0 && !lights[index]->on ){
                layerIndicies[layer].push_back(index);
                lights[index]->on = true;
                lights[index]->layer = layer;
            }
        }
    }
}

bool GridLightController::NoBoxOutside( int layer, const std::vector<Box*>& boxes ){
    return false;
}

void GridLightController::SeeLayerDistribution( void ){
    std::cout << "\n";
    for( int row = dimGrid-1; row >= 0; --row ){
        for( int col = 0; col < dimGrid; ++col )
            std::cout << lights[RowColToIndex(row, col)]->layer << " ";
        std::cout << "\n";
    }
    std::cout << "\n";
}

void GridLightController::Toggle( int layer ){
    if( layer > 0){
        // std::cout << "Toggling layer: " << layer << std::endl;
        for( int index : layerIndicies[layer] )
            lights[index]->on = !lights[index]->on;
    } else {
        Toggle( layer+dimGrid-1 ); // Modular arithmetics
    }
}

float GridLightController::GetIntensity( float x, float y ){
    int cell[2] = {};
    int index;
    neighbour_t nBour;
    std::vector<float> cells;   // All adjacent cells

    PointInCell( x, y, cell );
    int cellIndex = RowColToIndex(cell[0], cell[1]);

    // std::cout << "row: " << cell[0] << " col: " << cell[1] << " index: " << cellIndex << std::endl;
    if( cellIndex >= 0 ){
        cells.push_back(lights[cellIndex]->GetIntensity(x,y,scaleFactor));

        // Check intensities of all 8 adjacent cells
        for( int i = 0; i < NBOUR_NUM; ++i){
            nBour = static_cast<neighbour_t>(i);
            index = NeighbourIndex( cell, nBour );
            if( index >= 0 ){
                cells.push_back(lights[index]->GetIntensity(x,y,scaleFactor));
            }
        }
        return *std::min_element(cells.begin(), cells.end());
    } else {
        return 1.0;
    }
}

void GridLightController::Update( const std::vector<Goal*>& goals, const std::vector<Box*>& boxes, float timeStep ){
    timeElapsed += timeStep;
    if( timeElapsed > trialTime ){
        timeElapsed = 0;
        // std::cout << "(GLC) Changing lights configuration... \n";
        // Check that all boxes in shadow region
        // if( NoBoxesInLayer(goals, boxes) ){
            //std::cout << "Current Layer: " << currentLayer << std::endl;
            Toggle(currentLayer); 
            Toggle(currentLayer-layersActive);
            currentLayer = NextLayer(currentLayer); 
        //}
    }
}
// ===> END GRIDLIGHTCONTROLLER clas methods
