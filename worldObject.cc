#include <iostream>

#include "push.hh"

// ===> Set Static Variables
float Box::size = 0.25;
float Robot::size = 0.5;
float Light::HEIGHT = 1.0;
float Light::radius;
float Light::cosCritAngle;

// ===> COLORS
const float c_light[3]      = {1.0, 1.0, 0.6};
const float c_red[3]        = {1.0, 0.0, 0.0};
const float c_darkred[3]    = {0.8, 0.0, 0.0};
const float c_lightblue[3]  = {0.5, 0.5, 1.0};
const float c_blue[3]       = {0.0, 0.0, 1.0};
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
        // first point again to close disk
        glVertex2f( radius+center.x, 0+center.y ); 
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
                // // TODO: below makes circles not being properly filled in
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


// ===> WALL class methods
Wall::Wall( b2Body* body, b2Vec3& dim ) :
    WorldObject(dim.x, dim.y),
    body(body)
{
    if (body){
        body->SetTransform( b2Vec2( dim.x, dim.y ), dim.z );
    } else {
        std::cout << "[ERR-Wall] body parameter is NULL.\n";
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
    body->SetTransform( b2Vec2(
                drand48()*(world.width-2*spawnDist) + spawnDist, 
                drand48()*(world.height-2*spawnDist) + spawnDist), 0 ); 
      
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
    if( on ){
        glColor4f( c_light[0], c_light[1], c_light[2], 1 );
        DrawDisk( GetCenter(), radius );
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
