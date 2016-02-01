#include "IForceEffect.h"

#define INF numeric_limits<float>::epsilon()
//#define debuglvl1

IForceEffect::IForceEffect() : endTime(INF)
{

}

IForceEffect::~IForceEffect()
{

}

//------------------------------------
//------------------------------------


//------------------------------------
//------------------------------------

//gravityVector is initialized to -z by default :
GravityForceEffect::GravityForceEffect(const Mat<float>& g) : IForceEffect(), gravityVector(g)
{

}



GravityForceEffect::~GravityForceEffect()
{

}
	
void GravityForceEffect::Apply(float dt, RigidBody& RB)
{
	// this force is applied to the center of mass of the rigid body so it does not involve the creation of any torque.
	RB.addForce( RB.getMass()*gravityVector);
}


//--------------------------------------------
//--------------------------------------------

//--------------------------------------------
//--------------------------------------------


SpringForceEffect::SpringForceEffect( const Mat<float>& p1in1, const Mat<float>& p2in2, RigidBody& body_, RigidBody& other_, float restLength_, float springConstant_) : IForceEffect(), connectionPointL(p1in1), otherConnectionPointL(p2in2), body(body_), other(other_), restLength(restLength_), springConstant( springConstant_)
{

}
	
SpringForceEffect::~SpringForceEffect()
{

}

	
void SpringForceEffect::Apply(float dt, RigidBody& RB)
{
	Mat<float> p1W( body.getPointInWorld(connectionPointL) );
	Mat<float> p2W( other.getPointInWorld(otherConnectionPointL));
	
#ifdef debuglvl1
std::cout << "SPRING FORCE : pinW" << std::endl;
body.getPosition().afficher();
other.getPosition().afficher();
#endif
	Mat<float> force(p1W-p2W);
	
	float magnitude = norme2(force);
	if(magnitude < numeric_limits<float>::epsilon())
		magnitude = 1e-10f;
		
	force *= 1.0f/magnitude;
	
	magnitude = springConstant*fabs_(magnitude-restLength);
	
	body.addForceAtWorldPoint( (float)(-magnitude)*force, p1W);			
	other.addForceAtWorldPoint( (float)(magnitude)*force, p2W);
	
}
