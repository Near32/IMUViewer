#ifndef IFORCEEFFECT_H
#define IFORCEEFFECT_H

#include "RigidBody.h"

class IForceEffect
{
	protected :
	
	float endTime;
	
	
	//--------------------------------------------------------
	
	public :
	
	IForceEffect();
	
	virtual ~IForceEffect();
	
	virtual void Apply(float dt, RigidBody& RB) = 0;
	
	
	float getEndTime() const	
	{	
		return endTime;	
	}
	
	void setEndTime(const float et)	
	{
		endTime = et;	
	}
	
	virtual bool isGravity() =0;
};



class GravityForceEffect : public IForceEffect
{
	public :
	
	Mat<float> gravityVector;
	
	
	//--------------------------------------------------------
	
	//gravityVector is initialized to -z by default :
	GravityForceEffect();
	GravityForceEffect(const Mat<float>& g);
	
	~GravityForceEffect();
	
	virtual void Apply(float dt, RigidBody& RB) override;
	
	virtual bool isGravity() override
	{
		return true;
	}
};


class SpringForceEffect : public IForceEffect
{
	public :
	
	Mat<float> connectionPointL;	// has to be specified in the Local frame -> easier to deal we the changes of the pose of that rigid body...
	Mat<float> otherConnectionPointL;	//idem, but in the one of the other RigidBody :
	
	RigidBody& body;
	RigidBody& other;
	
	float springConstant;
	float restLength;
	
	
	//----------------------------------------
	//----------------------------------------
	SpringForceEffect();
	SpringForceEffect( const Mat<float>& p1in1, const Mat<float>& p2in2, RigidBody& body_, RigidBody& other_, float restLength_ = 1.0f, float springConstant_ = 1e-1f);
	
	~SpringForceEffect();
	
	virtual void Apply(float dt, RigidBody& RB) override;
	
	virtual bool isGravity() override
	{
		return false;
	}
};


 
#endif
