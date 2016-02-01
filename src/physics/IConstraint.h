#ifndef ICONSTRAINT_H
#define ICONSTRAINT_H

#include "RigidBody.h"
#include "ConstraintsList.h"

/* cf previous included file
enum ConstraintType{
	CTI,
	CTContact,
	CTBAS,
	CTH,
	CTIL
};
*/

class IConstraint
{
	public :
	
	ConstraintType type;	
	RigidBody& rbA;
	RigidBody& rbB;
	
	bool bodiesCanCollide;
	
	Mat<float> AnchorAL;
	Mat<float> AnchorBL;
	
	Mat<float> AxisA;	//in World
	Mat<float> AxisB;	//in World
	
	//TODO : warning about their use in the HingJoint constraint and the definition in Worl of Local of the HJAxises...
	
	Mat<float> constraintsImpules;
	Mat<float> constraintForce;
	
	Mat<float> JacobianA;
	Mat<float> JacobianB;
	
	Mat<float> C;
	//-----------------------------------
	//-----------------------------------
	
	//Anchors can be iniatilized to the center of mass, by default.
	//Axises can be initialized to the local coordinate frame axises.
	IConstraint(RigidBody& rbA_, RigidBody& rbB_);
	
	~IConstraint();
	
	virtual void addPenaltySpring(float dt = 0.001f) = 0;
	virtual void applyConstraintImpulse(float dt = 0.001f) = 0;
	virtual void applyPositionCorrection(float dt = 0.001f) = 0;
	virtual void computeJacobians() = 0;
	
	
	Mat<float> getJacobianA() const
	{
		return JacobianA;
	}
	
	Mat<float> getJacobianB() const
	{
		return JacobianB;
	}
	
	Mat<float> getConstraint()	const
	{
		return C;
	}
	
	ConstraintType getType()	const
	{
		return type;
	}
	
};



class ContactConstraint : public IConstraint
{
	public :
	
	float penetrationDepth;
	Mat<float> normalAL;	//normal of the face of rbA that is being penetrating. From rbA to rbB.
							// So, Cdot = (vpB - vpA)*nAB
	
	//-----------------------------------------
	//-----------------------------------------
	
	//Penetration depth is initialized to 0, by default.	
	//Anchors can be iniatilized to the center of mass, by default.
	//Axises can be initialized to the local coordinate frame axises.
	ContactConstraint(RigidBody& rbA_, RigidBody& rbB_, float penetrationDepth_ = 0.0f);
	ContactConstraint(RigidBody& rbA_, RigidBody& rbB_, const Mat<float>& cPointAL, const Mat<float>& cPointBL, float penetrationDepth_=0.0f);
	ContactConstraint(RigidBody& rbA_, RigidBody& rbB_, const Mat<float>& cPointAL, const Mat<float>& cPointBL, const Mat<float>& normalAL_, float penetrationDepth_=0.0f);
	
	
	~ContactConstraint();	
		
	virtual void addPenaltySpring(float dt = 0.001f) override;
	virtual void applyConstraintImpulse(float dt = 0.001f) override;
	virtual void applyPositionCorrection(float dt = 0.001f) override;
	virtual void computeJacobians() override;
	
	
};



class BallAndSocketJoint : public IConstraint
{
	public :
	
	
	
	//-----------------------------------------
	//-----------------------------------------
	
	//Anchors can be iniatilized to the center of mass, by default.
	//Axises can be initialized to the local coordinate frame axises.
	BallAndSocketJoint(RigidBody& rbA_, RigidBody& rbB_, const Mat<float>& AnchorAL, const Mat<float>& AnchorBL);
	
	
	~BallAndSocketJoint();
	
	virtual void addPenaltySpring(float dt = 0.001f) override;
	virtual void applyConstraintImpulse(float dt = 0.001f) override;
	virtual void applyPositionCorrection(float dt = 0.001f) override;
	
	virtual void computeJacobians() override;
	
	
	
};




class HingeJoint : public IConstraint
{
	public :
	
	//have to be used in world, but they are registered in local so that they do not change while simulating...
	//in local of A.
	Mat<float> HJAxisL;
	Mat<float> HJAxisL1;
	Mat<float> HJAxisL2;
	
	std::unique_ptr<BallAndSocketJoint> BASJoint;
	Mat<float> AnchorL;
	//in local of A.
	
	//-----------------------------------------
	//-----------------------------------------
	
	//Anchors can be iniatilized to the center of mass, by default.
	//Axises can be initialized to the local coordinate frame axises.
	HingeJoint( RigidBody& rbA_, RigidBody& rbB_, const Mat<float>& HJAxisL, const Mat<float>& AnchorL_);
	
	
	~HingeJoint();
	
	virtual void addPenaltySpring(float dt = 0.001f) override;
	virtual void applyConstraintImpulse(float dt = 0.001f) override;
	virtual void applyPositionCorrection(float dt = 0.001f) override;
	
	virtual void computeJacobians() override;
	
	
};




class ILimitConstraint : public IConstraint
{
	public :
	
	float Min;
	float Max;
	
	//-----------------------------------------
	//-----------------------------------------
	
	//Penetration depth is initialized to 0, by default.	
	//Anchors can be iniatilized to the center of mass, by default.
	//Axises can be initialized to the local coordinate frame axises.
	//ILimitConstraint(RigidBody& rbA_, RigidBody& rbB_, float Min_ = -INF, float Max_ = INF);
	ILimitConstraint(RigidBody& rbA_, RigidBody& rbB_, float Min_, float Max_);
	
	
	~ILimitConstraint();
	
	virtual void addPenaltySpring(float dt = 0.001f) override;
	virtual void applyConstraintImpulse(float dt = 0.001f) override;
	virtual void applyPositionCorrection(float dt = 0.001f) override;
	
	virtual void computeJacobians() override;
	
	virtual Mat<float> getLowLimit() =0;
	virtual Mat<float> getHighLimit() =0;
	
	
	
};


//TODO : declare and implant [Linear-Angular]LimitConstraint



#endif
