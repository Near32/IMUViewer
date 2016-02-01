#include "IConstraint.h"

//#define debug

IConstraint::IConstraint(RigidBody& rbA_, RigidBody& rbB_) : rbA(rbA_), rbB(rbB_), bodiesCanCollide(true)
{
	type = CTIC;
	
	C = Mat<float>(0.0f,3,1);
	
	AnchorAL = rbA.getPointInLocal( rbA.getPosition() );
	AnchorBL = rbB.getPointInLocal( rbB.getPosition() );
	
	
	//axises of A in W:
	Mat<float> R(4,4);
	Qt_ToMatrix(rbA.getOrientation(), &R);
	R = extract( R, 1,1, 3,3);
	
	
	Mat<float> tu((float)0,3,1);
	tu.set((float)1,1,1);		//tx
	AxisA = R*tu;
	
	tu *= (float)0;
	tu.set((float)1,2,1);		//ty
	AxisA = operatorL( AxisA, R*tu);
	
	tu *= (float)0;
	tu.set((float)1,3,1);		//tz
	AxisA = operatorL( AxisA, R*tu);
	
	//---------------------------------
	
	
	//axises of B in W:
	Qt_ToMatrix(rbB.getOrientation(), &R);
	R = extract( R, 1,1, 3,3);
	
	
	tu *= (float)0;
	tu.set((float)1,1,1);		//tx
	AxisB = R*tu;
	
	tu *= (float)0;
	tu.set((float)1,2,1);		//ty
	AxisB = operatorL( AxisB, R*tu);
	
	tu *= (float)0;
	tu.set((float)1,3,1);		//tz
	AxisB = operatorL( AxisB, R*tu);
	
	
	//---------------------------------
	
}
	
IConstraint::~IConstraint()
{

}

//---------------------------------------------
//---------------------------------------------

//---------------------------------------------
//---------------------------------------------






//Penetration depth is initialized to 0, by default.	
//Anchors can be iniatilized to the center of mass, by default.
//Axises can be initialized to the local coordinate frame axises.
ContactConstraint::ContactConstraint(RigidBody& rbA_, RigidBody& rbB_, float penetrationDepth_) : penetrationDepth(penetrationDepth_), IConstraint( rbA_, rbB_)
{
	type = CTContactConstraint;
	normalAL = Mat<float>(0.0f,3,1);
	normalAL.set( 1.0f,3,1);
	
	//C = transpose(rbA.getAxisInWorld(normalAL))*(rbB.getPointInWorld(AnchorBL)-rbA.getPointInWorld(AnchorAL));
	C = Mat<float>(penetrationDepth,1,1);
}

ContactConstraint::ContactConstraint(RigidBody& rbA_, RigidBody& rbB_, const Mat<float>& cPointAL, const Mat<float>& cPointBL, float penetrationDepth_) : penetrationDepth(penetrationDepth_), IConstraint( rbA_, rbB_)
{
	AnchorAL = cPointAL;
	AnchorBL = cPointBL;
	
	type = CTContactConstraint;
	normalAL = Mat<float>(0.0f,3,1);
	normalAL.set( 1.0f,3,1);
	
	C = Mat<float>(penetrationDepth,1,1);

}

ContactConstraint::ContactConstraint(RigidBody& rbA_, RigidBody& rbB_, const Mat<float>& cPointAL, const Mat<float>& cPointBL, const Mat<float>& normalAL_, float penetrationDepth_) : penetrationDepth(penetrationDepth_), IConstraint( rbA_, rbB_)
{
	AnchorAL = cPointAL;
	AnchorBL = cPointBL;
	
	type = CTContactConstraint;
	normalAL = normalAL_;
	
	C = Mat<float>(penetrationDepth,1,1);

}

ContactConstraint::~ContactConstraint()
{

}

void ContactConstraint::addPenaltySpring(float dt)
{

}

void ContactConstraint::applyConstraintImpulse(float dt)
{

}


void ContactConstraint::applyPositionCorrection(float dt)
{

}

void ContactConstraint::computeJacobians()
{
	//A :
	Mat<float> Identity(0.0f,3,3);
	for(int i=3;i--;)	Identity.set(1.0f,i+1,i+1);
	
	//JacobianA = operatorL( Identity, (1.0f)*crossProduct(AnchorAL) );
	Mat<float> normalG(rbA.getAxisInWorld(normalAL));
	JacobianA = operatorL( (-1.0f)*transpose(normalG), transpose((-1.0f)*crossproductV( rbA.getPointInWorld(AnchorAL)-rbA.getPosition(), normalG)) );
	
	//---------------------
	
	//B : 
	//JacobianB = operatorL( (-1.0f)*Identity, (-1.0f)*crossProduct(AnchorBL) );
	JacobianB = operatorL( (1.0f)*transpose(normalG), transpose((1.0f)*crossproductV( rbA.getPointInWorld(AnchorAL) - rbB.getPosition(), normalG)) );
	
	
	//--------------------------
	//Constraint :
	C = absM(transpose(normalAL)*(AnchorBL-AnchorAL));
	//anchorBL is AnchorALProjected.
}








//---------------------------------------------
//---------------------------------------------

//---------------------------------------------
//---------------------------------------------






//Penetration depth is initialized to 0, by default.	
//Anchors can be iniatilized to the center of mass, by default.
//Axises can be initialized to the local coordinate frame axises.
BallAndSocketJoint::BallAndSocketJoint(RigidBody& rbA_, RigidBody& rbB_, const Mat<float>& AnchorAL_, const Mat<float>& AnchorBL_) : IConstraint( rbA_, rbB_)
{
	AnchorAL = AnchorAL_;
	AnchorBL = AnchorBL_;
	
	C = rbA.getPointInWorld(AnchorAL)-rbB.getPointInWorld(AnchorBL); 
	
	type = CTBallAndSocketJoint;
}

BallAndSocketJoint::~BallAndSocketJoint()
{

}

	
void BallAndSocketJoint::addPenaltySpring(float dt)
{

}

void BallAndSocketJoint::applyConstraintImpulse(float dt)
{

}


void BallAndSocketJoint::applyPositionCorrection(float dt)
{

}

void BallAndSocketJoint::computeJacobians()
{
	//A :
	Mat<float> Identity(0.0f,3,3);
	for(int i=3;i--;)	Identity.set(1.0f,i+1,i+1);
	
	JacobianA = operatorL( (-1.0f)*Identity, (1.0f)*crossProduct(rbA.getPointInWorld(AnchorAL)-rbA.getPosition()) );
	
	//---------------------
	
	//B : 
	JacobianB = operatorL( (1.0f)*Identity, (-1.0f)*crossProduct(rbB.getPointInWorld(AnchorBL)-rbB.getPosition()) );
	
	rbB.getPointInWorld(AnchorBL).afficher();
	rbA.getPointInWorld(AnchorAL).afficher();
	//---------------------------
	//Constraints :
	C = rbA.getPointInWorld(AnchorAL)-rbB.getPointInWorld(AnchorBL);
}




//---------------------------------------------
//---------------------------------------------

//---------------------------------------------
//---------------------------------------------




//Penetration depth is initialized to 0, by default.	
//Anchors can be iniatilized to the center of mass, by default.
//Axises can be initialized to the local coordinate frame axises.
HingeJoint::HingeJoint( RigidBody& rbA_, RigidBody& rbB_, const Mat<float>& HJAxisL_, const Mat<float>& AnchorL_) : IConstraint(rbA_,rbB_)
{
	HJAxisL = HJAxisL_;
	AnchorL = AnchorL_;
	BASJoint = std::unique_ptr<BallAndSocketJoint>(new BallAndSocketJoint( rbA_,rbB_, AnchorL, rbB.getPointInLocal( rbA.getPointInWorld(AnchorL) ) ) );
	
	//--------------------
	
	HJAxisL1 = crossproductV( HJAxisL, Mat<float>(1.0f,3,1) );
	//default idea, be careful with another use of this physics engine...
	HJAxisL2 = crossproductV( HJAxisL, HJAxisL1);
	
	//TODO : verify the correctness of this use of the axises in World Reference Frame ?...!!
	
	//--------------------
	BASJoint->computeJacobians();
	Mat<float> wij(rbA.getAngularVelocity()-rbB.getAngularVelocity());
	C = operatorC( operatorC( BASJoint->getConstraint(), transpose(rbA.getAxisInWorld(HJAxisL1))*wij ), transpose(rbA.getAxisInWorld(HJAxisL2))*wij ); 
	//C = operatorC( operatorC( BASJoint->getConstraint(), transpose(HJAxisL1)*wij ), transpose(HJAxisL2)*wij ); 
	
	type = CTHingeJoint;
}

HingeJoint::~HingeJoint()
{

}
	
void HingeJoint::addPenaltySpring(float dt)
{

}

void HingeJoint::applyConstraintImpulse(float dt)
{

}


void HingeJoint::applyPositionCorrection(float dt)
{

}

void HingeJoint::computeJacobians()
{
	BASJoint->computeJacobians();
	//---------------------------
	
	
	//A :
	Mat<float> zero(0.0f,1,3);
	
	JacobianA = operatorC( 
	
		operatorC( BASJoint->getJacobianA(),
					operatorL( zero, (1.0f)*transpose( rbA.getAxisInWorld(HJAxisL1) ) ) ),
					//operatorL( zero, (1.0f)*transpose(HJAxisL1) ) ),
					
		operatorL( zero, (1.0f)*transpose( rbA.getAxisInWorld(HJAxisL2) ) )
		//operatorL( zero, (1.0f)*transpose(HJAxisL2) )
							 );
	
	//-----------------------
	
	//B :
	JacobianB = operatorC( 
	
		operatorC( BASJoint->getJacobianB(),
					operatorL( zero, (-1.0f)*transpose( rbA.getAxisInWorld( HJAxisL1 ) ) ) ),
					//operatorL( zero, (-1.0f)*transpose( HJAxisL1 ) ) ),
					
		operatorL( zero, (-1.0f)*transpose( rbA.getAxisInWorld( HJAxisL2 ) ) )
		//operatorL( zero, (-1.0f)*transpose( HJAxisL2 ) )
							 );

	Mat<float> wij(rbA.getAngularVelocity()-rbB.getAngularVelocity());
	//C = operatorC( operatorC( BASJoint->getConstraint(), transpose(HJAxisL1)*wij ), transpose(HJAxisL2)*wij ); 
	C = operatorC( operatorC( BASJoint->getConstraint(), transpose(rbA.getAxisInWorld(HJAxisL1))*wij ), transpose(rbA.getAxisInWorld(HJAxisL2))*wij ); 	
}


//---------------------------------------------
//---------------------------------------------

//---------------------------------------------
//---------------------------------------------





//Penetration depth is initialized to 0, by default.	
//Anchors can be iniatilized to the center of mass, by default.
//Axises can be initialized to the local coordinate frame axises.
ILimitConstraint::ILimitConstraint(RigidBody& rbA_, RigidBody& rbB_, float Min_, float Max_) : Min(Min_), Max(Max_), IConstraint( rbA_, rbB_)
{
	type = CTLimitConstraint;
}

ILimitConstraint::~ILimitConstraint()
{

}

void ILimitConstraint::addPenaltySpring(float dt)
{

}

void ILimitConstraint::applyConstraintImpulse(float dt)
{

}


void ILimitConstraint::applyPositionCorrection(float dt)
{

}

void ILimitConstraint::computeJacobians()
{

}



//---------------------------------------------
//---------------------------------------------

//---------------------------------------------
//---------------------------------------------





