#ifndef IMOVEABLE_H
#define IMOVEABLE_H

#include "../utils/math.h"
#include <exception>

class IMoveable
{
	protected :
	
	se3* Pose;
	
	Mat<float>* LinearVelocity;
	Mat<float>* AngularVelocity;
	
//-----------------------------------------------------
//-----------------------------------------------------
	public :
	
	
	IMoveable();
	IMoveable( const se3& Pose_);													//Orientation has to be initialized from Pose.
	IMoveable( const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel);	//Orientation has to be initialized from Pose.
	
	virtual ~IMoveable();
	
	
	//---------------------------------------------------------
	//---------------------------------------------------------
	
		
	se3 getPose() const	;
	
	Mat<float> getPosition();
		
	Quat getOrientation();
	
	Mat<float> getMatOrientation();
	
	Mat<float> getTransformation()	const;
	
	Mat<float> getLinearVelocity()	const;
	
	Mat<float> getAngularVelocity()	const;

	
	//---------------------------------------------------------
	//---------------------------------------------------------
	
	void setPose( const se3& pose_);
	void setPosition( const Mat<float>& t_);
	void setOrientation( const Quat& q);
	void setMatOrientation( const Mat<float>& q);
	void setLinearVelocity( const Mat<float>& lvel);
	void setAngularVelocity( const Mat<float>& avel);
	
};

#endif
