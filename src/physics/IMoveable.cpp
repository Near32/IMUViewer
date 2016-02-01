#include "IMoveable.h"

//#define debuglvl1

IMoveable::IMoveable()
{	
	try
	{
		Pose = new se3();
		//Orientation = Qt_FromMat( extract(Pose.exp(), 1,1,3,3) );	
		LinearVelocity = new Mat<float>((float)0,3,1);
		AngularVelocity = new Mat<float>((float)0,3,1);
	}
	catch( std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		throw e;
	}
}
	
//Orientation has to be initialized from Pose.
IMoveable::IMoveable( const se3& Pose_)
{
	try
	{
		Pose = new se3();
		LinearVelocity = new Mat<float>((float)0,3,1);
		AngularVelocity = new Mat<float>((float)0,3,1);
	}
	catch(std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		throw e;
	}
	
	*Pose = Pose_;
	//Orientation = Qt_FromMat( extract(Pose.exp(), 1,1,3,3) );		
}

//Orientation has to be initialized from Pose.
IMoveable::IMoveable( const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel): Pose( new se3() ), LinearVelocity( new Mat<float>(Lvel)), AngularVelocity( new Mat<float>(Avel))
{
	*Pose = Pose_;
	//Orientation = Qt_FromMat( extract(Pose.exp(), 1,1,3,3) );	
}	

IMoveable::~IMoveable()
{
		delete Pose;
		delete LinearVelocity;
		delete AngularVelocity;
}


se3 IMoveable::getPose() const	
{ 
	return *Pose;
}

Mat<float> IMoveable::getPosition()
{
	return Pose->getT();
}
	
Quat IMoveable::getOrientation()		
{
	return Qt_FromMat( this->Pose->exp());
}

Mat<float> IMoveable::getMatOrientation()
{
	return Qt2Mat<float>( this->getOrientation() );
}

Mat<float> IMoveable::getTransformation()	const
{
	return this->Pose->exp();	
}

Mat<float> IMoveable::getLinearVelocity()	const
{
	return *(this->LinearVelocity);
}

Mat<float> IMoveable::getAngularVelocity()	const
{
	return *(this->AngularVelocity);
}

void IMoveable::setPose( const se3& pose_)	
{	
	*Pose = pose_;
}

void IMoveable::setPosition( const Mat<float>& t_)
{
	Pose->setT(t_);
}

void IMoveable::setOrientation( const Quat& q)	
{
	Pose->setOrientation(q);	
}

void IMoveable::setMatOrientation( const Mat<float>& q)	
{	
	
#ifdef debuglvl1		
	std::cout << " IMOVEABLE : TEST AVANT MODIF : " << std::endl;
	Quat tempq = Mat2Qt<float>(q);
	std::cout << tempq.x << " ; " << tempq.y << " ; " << tempq.z << " ; " << tempq.w << std::endl;
#endif

	this->setOrientation( Mat2Qt<float>(q) );	
}

void IMoveable::setLinearVelocity( const Mat<float>& lvel)	
{	
	*LinearVelocity = lvel;	
}

void IMoveable::setAngularVelocity( const Mat<float>& avel)	
{	
	*AngularVelocity = avel;	
}


