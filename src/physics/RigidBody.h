#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "ISimulationObject.h"
#include "IMoveable.h"
#include "IShape.h"


class RigidBody : public ISimulationObject, public IMoveable
{
	public :
	
	Mat<float> userForce;
	Mat<float> userTorque;
	
	bool isFixed;
	bool canCollide;
	
	float mass;
	float imass;
	//default = 1.0f kg ... unless ground..
	
	Mat<float> Inertia;
	Mat<float> iInertia;
	
	Mat<float> iInertiaWorld;
	
	
	IShape* ptrShape;
	
//--------------------------------------------------------
//--------------------------------------------------------
	
	RigidBody();
	RigidBody(const std::string& name_, int id_, bool isActive_);
	RigidBody(const se3& Pose_, const std::string& name_, int id_, bool isActive_ = true);
	RigidBody(const se3& Pose_, const std::string& name_, int id_, ShapeType shtype, bool isActive_ = true);
	RigidBody(const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel);
	RigidBody(const std::string& name_, int id_, bool isActive_, const se3& Pose_);
	RigidBody(const std::string& name_, int id_, bool isActive_, const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel);
	
	//RigidBody(const std::string& name_, int id_, bool isActive_, const se3& Pose_, std::unique_ptr<IShape> ptrShape_);
	//RigidBody(const std::string& name_, int id_, bool isActive_, const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel, std::unique_ptr<IShape> ptrShape_);
	
	~RigidBody();
	
	//TODO : implantation
	//Mat<float> getFirstOrderDerivatives(float dt = 0.001f);
	
	//TODO : implantation
	void Render(const se3& WorldTransformation) override;
	
	Mat<float> transformInertiaTensorL2W();
	
	void addForce(const Mat<float>& force);
	void addForceAtWorldPoint(const Mat<float>& force, const Mat<float>& pointW);
	void addForceAtBodyPoint(const Mat<float>& force, const Mat<float>& pointL);
	
	void addTorque(const Mat<float>& torque);
	
	void clearUser();
	void calculateDerivedData();
		
	void computeInertia();
	
	//------------------------------------------------------
	//------------------------------------------------------
	
	Mat<float> getForceAccumulator() const;
	
	Mat<float> getTorqueAccumulator() const;
	
	float getMass() const
	{
	 	return mass;	
	}
	float getIMass() const
	{
		return imass;
	}
	
	Mat<float> getInertialLocal() const
	{
		return Inertia;
	}
	
	Mat<float> getInverseInertialLocal() const
	{
		return iInertia;
	}
		
	Mat<float> getInverseInertialWorld() const
	{
		return iInertiaWorld;
	}
			
	bool getCollisionStatus()	const	
	{	
		return canCollide;
	}
	
	bool getFixedStatus()	const	
	{	
		return isFixed;
	}
	
	IShape& getShapeReference() const
	{ 
		/*if( ptrShape.get())
		{
			return *(ptrShape.get());
		}
		else
		{
			std::cout << "uninitialized shape..." <<std::endl;
			throw;
		}*/
		
		return *ptrShape;
	}
	
	ShapeType getShapeType() const	
	{	
		return ptrShape->getShapeType();
	}
	
		
	
	Mat<float> getPointInWorld( const Mat<float>& pointL);
	Mat<float> getPointInLocal( const Mat<float>& pointW);
	Mat<float> getAxisInWorld( const Mat<float>& aL);
	Mat<float> getAxisInLocal( const Mat<float>& aW);
	
	
	//------------------------------------------
	
	void setPtrShape( IShape* ptrShape_)
	{
		//ptrShape.reset( ptrShape_);
		if( ptrShape)
		{
			delete ptrShape;
			ptrShape = ptrShape_;
		}
		
		computeInertia();
	}
	
	void setMass(const float m)
	{
		mass = m;
		imass = 1.0f/m;
	}
	
	void setIMass(const float im)
	{
		imass = im;
		mass = 1.0f/im;
	}	
	
};
#endif
