#include "IIntegrator.h"
#include "../Simulation.h"

IIntegrator::IIntegrator(Simulation* sim) : sim(sim)
{

}

IIntegrator::~IIntegrator()
{

}

//---------------------------------------------
//---------------------------------------------

//---------------------------------------------
//---------------------------------------------

ExplicitEulerIntegrator::ExplicitEulerIntegrator(Simulation* sim) : IIntegrator(sim)
{

}

ExplicitEulerIntegrator::~ExplicitEulerIntegrator()
{

}	

void ExplicitEulerIntegrator::integrate(float dt)
{
	assert(dt > 0.0f);
	
	//int nbrSimulatedObjects = sim->simulatedObjects.size();
	std::vector<std::unique_ptr<ISimulationObject> >::iterator it = sim->simulatedObjects.begin();
	
	while( it != sim->simulatedObjects.end())
	{
		if( (*it)->getType() == TSORigidBody)
		{
			((RigidBody&)(*(*it))).calculateDerivedData();
			
			
			//Linear :
			Mat<float> resultingLinearAcceleration(  ((RigidBody&)(*(*it))).getIMass() * ((RigidBody&)(*(*it))).getForceAccumulator());
	
			((RigidBody&)(*(*it))).setLinearVelocity( ((RigidBody&)(*(*it))).getLinearVelocity() + dt*resultingLinearAcceleration);
	
			//velocity *= pow( damping, duration);
			
			((RigidBody&)(*(*it))).setPosition( ((RigidBody&)(*(*it))).getPosition() + dt*((RigidBody&)(*(*it))).getLinearVelocity());
	
			//Angular :
			Mat<float> resultingAngularAcceleration(  ((RigidBody&)(*(*it))).getInverseInertialWorld() * ((RigidBody&)(*(*it))).getTorqueAccumulator());
	
			((RigidBody&)(*(*it))).setAngularVelocity( ((RigidBody&)(*(*it))).getAngularVelocity() + dt*resultingAngularAcceleration);
	
			//velocity *= pow( damping, duration);
			
			//----------------------------------------------
			//Quaternion : via Euler Angles...
			//Mat<float> deltaQEuler(dt*getAngularVelocity);
			//it->setOrientation( Qt_Mul( Euler2Qt( deltaQEuler.get(1,1), deltaQEuler.get(2,1), deltaQEuler.get(3,1)), it->getOrientation() ) );
			//----------------------------------------------
			//----------------------------------------------
			//Quaternion : via qrond matrix : watch out for the quaternion definition : here : (x y z w)^T...
			Mat<float> qrond(4,3);
			Quat q( ((RigidBody&)(*(*it))).getOrientation() );
			qrond.set( q.w, 1,1);
			qrond.set( q.w, 2,2);
			qrond.set( q.w, 3,3);
			
			qrond.set( q.x, 2,3);
			qrond.set( -q.x, 3,2);
			qrond.set( -q.x, 4,1);
			
			qrond.set( q.y, 3,1);
			qrond.set( -q.y, 1,3);
			qrond.set( -q.y, 4,2);
			
			qrond.set( q.z, 1,2);
			qrond.set( -q.z, 2,1);
			qrond.set( -q.z, 4,3);
			
			((RigidBody&)(*(*it))).setOrientation( ((RigidBody&)(*(*it))).getOrientation()+Mat2Qt( dt * (qrond* ((RigidBody&)(*(*it))).getAngularVelocity()) ) );
			//----------------------------------------------
			
			//-------------------
			((RigidBody&)(*(*it))).clearUser();
		}
	}
}
