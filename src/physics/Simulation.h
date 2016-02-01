#ifndef SIMULATION_H
#define SIMULATION_H


#include <memory>
#include "ISimulationObject.h"
#include "IConstraint.h"
#include "IForceEffect.h"
#include "../Environnement.h"


#include "eval/IUpdater.h"
#include "eval/IConstraintSolverStrategy.h"
#include "eval/CollisionDetector.h"

class Simulation
{
	public :
	
	Environnement *env;
	float time;
	std::map<std::string,int> Name2ID;
	
	std::vector< std::unique_ptr< ISimulationObject > > simulatedObjects;
	std::vector<std::unique_ptr<IConstraint> > collectionC;
	std::vector<std::unique_ptr<IForceEffect> > collectionF;
	
	std::unique_ptr<IConstraintSolverStrategy> constraintsSolver;
	std::unique_ptr<IUpdater> updater;
	std::unique_ptr<CollisionDetector> collisionDetector;
	
	bool initializedQQdotInvMFext;
	Mat<float> q;
	Mat<float> qdot;
	Mat<float> Fext;
	
	SparseMat<float> invM;
	SparseMat<float> S;
	
	float SimulationTimeStep;
	
	//-------------------------------------------------------------
	//-------------------------------------------------------------
	
	
	Simulation();
	//Simulation(IConstraintsSolver* cs, IUpdater* u, CollisionDetector* cd);
	Simulation(Environnement* env_);
	Simulation(Environnement* env_, ConstraintsList& cl);
	
	~Simulation();
	
	void run(float timeStep, float endTime);
	
	//----------------------------------------------------------------
	//----------------------------------------------------------------
	
	void runStride( float timeStep = 1e-3f);
	
	void constructQ();
	
	void constructQdot();
	
	void constructQQdotInvMSFext();
	void constructQQdotInvMSFextDEBUG();
	void updateInvMSFext();
	
	//void updateQQdotInvMSFext();
	
	void applyForces(float timeStep);
	void updateQQdot();
	void updateStates();
		
	/*
	void majQ(const Mat<float>& add)
	{
		q += add;
	}
	
	void majQdot(const Mat<float>& add)
	{
		qdot += add;
	}
	*/
	
	float getTime()	const;
	float getTimeStep()	const;
	
	
};
#endif
