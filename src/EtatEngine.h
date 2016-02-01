#ifndef ETATENGINE_H
#define ETATENGINE_H

#include "IEngine.h"
#include "Environnement.h"
#include "physics/Simulation.h"
#include <thread>
#include <mutex>

#include "Traj.h"

class EtatEngine : public IEngine
{
	public :
	
	Environnement* env;
	Simulation* sim;
	Traj *traj;
	
	//---------------------------------------------
	//---------------------------------------------
	
	EtatEngine(Game* game_, GameState gameState_);
	~EtatEngine();
	
	void loop() override ;

	void init();

};

#endif
