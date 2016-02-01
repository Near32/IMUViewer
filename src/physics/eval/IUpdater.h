#ifndef IUPDATER_H
#define IUPDATER_H

#include "IIntegrator.h"
class Simulation;

class IUpdater
{
	private :
	
	Simulation* sim;	
	std::unique_ptr<IIntegrator> integrator;
	
	
	//----------------------------------------------
	//----------------------------------------------
	public :
	
	IUpdater(Simulation* sim_, IIntegrator* integ);
	
	~IUpdater();
	
		
};


class Updater : public IUpdater
{
	public :
	
	Updater(Simulation* sim_, IIntegrator* integ);
	
	~Updater();
	
};



#endif
