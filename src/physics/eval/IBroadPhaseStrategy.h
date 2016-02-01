#ifndef IBROADPHASESTRATEGY_H
#define IBROADPHASESTRATEGY_H


#include "Contact.h"
class Simulation;

class IBroadPhaseStrategy 
{
	protected :
	
	Simulation* sim;
	
	
	//----------------------------------------
	//----------------------------------------	
	public :
	
	IBroadPhaseStrategy(Simulation* sim_);
	
	virtual ~IBroadPhaseStrategy();
	
	virtual void checkForCollisions(std::vector<Contact>& c) = 0;
};













class BroadPhaseStrategyA: public IBroadPhaseStrategy
{
	private :
	
	
	
	//----------------------------------------
	//----------------------------------------	
	public :
	
	BroadPhaseStrategyA(Simulation* sim_);
	
	virtual ~BroadPhaseStrategyA();
	
	virtual void checkForCollisions(std::vector<Contact>& c) override;
	
};

#endif
