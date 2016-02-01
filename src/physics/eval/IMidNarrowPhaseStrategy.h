#ifndef IMIDNARROWPHASESTRATEGY_H
#define IMIDNARROWPHASESTRATEGY_H

#include "Contact.h"

#include "HelperFunctions.h"

class Simulation;

class IMidNarrowPhaseStrategy 
{
	protected :
	
	Simulation* sim;
	
	
	//----------------------------------------
	//----------------------------------------	
	public :
	
	IMidNarrowPhaseStrategy(Simulation* sim_);
	
	virtual ~IMidNarrowPhaseStrategy();
	virtual void checkForCollisions(std::vector<Contact>& c) = 0;
};





class MidNarrowPhaseStrategyA : public IMidNarrowPhaseStrategy
{
	private :
	
	
	//----------------------------------------
	//----------------------------------------	
	public :
	
	MidNarrowPhaseStrategyA(Simulation *sim_);
	
	virtual ~MidNarrowPhaseStrategyA();
	
	virtual void checkForCollisions(std::vector<Contact>& c) override;
};





#endif
