#ifndef ICONTACTGENERATOR_H
#define ICONTACTGENERATOR_H

#include "Contact.h"

class Simulation;



class IContactGenerator
{
	protected :
	
	Simulation* sim;
	
	
	//----------------------------------------
	//----------------------------------------	
	public :
	
	IContactGenerator(Simulation* sim_);
	virtual ~IContactGenerator();
	
	virtual void generateContactConstraints(std::vector<Contact>& c) = 0;
};




/*This contact generator turns every collision/contact into a SpringForceEffect between the two bodies and the two connections points are used as anchors. The restLength of the spring will be 2 times the value of the penetrationDepth that is to be computed here, and the springConstant will be set by default to 1.0f, for starters...*/
/*WARNING : endTime's of those IForceEffect are to be set to the next simulation time.*/

class ContactGeneratorA : public IContactGenerator
{
	private :	
	
	//----------------------------------------
	//----------------------------------------	
	public :
	
	ContactGeneratorA(Simulation *sim_);
	
	virtual ~ContactGeneratorA();
	virtual void generateContactConstraints(std::vector<Contact>& c) override;
};



#endif
