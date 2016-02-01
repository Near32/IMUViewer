#ifndef COLLISIONDETECTOR_H
#define COLLISIONDETECTOR_H

#include "IBroadPhaseStrategy.h"
#include "IMidNarrowPhaseStrategy.h"
#include "IContactGenerator.h"

#include "Contact.h"

class Simulation;


class CollisionDetector
{
	private :
	
	Simulation* sim;
	
	float epsilon;
	std::unique_ptr<IBroadPhaseStrategy> broadPhase;
	std::unique_ptr<IMidNarrowPhaseStrategy> midNarrowPhase;
	std::unique_ptr<IContactGenerator> contactGenerator;
	
	std::vector<Contact> contacts;
	//------------------------------
	//------------------------------
	
	public :
	
	CollisionDetector(Simulation* sim_, float eps = (float)1e-10);
	
	~CollisionDetector();
	
	void checkForCollision(float dt = 0.0001f);	
	
	
	//-------------------------------
	
	std::vector<Contact> getContacts();
	
	
};

#endif
