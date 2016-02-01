#include "CollisionDetector.h"
#include "../Simulation.h"

#define debug


CollisionDetector::CollisionDetector(Simulation* sim_, float eps) : sim(sim_), epsilon(eps), broadPhase(new BroadPhaseStrategyA( sim_)), midNarrowPhase(new MidNarrowPhaseStrategyA( sim_)), contactGenerator( new ContactGeneratorA( sim_))
{

}

CollisionDetector::~CollisionDetector()
{

}

void CollisionDetector::checkForCollision(float dt)
{
#ifdef debug
std::cout << "COLLISION DETECTOR : initialization : ..." << std::endl;
#endif	
	contacts.clear();
#ifdef debug
std::cout << "COLLISION DETECTOR : broadPhase : ..." << std::endl;
#endif	
	
	broadPhase->checkForCollisions(contacts);
	
	
#ifdef debug
std::cout << "COLLISION DETECTOR : nbrContacts : " << contacts.size() << std::endl;
std::cout << "COLLISION DETECTOR : midNarrowPhase : ..." << std::endl;
#endif

	
	midNarrowPhase->checkForCollisions(contacts);
	
	
#ifdef debug
std::cout << "COLLISION DETECTOR : nbrContacts : " << contacts.size() << std::endl;
std::cout << "COLLISION DETECTOR : constraints generation : ..." << std::endl;
#endif
	
	contactGenerator->generateContactConstraints(contacts);
	
#ifdef debug
std::cout << "COLLISION DETECTOR : nbrContacts : " << contacts.size() << std::endl;
std::cout << "COLLISION DETECTOR : all set : OKAY." << std::endl;
#endif	
}



//-------------------------------

std::vector<Contact> CollisionDetector::getContacts()
{	
	return contacts;	
}
