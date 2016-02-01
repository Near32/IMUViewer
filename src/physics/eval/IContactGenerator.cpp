#include "IContactGenerator.h"
#include "../Simulation.h"

//#define debug


IContactGenerator::IContactGenerator(Simulation* sim_) : sim(sim_)
{

}

IContactGenerator::~IContactGenerator()
{

}

//----------------------------
//----------------------------
//----------------------------

//----------------------------
//----------------------------
//----------------------------


ContactGeneratorA::ContactGeneratorA(Simulation* sim_) : IContactGenerator(sim_)
{

}

ContactGeneratorA::~ContactGeneratorA()
{

}

void ContactGeneratorA::generateContactConstraints(std::vector<Contact>& c)
{
	//let us create a ContactConstraint in the simulation for every contact in c in order to have penetrationDepth :
	
#ifdef debug
std::cout << "COLLISION DETECTOR : CONTACT GENERATOR : initialization : OKAY." << std::endl;
#endif	

#ifdef debug
std::cout << "COLLISION DETECTOR : CONTACT GENERATOR : contact forces creations : ..." << std::endl;
#endif	
	//--------------------------------------
	
	
	for(auto& contact : c)
	{
		//let us create the SpringForceEffect required into the simulation with endTime being the next simulation step time :
		//sim->collectionF.insert( sim->collectionF.begin(), std::unique_ptr<IForceEffect>( new SpringForceEffect( /*in LOCAL : c[i].rbA->getPosition()*/c[i].rbA->getPointInLocal(c[i].contactPoint[0]), /*in LOCAL : c[i].rbB->getPosition()*/c[i].rbB->getPointInLocal(c[i].contactPoint[0]), *(c[i].rbA), *(c[i].rbB), pD[i] ) ) );
		//sim->collectionF[0]->setEndTime( sim->getTime()+sim->getTimeStep());
		//this collision force is only here for one timestep and it will be recreated if needed on the next time step.

		for(int i=contact.contactPoint.size();i--;)
		{
			//let us create a contact constraints for every contact point :
			
			//let us find the anchors in local frames :
			Mat<float> anchorAL( contact.rbA->getPointInLocal(contact.contactPoint[i]) );
			Mat<float> anchorALProjected( contact.rbA->getPointInLocal(contact.contactPoint[i]) );
			
			Mat<float> normalAB(0.0f,3,1);
			innerVoronoiProjectionANDNormal( *(contact.rbA), anchorALProjected, normalAB);
			
			float penetrationDepth = fabs(( transpose(normalAB)*(anchorALProjected-anchorAL) ).get(1,1));
			
			sim->collectionC.insert( sim->collectionC.end(), std::unique_ptr<IConstraint>( new ContactConstraint( *(contact.rbA), *(contact.rbB), anchorAL, anchorALProjected, normalAB, penetrationDepth) ) );
		}
	}
	

	
}
