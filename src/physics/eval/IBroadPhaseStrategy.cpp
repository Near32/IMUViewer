#include "IBroadPhaseStrategy.h"
#include "../Simulation.h"

IBroadPhaseStrategy::IBroadPhaseStrategy(Simulation* sim_) : sim(sim_)
{

}

IBroadPhaseStrategy::~IBroadPhaseStrategy()
{

}

//--------------------------------------------
//--------------------------------------------
//--------------------------------------------

//--------------------------------------------
//--------------------------------------------
//--------------------------------------------

BroadPhaseStrategyA::BroadPhaseStrategyA(Simulation* sim_) : IBroadPhaseStrategy(sim_)
{

}

BroadPhaseStrategyA::~BroadPhaseStrategyA()
{

}

/*
void BroadPhaseStrategyA::checkForCollisions(std::vector<Contact>& c)
{
	//let us access all the RigidBodys in the Simulation and check if they are in contact broadly via the boundingRadius of their Shape :
	//std::vector<std::unique_ptr<ISimulationObject> >::iterator it = sim->simulatedObjects.begin();
	
	#define simul sim->simulatedObjects
	
	for( //;it !=sim->simulatedObjects.end();it++)
	{
		if( (it->get())->getType() == TSORigidBody)
		{
			if( ((RigidBody*)(it->get()))->getCollisionStatus() )
			{
				//let us access to all of the RigidBody that haven't already been checked with regards to this one :
				std::vector<std::unique_ptr<ISimulationObject> >::iterator other = it;
				other++;
			
				ShapeType typeIt = ((RigidBody*)(it->get()))->getShapeType();
				
			
				for(;other!=sim->simulatedObjects.end();other++)
				{
					ShapeType typeOther = ((RigidBody*)(other->get()))->getShapeType();
					//------------------
					if( ((other->get()))->getType() == TSORigidBody)
					{
						if( ((RigidBody*)(other->get()))->getCollisionStatus() )
						{
							Mat<float> midline( ((RigidBody*)(it->get()))->getPosition()-((RigidBody*)(other->get()))->getPosition());
							float magnitude = norme2(midline);
					
							if(magnitude < ((RigidBody*)(it->get()))->getShapeReference().getBRadius() + ((RigidBody*)(other->get()))->getShapeReference().getBRadius() )
							{
								// then there is a potentiel contact :
								Contact contact( (RigidBody*)(it->get()), (RigidBody*)(other->get()) );
								//contact.contactPoint.insert( contact.contactPoint.end(), ((RigidBody*)(other->get()))->getPosition()+(1.0f/2.0f)*midline);
								contact.normal.insert( contact.normal.end(), (1.0f/magnitude)*midline);
						
								c.insert(c.end(), contact );
							}
						}
						 
				
					}

				}
			}
		
		}
	}
}

*/



void BroadPhaseStrategyA::checkForCollisions(std::vector<Contact>& c)
{
	//let us access all the RigidBodys in the Simulation and check if they are in contact broadly via the boundingRadius of their Shape :
	//std::vector<std::unique_ptr<ISimulationObject> >::iterator it = sim->simulatedObjects.begin();
	
	#define simul sim->simulatedObjects
	
	for(int i=0;i<simul.size();i++)
	{
		if( simul[i]->getType() == TSORigidBody)
		{
			if( ((RigidBody*)(simul[i].get()))->getCollisionStatus() )
			{
				//let us access to all of the RigidBody that haven't already been checked with regards to this one :
				//std::vector<std::unique_ptr<ISimulationObject> >::iterator other = it;
				//other++;
			
				ShapeType typeIt = ((RigidBody*)(simul[i].get()))->getShapeType();
				
			
				for(int o=i+1;o<simul.size();o++)//;other!=sim->simulatedObjects.end();other++)
				{
					ShapeType typeOther = ((RigidBody*)(simul[o].get()))->getShapeType();
					//------------------
					if( (simul[o].get())->getType() == TSORigidBody)
					{
						if( ((RigidBody*)(simul[o].get()))->getCollisionStatus() )
						{
							Mat<float> midline( ((RigidBody*)(simul[i].get()))->getPosition()-((RigidBody*)(simul[o].get()))->getPosition());
							float magnitude = norme2(midline);
					
							if(magnitude < ((RigidBody*)(simul[i].get()))->getShapeReference().getBRadius() + ((RigidBody*)(simul[o].get()))->getShapeReference().getBRadius() )
							{
								// then there is a potentiel contact :
								Contact contact( ((RigidBody*)(simul[i].get())), ((RigidBody*)(simul[o].get())) );
								//contact.contactPoint.insert( contact.contactPoint.end(), ((RigidBody*)(simul[o].get()))->getPosition()+(1.0f/2.0f)*midline);
								contact.normal.insert( contact.normal.end(), (1.0f/magnitude)*midline);
						
								c.insert(c.end(), contact );
							}
						}
						 
				
					}

				}
			}
		
		}
	}
}

