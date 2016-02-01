#ifndef IINTEGRATOR_H
#define IINTEGRATOR_H

#include <memory>
class Simulation;

class IIntegrator
{
	protected :
	
	Simulation* sim;
	
	//---------------------------------------------
	//---------------------------------------------
	public :
	
	IIntegrator(Simulation* sim);
	
	virtual ~IIntegrator();
	
	virtual void integrate(float dt = 0.0001f) =0 ;
	
};



class ExplicitEulerIntegrator : public IIntegrator
{
	private :
	
	
	//---------------------------------------------
	//---------------------------------------------
	public :
	
	
	ExplicitEulerIntegrator(Simulation* sim);
	
	~ExplicitEulerIntegrator();
	
	virtual void integrate(float dt = 0.0001f) override;
			
};


#endif
