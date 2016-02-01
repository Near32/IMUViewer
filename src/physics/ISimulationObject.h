#ifndef ISIMULATIONOBJECT_H
#define ISIMULATIONOBJECT_H

#include <iostream>
#include <string>
#include "../utils/math.h"
#include "TypeSimulationObject.h"

class ISimulationObject 
{
	protected :
	
	std::string name;	//name of the object
	int id;				//unique id of the object
	TypeSimulationObject type;
	
	public :

	bool isActive;		//indicates if the object is to be taken care by the simulation, or not.	
	
	//--------------------------------------
	//--------------------------------------
		
	ISimulationObject();
	ISimulationObject(int id_);
	ISimulationObject( std::string name_, int id_, bool isActive_ = true);
	
	~ISimulationObject();
	
	virtual void Render(const se3& WorldTransformation) = 0;	//abstract method that renders information for debug purpose, for instance.
	
	
	//---------------------------------
	TypeSimulationObject getType()	const	{	return type;	}
	int getID()	const	{	return id;	}
	std::string getName() const	{	return name;	}

};
#endif
