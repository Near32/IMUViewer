#ifndef IELEMENT_H
#define IELEMENT_H

#include <iostream>
#include <memory>

#include <string>
#include "utils/math.h"


class IElement
{
	public :
	
	std::string name;
	std::unique_ptr<se3> pose;
	
	//-----------------------------
	//-----------------------------
	
	
	IElement();
	IElement( std::string name, std::unique_ptr<se3> pose_ );
	IElement( std::string name, se3* pose_ );
	
	~IElement();
	
	virtual bool isFixe() =0 ;
	
	se3& getPoseReference() const
	{
		return *((se3*)(pose.get()));
	}
	
	void setPose( const se3& p)
	{
		pose.reset( new se3(p) );
		//(*pose) = p;
	}
	
	std::string getName()	const
	{
		return name;
	}
	
};


#endif
