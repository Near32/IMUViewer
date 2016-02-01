#ifndef IFABRICANT_H
#define IFABRICANT_H

#include "IElementFixe.h"

class IFabricant
{
	public :
	
	IFabricant();
	virtual ~IFabricant();
	
	virtual IElementFixe* fabriquer(std::string name_, std::unique_ptr<se3> pose_, const Mat<float>& hwd_) = 0;
	virtual IElementFixe* fabriquer(std::string name_, se3* pose_, const Mat<float>& hwd_) = 0;
	
};

#endif
