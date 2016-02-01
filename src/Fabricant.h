#ifndef FABRICANT_H
#define FABRICANT_H

#include "IFabricant.h"
#include "Obstacle.h"
#include "OrbeBonus.h"


template<class T>
class Fabricant : public IFabricant
{
	public :
	
	Fabricant() : IFabricant()
	{
	
	}
	
	~Fabricant()
	{
	
	}
	
	virtual IElementFixe* fabriquer( std::string name_, std::unique_ptr<se3> pose_, const Mat<float>& hwd_) override
	{
		return (IElementFixe*)new T(name_,std::move(pose_), hwd_);
	}
	
	virtual IElementFixe* fabriquer( std::string name_, se3* pose_, const Mat<float>& hwd_) override
	{
		return (IElementFixe*)new T(name_, std::unique_ptr<se3>(pose_), hwd_ );
	}
};

#endif
