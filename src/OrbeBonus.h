#ifndef ORBEBONUS_H
#define ORBEBONUS_H

#include "IElementFixe.h"
#include "TypeOrbeBonus.h"

class OrbeBonus : IElementFixe
{
	public :
	TypeOrbeBonus type;
	
	//-----------------------------------------
	//-----------------------------------------
	
		
	OrbeBonus();
	OrbeBonus( std::string name_, std::unique_ptr<se3> pose_);
	OrbeBonus( TypeOrbeBonus type_, std::string name_, std::unique_ptr<se3> pose_);
	OrbeBonus( std::string name_, std::unique_ptr<se3> pose_, const Mat<float>& hwd_);
	OrbeBonus( TypeOrbeBonus type_, std::string name_, std::unique_ptr<se3> pose_, const Mat<float>& hwd_);
	
	~OrbeBonus();
	
	virtual bool isObstacle() override ;
};

#endif
