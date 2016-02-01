#ifndef CONSTRAINTSLIST_H
#define CONSTRAINTSLIST_H

#include <memory>
#include <string>
#include <vector>
#include "../utils/Mat/Mat.h"

enum ConstraintType{
	CTIC,
	CTContactConstraint,
	CTHingeJoint,
	CTBallAndSocketJoint,
	CTLimitConstraint
};

typedef struct ConstraintInfo{
	std::string nameEl1;
	std::string nameEl2;
	ConstraintType ct;
	Mat<float> data;
	//contains the further informations needed to create a constraints :
	//BASJ :
	//anchorsAL, anchorsBL
	//HJ :
	//HJaxisAL,AnchorAL
	//...
	//TODO : contact and limit
	
	ConstraintInfo(std::string nameEl1_, std::string nameEl2_, ConstraintType ct_, const Mat<float>& data_) : nameEl1(nameEl1_), nameEl2(nameEl2_), ct(ct_), data(data_)
	{
	
	}
} ConstraintInfo;

typedef std::vector<ConstraintInfo> ConstraintsList;

#endif
