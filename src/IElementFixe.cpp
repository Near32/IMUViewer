#include "IElementFixe.h"


IElementFixe::IElementFixe() :  IElement()
{
	hwd = Mat<float>( 1.0f, 3,1);
}

IElementFixe::IElementFixe(std::string name_, std::unique_ptr<se3> pose_) : IElement(name_, std::move(pose_) )
{
	hwd = Mat<float>( 1.0f, 3,1);
}

IElementFixe::IElementFixe(std::string name_, std::unique_ptr<se3> pose_, const Mat<float>& hwd_) : IElement(name_, std::move(pose_) )
{

	if(hwd_.getLine() == 3 && hwd.getColumn() == 1)
	{
		hwd = hwd_;
	}
	else
	{
		hwd = Mat<float>( 1.0f, 3,1);
	}
}


IElementFixe::~IElementFixe()
{

}

bool IElementFixe::isFixe()
{
  return true;
}
