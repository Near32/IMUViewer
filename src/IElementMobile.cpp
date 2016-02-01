#include "IElementMobile.h"

IElementMobile::IElementMobile() :IElement()
{
	hwd = Mat<float>(1.0f,3,1);
}


IElementMobile::IElementMobile(std::string name_, std::unique_ptr<se3> pose_) : IElement( name_, std::move(pose_) )
{
	hwd = Mat<float>(1.0f,3,1);
}

IElementMobile::IElementMobile(std::string name_, std::unique_ptr<se3> pose_, const Mat<float>& hwd_) : hwd(hwd_),IElement(name_, std::move(pose_) )
{

}

IElementMobile::IElementMobile(std::string name_,se3* pose_, const Mat<float>& hwd_) : IElement(name_, pose_ ), hwd(hwd_)
{

}

IElementMobile::~IElementMobile()
{

}

bool IElementMobile::isFixe()
{
  return false;
}

