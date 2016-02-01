#include "ElementMobileComposit.h"

ElementMobileComposit::ElementMobileComposit() : IElementMobile()
{
	
}

ElementMobileComposit::ElementMobileComposit(std::string name_, std::unique_ptr<se3> pose_) : IElementMobile(name_, std::move(pose_) ) 
{
	
}



ElementMobileComposit::~ElementMobileComposit()
{

}


void ElementMobileComposit::addElementMobile(std::unique_ptr<IElementMobile> EleMobile)
{
    collectionElementMobile.push_back( std::move(EleMobile));
}

void ElementMobileComposit::addElementMobile(IElementMobile* EleMobile)
{
    collectionElementMobile.push_back( std::unique_ptr<IElementMobile>( EleMobile) );
}
