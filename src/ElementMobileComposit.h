#ifndef ELEMENTMOBILECOMPOSIT_H
#define ELEMENTMOBILECOMPOSIT_H

#include <vector>

#include "IElementMobile.h"



class ElementMobileComposit : public IElementMobile
{
	public:
		
		std::vector<std::unique_ptr<IElementMobile> >  collectionElementMobile;
		
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
		
		
		ElementMobileComposit();
		ElementMobileComposit(std::string name_, std::unique_ptr<se3> pose_);
		
		virtual ~ElementMobileComposit();
	
		

		void addElementMobile(std::unique_ptr<IElementMobile> EleMobile);
		void addElementMobile( IElementMobile* EleMobile);
		
		virtual bool isComposit()	override
		{
		 	return true;
		}

};


#endif


