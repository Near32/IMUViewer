#ifndef IELEMENTMOBILE_H
#define IELEMENTMOBILE_H

#include "IElement.h"

class IElementMobile : public IElement
{
	public:  

	Mat<float> hwd;
	
	
	//--------------------------------------
	//--------------------------------------
	
		
		 IElementMobile();
		 
		 IElementMobile(std::string name_, std::unique_ptr<se3> pose_);
		 IElementMobile(std::string name_, std::unique_ptr<se3> pose_, const Mat<float>& hwd_);
		 IElementMobile(std::string name_,se3* pose_, const Mat<float>& hwd_);
		 
		 virtual ~IElementMobile(); // Le desctructeur d'une interface doit être virtuel (c.f. wikipedia objet composite)
		
		  
		 virtual bool isFixe() override;
		 
		 virtual bool isComposit() = 0;
		  
};

#endif

