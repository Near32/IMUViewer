#ifndef FABRIQUES_H
#define FABRIQUES_H

#include "IElementFixe.h"
#include "TypeELF.h"
#include "IFabricant.h"
#include "Fabricant.h"
#include <map>

class Fabriques
{
	public :
	
	std::map<TypeELF, std::unique_ptr<IFabricant> > collectionFabricants;
	
	//-------------------------------------------------------
	//-------------------------------------------------------
	
	Fabriques();
	~Fabriques();
	
	void registrer(TypeELF typeELF, std::unique_ptr<IFabricant> ptrIFabricant);
	void registrer(TypeELF typeELF, IFabricant* ptrIFabricant);
	
	IElementFixe* fabriquer(TypeELF typeELF, std::string name_, std::unique_ptr<se3> pose_, const Mat<float>& hwd);
	IElementFixe* fabriquer(TypeELF typeELF, std::string name_, se3* pose_,  const Mat<float>& hwd);
	
};

#endif
