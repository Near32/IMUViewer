#include "Fabriques.h"

Fabriques::Fabriques()
{

}

Fabriques::~Fabriques()
{

}
	
void Fabriques::registrer(TypeELF typeELF, std::unique_ptr<IFabricant> ptrIFabricant)
{
	//collectionFabricants.insert( std::pair<TypeELF,std::unique_ptr<IFabricant> >(typeELF, std::move(ptrIFabricant) ) );
	collectionFabricants[typeELF] =  std::move(ptrIFabricant) ;
}

void Fabriques::registrer(TypeELF typeELF, IFabricant* ptrIFabricant)
{
	//collectionFabricants.insert( std::pair<TypeELF,std::unique_ptr<IFabricant> >(typeELF, std::unique_ptr<IFabricant>(ptrIFabricant) ) );
	collectionFabricants[typeELF] = std::unique_ptr<IFabricant>( ptrIFabricant )  ;
}

IElementFixe* Fabriques::fabriquer(TypeELF typeELF, std::string name_, std::unique_ptr<se3> pose_, const Mat<float>& hwd)
{
	return collectionFabricants[typeELF]->fabriquer(name_, std::move(pose_), hwd );
}

IElementFixe* Fabriques::fabriquer(TypeELF typeELF, std::string name_, se3* pose_, const Mat<float>& hwd)
{
	return collectionFabricants[typeELF]->fabriquer(name_, pose_, hwd );
}
