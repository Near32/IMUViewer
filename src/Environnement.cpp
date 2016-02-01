#include "Environnement.h"

Environnement::Environnement()
{
	fabriques = std::unique_ptr<Fabriques>( new Fabriques() );
	fabriques->registrer(ELFObstacle, new Fabricant<Obstacle>() );
	fabriques->registrer(ELFOrbeBonus, new Fabricant<OrbeBonus>() );			
}

/*Environnement::Environnement(std::vector<std::unique_ptr<IElement> > ListeElements_)
{
	ListeElements = ListeElements_;
}*/

Environnement::~Environnement()
{
	
}



void Environnement::addElement( std::unique_ptr<IElement> El)
{
	ListeElements.push_back( std::move(El) );
}

void Environnement::addElement( IElement* El)
{
	ListeElements.push_back( std::unique_ptr<IElement>(El) );
}
/*void Environnement::addElement( IElement El)
{
	ListeElements.push_back(El);
}*/

bool Environnement::deleteElementIDX(int idx)
{

	if(idx < ListeElements.size() )
	{
		ListeElements.erase(ListeElements.begin()+idx);
		
		return true;
	}

	return false;
}


bool Environnement::deleteElementSTRING(std::string name)
{
	bool ret = false;
	
	for(int i=ListeElements.size();i--;)
	{
		if( ListeElements[i]->name == name)
		{
			ret = true;
			ListeElements.erase(ListeElements.begin()+i);
		}
		
		//delete every Element named with name.
	}
	
	return ret;
}
