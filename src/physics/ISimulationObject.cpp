#include "ISimulationObject.h"

ISimulationObject::ISimulationObject() : name(std::string("not mentioned")), id(0), isActive(true), type(TSOUndefined)
{

}

ISimulationObject::ISimulationObject(int id_) : name(std::string("not mentioned")), id(id_), isActive(true), type(TSOUndefined)
{

}

ISimulationObject::ISimulationObject(std::string name_, int id_, bool isActive_) : name(name_), id(id_), isActive(isActive_), type(TSOUndefined)
{

}

ISimulationObject::~ISimulationObject()
{

}

