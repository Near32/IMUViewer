#include "Contact.h"

/*
Contact::Contact()
{

}
*/

Contact::Contact( Contact& c) : rbA(c.rbA), rbB(c.rbB)
{
	contactPoint = c.contactPoint;
	normal = c.normal;
}

Contact::Contact(const Contact& c) : rbA(c.rbA), rbB(c.rbB)
{
	contactPoint = c.contactPoint;
	normal = c.normal;
}

Contact::Contact(RigidBody* rbA_, RigidBody* rbB_) : rbA(rbA_), rbB(rbB_)
{
	
}

Contact::~Contact()
{

}

Contact& Contact::operator=(Contact& c)
{
	rbA = c.rbA;
	rbB = c.rbB;
	
	contactPoint = c.contactPoint;
	normal = c.normal;
	
	return *this;
}

Contact Contact::operator=(Contact c)
{
	rbA = c.rbA;
	rbB = c.rbB;
	
	contactPoint = c.contactPoint;
	normal = c.normal;
	
	return *this;
}
