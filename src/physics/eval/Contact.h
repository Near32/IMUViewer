#ifndef CONTACT_H
#define CONTACT_H

#include "../RigidBody.h"

class Contact
{
	public:
	
	RigidBody* rbA;
	RigidBody* rbB;
	std::vector<Mat<float> > contactPoint;	//there can be multiple contact points...
	std::vector<Mat<float> > normal;
	
	//Contact();
	Contact(Contact& c); 
	Contact(const Contact& c);
	
	Contact(RigidBody* rbA_, RigidBody* rbB_);
	~Contact();
	
	Contact& operator=(Contact& c);
	Contact operator=(Contact c);
	
};

#endif
