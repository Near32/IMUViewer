#include "IShape.h"
#include "RigidBody.h"

IShape::IShape(RigidBody* Owner_, float boundingRadius_) : boundingRadius(boundingRadius_), hasMoved(false), Owner(Owner_), type(ABSTRACT)
{
	IMoveable();
}


IShape::IShape( const se3& Pose_, RigidBody* Owner_, float boundingRadius_) : IMoveable(Pose_), boundingRadius(boundingRadius_), hasMoved(false), Owner(Owner_), type(ABSTRACT)
{
	
}

IShape::IShape( const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel, RigidBody* Owner_, float boundingRadius_) : 	IMoveable(Pose_,Lvel,Avel), boundingRadius(boundingRadius_), hasMoved(false), Owner(Owner_), type(ABSTRACT)
{

}

IShape::~IShape()
{

}

//--------------------------------
//--------------------------------


SphereShape::~SphereShape()
{

}

//--------------------------------
//--------------------------------



BoxShape::~BoxShape()
{

}

//--------------------------------
//--------------------------------


//--------------------------------
//--------------------------------





CompositShape::CompositShape(RigidBody* Owner_, float boundingRadius_) : IShape(Owner_,boundingRadius), nbrShapes(0)
{
	type = COMPOSIT;	
}
	
CompositShape::CompositShape(RigidBody* Owner_, std::vector<std::unique_ptr<IShape> > Shapes_) : IShape(Owner_, (float)0)
{
	type = COMPOSIT;	
	nbrShapes = Shapes.size();
	//Shapes = Shapes_;
	for(int i=nbrShapes;i--;)	Shapes.insert( Shapes.begin(), std::move( Shapes_[i] ));
		
	
	//TODO boundingRadius ? 
}

CompositShape::~CompositShape()
{

}

void CompositShape::addShape( std::unique_ptr<IShape> shape)
{
	nbrShapes++;
	Shapes.insert( Shapes.end(), std::move(shape));
}

void CompositShape::removeShape( int id)
{
	if(id<=0 && id < Shapes.size())
	{
		Shapes.erase(Shapes.begin()+id);
	}
}






//--------------------------------
//--------------------------------


//--------------------------------
//--------------------------------




