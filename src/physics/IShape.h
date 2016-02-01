#ifndef ISHAPE_H
#define ISHAPE_H

#include "IMoveable.h"
#include <vector>
#include <memory>

enum ShapeType{
	ABSTRACT,
	PLANE,
	SPHERE,
	BOX,
	COMPOSIT
};

class RigidBody;


class IShape : public IMoveable
{
	protected :
	
	float boundingRadius;
	bool hasMoved;
	RigidBody* Owner;
	ShapeType type;
	
	
	public :
	
	
	IShape(RigidBody* Owner_, float boundingRadius_ = 1.0f);
	IShape( const se3& Pose_, RigidBody* Owner_, float boundingRadius_ = 1.0f);
	IShape( const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel, RigidBody* Owner_, float boundingRadius_ = 1.0f);
	
	~IShape();
	
	
	//-----------------------
	ShapeType getShapeType()	{	return type;	}
	
	float getBRadius()	{	return boundingRadius;	}
	bool getMovingState()	{	return hasMoved;	}
	RigidBody* getOwner()	{ 	return Owner;	}
	const ShapeType& getShapeType() const	{	return type;	}
	
	void setShapeType(enum ShapeType st)	{	type = st;	}
	void setBRadius(const float br)	{	if(br>0.0f)	boundingRadius = br;	}
	void setMovingState(const bool hm)	{	hasMoved = hm;	}
	void setOwner( RigidBody* o)	{	if(o!=NULL)	Owner = o;	}
	
};

//Infinite plane :
class PlaneShape : public IShape
{
	private :
	float distance;
	Mat<float> normal;
	
	public :
	
	//TODO : mind the boundingRadius according to the shape, maybe ?
	
	PlaneShape(RigidBody* Owner_, float boundingRadius_ = 1.0f) : IShape( Owner_, boundingRadius_), distance(0.0f)
	{
		type = PLANE;
		normal = Mat<float>(0.0f,3,1);
		normal.set(1.0f, 3,1);
		// normal along z.
	}
	
	PlaneShape(const float& distance_, const Mat<float>& normal_, RigidBody* Owner_, float boundingRadius_ = 1.0f) : IShape( Owner_, boundingRadius_), distance(distance_), normal(normal_)
	{
		 type = PLANE;
	}
	
	PlaneShape( const se3& Pose_, RigidBody* Owner_, float boundingRadius_ = 1.0f) : IShape( Pose_, Owner_, boundingRadius_), distance(0.0f)
	{
		 type = PLANE;
		normal = Mat<float>(0.0f,3,1);
		normal.set(1.0f, 3,1);
		// normal along z.
	}
	
	
	PlaneShape( const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel, RigidBody* Owner_, float boundingRadius_ = 1.0f) : IShape( Pose_, Lvel, Avel, Owner_, boundingRadius_), distance(0.0f)
	{
		 type = PLANE;
		normal = Mat<float>(0.0f,3,1);
		normal.set(1.0f, 3,1);
		// normal along z.
	}
	
	
	~PlaneShape()
	{
	
	}
	
	
	const float getDistance() const	{	return distance;	}
	const Mat<float> getNormal() const	{	return normal;	}
	
};


class SphereShape : public IShape
{
	private :
	
	float radius;
	
	//---------------------------------------
	//---------------------------------------
	
	public :
	
	SphereShape(RigidBody* Owner_, float boundingRadius_ = 1.0f) : radius(boundingRadius), IShape( Owner_, boundingRadius_)
	{
		type = SPHERE;
	}
	
	SphereShape( const se3& Pose_, RigidBody* Owner_, float boundingRadius_ = 1.0f) : radius(boundingRadius), IShape( Pose_, Owner_, boundingRadius_)
	{
		type = SPHERE;
	}
	
	SphereShape( const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel, RigidBody* Owner_, float boundingRadius_ = 1.0f) : radius(boundingRadius), IShape( Pose_, Lvel, Avel, Owner_, boundingRadius_)
	{
		type = SPHERE;
	}
	
	~SphereShape();
	
	
	//---------------------
	float getRadius()	{	return radius;	}
	void setRadius( const float r)	{	if(r>0.0f)	radius = r;	}
	
};



class BoxShape : public IShape
{
	private :
	
	float height;
	float width;
	float depth;
	
	//---------------------------------------------
	//----------------------------------------------
	
	
	public :
	
	//TODO : mind the boundingradius ?
	
	BoxShape(RigidBody* Owner_, float boundingRadius_ = 1.0f) : IShape( Owner_, boundingRadius_)
	{
		type = BOX;
		height = sqrt(2);
		width = sqrt(2);
		depth = sqrt(2);
	}
	
	BoxShape( const se3& Pose_, RigidBody* Owner_, float boundingRadius_ = 1.0f) : IShape( Pose_, Owner_, boundingRadius_)
	{
		type = BOX;
		height = sqrt(2);
		width = sqrt(2);
		depth = sqrt(2);
	}
	
	BoxShape( const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel, RigidBody* Owner_, float boundingRadius_ = 1.0f) : IShape( Pose_, Lvel, Avel, Owner_, boundingRadius_)
	{
		type = BOX;
		height = sqrt(2);
		width = sqrt(2);
		depth = sqrt(2);
	}
	
	//---------------
	
	BoxShape(RigidBody* Owner_, const Mat<float>& hwd) : IShape( Owner_, 1.0f)
	{
		type = BOX;
		if( hwd.getLine() == 3)
		{
			height = hwd.get(1,1);
			width = hwd.get(2,1);
			depth = hwd.get(3,1);
			
			float br = max(hwd);
			float other = height;
			if( br == other)
				other = width;
			
			br = sqrt( (br*br+other*other) )/2.0f;
			setBRadius(br);
		}
		else
		{
			height = sqrt(2);
			width = sqrt(2);
			depth = sqrt(2);
		}
	}
	
	BoxShape( const se3& Pose_, RigidBody* Owner_, const Mat<float>& hwd) : IShape( Pose_, Owner_, 1.0f)
	{
		type = BOX;
		if( hwd.getLine() == 3)
		{
			height = hwd.get(1,1);
			width = hwd.get(2,1);
			depth = hwd.get(3,1);
			
			float br = max(hwd);
			float other = height;
			if( br == other)
				other = width;
			
			br = sqrt( (br*br+other*other) )/2.0f;
			setBRadius(br);
		}
		else
		{
			height = sqrt(2);
			width = sqrt(2);
			depth = sqrt(2);
		}
	}
	
	BoxShape( const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel, RigidBody* Owner_, const Mat<float> hwd) : IShape( Pose_, Lvel, Avel, Owner_)
	{
		type = BOX;
		if( hwd.getLine() == 3)
		{
			height = hwd.get(1,1);
			width = hwd.get(2,1);
			depth = hwd.get(3,1);
			
			float br = max(hwd);
			float other = height;
			if( br == other)
				other = width;
			
			br = sqrt( (br*br+other*other) )/2.0f;
			setBRadius(br);
		}
		else
		{
			height = sqrt(2);
			width = sqrt(2);
			depth = sqrt(2);
		}
	}
	
	
	~BoxShape();
	
	
	//---------------------------
	float getHeight()	{	return height;	}
	float getWidth()	{	return width;	}
	float getDepth()	{	return depth;	}
	Mat<float> getHWD()	{	Mat<float> hwd(3,1);hwd.set(height,1,1);hwd.set(width,2,1);hwd.set(depth,3,1);return hwd;	}
	
	void setHeight(const float h)	
	{	
		if(h>0.0f)	
		{
			float max = height;
			if(max<width)
				max = width;
			if(max<depth)
				max = depth;
			
			height = h;	
			if(h>max)
			{
				//verify that the boundingRadius has not to be changed...:
				float br = height;
				float other = depth;
				if( br == other)
					other = width;
			
				br = sqrt( (br*br+other*other) )/2.0f;
				setBRadius(br);
			}
		}
	}
	
	void setWidth(const float w)	
	{	
		if(w>0.0f)	
		{
			float max = height;
			if(max<width)
				max = width;
			if(max<depth)
				max = depth;
			
			width = w;	
			if(w>max)
			{
				//verify that the boundingRadius has not to be changed...:
				float br = width;
				float other = height;
				if( br == other)
					other = depth;
			
				br = sqrt( (br*br+other*other) )/2.0f;
				setBRadius(br);
			}
		}
	}
	
	void setDepth(const float d)	
	{	
		if(d>0.0f)	
		{
			float max = height;
			if(max<width)
				max = width;
			if(max<depth)
				max = depth;
			
			depth = d;	
			if(d>max)
			{
				//verify that the boundingRadius has not to be changed...:
				float br = depth;
				float other = height;
				if( br == other)
					other = width;
			
				br = sqrt( (br*br+other*other) )/2.0f;
				setBRadius(br);
			}
		}
	}
	
	void setHWD(const Mat<float>& hwd_)
	{
		setHeight(hwd_.get(1,1));
		setWidth(hwd_.get(2,1));
		setDepth(hwd_.get(3,1));
	}
};



class CompositShape : public IShape
{
	public :
	
	int nbrShapes;
	std::vector<std::unique_ptr<IShape> > Shapes;
	
	
	//----------------------------------
	
	// TODO : MIND THE BOUNDINGRADIUS  OF THE COMPOSITION !!!!
	
	CompositShape(RigidBody* Owner_, float boundingRadius_ = 1.0f);
	CompositShape(RigidBody* Owner_, std::vector<std::unique_ptr<IShape> > Shapes_);
	
	~CompositShape();
	
	void addShape( std::unique_ptr<IShape> shape);
	void removeShape( int id);
	

};


#endif 
