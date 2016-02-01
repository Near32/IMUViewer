#include "HelperFunctions.h"


//#define debug 

Mat<float> closestPointLOfBOXGivenPointL(RigidBody& rb, const Mat<float>& pointL)
{
	BoxShape& box = (BoxShape&)rb.getShapeReference();
	Mat<float> min((float)0,3,1);
	Mat<float> max((float)0,3,1);
	
	min.set( -box.getHeight()/2.0f, 1,1);
	min.set( -box.getWidth()/2.0f, 2,1);
	min.set( -box.getDepth()/2.0f, 3,1);
	max.set( -min.get(1,1), 1,1);
	max.set( -min.get(2,1), 2,1);
	max.set( -min.get(3,1), 3,1);
	
	
	Mat<float> clpL(3,1);
	//compute the closest point by projecting the point to the closest face using Voronoi regions approach.
	bool change = false;
	for(int i=1;i<=3;i++)
	{
		
		float v = pointL.get(i,1);
		if( v < min.get(i,1))
		{
			v = min.get(i,1);
			change = true;
		}
		if( v > max.get(i,1))
		{
			v = max.get(i,1);
			change = true;
		}
		
		clpL.set( v, i,1);
	}
	if(!change)
	{
		//then it means that the given point is to be inside the BoxShape :
#ifdef debug
std::cout << "COLLISION DETECTOR : CLPLL : inside point : " << std::endl;
operatorL(operatorL(clpL,pointL), rb.getPointInWorld(pointL)).afficher();
#endif			
	}
	
	return clpL;
}  

Mat<float> closestPointWOfBOXGivenPointL(RigidBody& rb, const Mat<float>& pointL)
{
	Mat<float> clpL( closestPointLOfBOXGivenPointL( rb, pointL) );
	return rb.getPointInWorld( clpL);
}

Mat<float> closestPointWOfBOXGivenPointW(RigidBody& rb, const Mat<float>& pointW)
{
	return closestPointWOfBOXGivenPointL( rb, rb.getPointInLocal(pointW) ) ;
}


bool testOBBPlane( RigidBody& box, RigidBody& plane)
{
	BoxShape boxs = (BoxShape&)box.getShapeReference();
	float d = ((PlaneShape&)plane.getShapeReference()).getDistance();
	Mat<float> n( ((PlaneShape&)plane.getShapeReference()).getNormal());
	
	Mat<float> e((float)0,3,3);
	for(int i=1;i<=3;i++)	e.set(1.0f, i,i);
	//identity matrix to be rotated in order to have the basis related to the coordinate frame of the box :
	e = extract( box.getTransformation(), 1,1, 3,3) * e;
	
	float r = boxs.getHeight()*fabs_( dotProduct( Cola(e,1), n) ) + boxs.getWidth()*fabs_( dotProduct( Cola(e,2), n) ) + boxs.getDepth()*fabs_( dotProduct( Cola(e,3), n) );
	
	float s = dotProduct( n, box.getPosition()) - d;
	
	return fabs_(s) <= r;
	
}

/*	Returns a matrix made up with the coordinates of intersecting points in the World frame of b1 that are within b2. If there are none, it returns a zero 3x8 matrix.
*/
Mat<float> testOBBOBB( RigidBody& b1, RigidBody& b2, bool& intersect)
{
	float precision = 1e-3f;
	Mat<float> ret((float)0,3,1);
	bool initialized = false;
	BoxShape& box1 = (BoxShape&)(b1.getShapeReference());
	BoxShape& box2 = (BoxShape&)(b2.getShapeReference());
	
	Mat<float> pointsB1((float)0,3,8);
	//register the position of the 8 points composing b1:
	Mat<float> min1((float)0,3,1);
	Mat<float> max1((float)0,3,1);
	
	min1.set( -box1.getHeight()/2.0f, 1,1);
	min1.set( -box1.getWidth()/2.0f, 2,1);
	min1.set( -box1.getDepth()/2.0f, 3,1);
	max1.set( -min1.get(1,1), 1,1);
	max1.set( -min1.get(2,1), 2,1);
	max1.set( -min1.get(3,1), 3,1);
	
	int col = 1;
	Mat<float> temp(3,1);
	Mat<float> voronoiTemp(3,1);
	
	for(int pm1=2;pm1--;)
	{
		for(int pm2=2;pm2--;)
		{
			for(int pm3=2;pm3--;)
			{
				Mat<float> tempL(3,1);
				tempL.set( pm1*( min1.get(1,1) ) + (1-pm1)*( max1.get(1,1) ), 1,1);
				tempL.set( pm2*( min1.get(2,1) ) + (1-pm2)*( max1.get(2,1) ), 2,1);
				tempL.set( pm3*( min1.get(3,1) ) + (1-pm3)*( max1.get(3,1) ), 3,1);
				
				
				//--------------------
				//let us compute its coordinate in the world frame :
				temp = b1.getPointInWorld( tempL);
#ifdef debug				
				b1.getPose().exp().afficher();
				b2.getPose().exp().afficher();
				tempL.afficher();
				temp.afficher();
#endif
				//----------------------
				//let us find its associated projected point :
				voronoiTemp = closestPointWOfBOXGivenPointW( b2, temp);
				
				//----------------------
				//let us find out if there was an intersection
				// <==> voronoiTemp = temp, because it would means that the projected point is already the closest to b2 for it is within it.
				if( equals(voronoiTemp,temp,precision) )
				{
					//let us refill pointsB1 with it :
					pointsB1.set( temp.get(1,1), 1,col);
					pointsB1.set( temp.get(2,1), 2,col);
					pointsB1.set( temp.get(3,1), 3,col);
					
					if(initialized)
					{
						ret = operatorL( ret, temp);
					}
					else
					{
						initialized = true;
						ret = temp;
					}
					intersect = true;
					
#ifdef debug
std::cout << "COLLISION DETECTOR : midnarrowphase : testOBBOBB : voronoi and temp : " << std::endl;
std::cout << " ids : b1 = " << b1.getID() << " ; b2 = " << b2.getID() << std::endl;
operatorL(voronoiTemp,temp).afficher();
//std::cout << "COLLISION POINTS :" << std::endl;
//ret.afficher();
//tempL.afficher();
#endif				
				}
				else
				{
#ifdef debug
std::cout << "COLLISION DETECTOR : ELSE : midnarrowphase : testOBBOBB : voronoi and temp : " << std::endl;
std::cout << " ids : b1 = " << b1.getID() << " ; b2 = " << b2.getID() << std::endl;
operatorL(voronoiTemp,temp).afficher();
//std::cout << "COLLISION POINTS :" << std::endl;
//ret.afficher();
//tempL.afficher();
#endif
				}
				
				
				col++;
			}
		}
	}
	
	return ret;
}

bool equals(const Mat<float>& a, const Mat<float>& b, float precision)
{
	if(a.getLine() == b.getLine() && a.getColumn() == b.getColumn())
	{
		for(int i=a.getLine();i--;)
		{
			for(int j=a.getColumn();j--;)
			{
				float vala = a.get(i+1,j+1);
				float valb = b.get(i+1,j+1);
				if( vala+precision < valb || vala-precision > valb)
				{
					return false;
				}
			}
		}
	}
	else
	{
		std::cerr << "EQUALS : matrices a and b are not of the same sizes." << std::endl;
		return false;
	}
		
	return true;
}

void innerVoronoiProjection(RigidBody& rb, Mat<float>& pointL)
{
	//given an inner point, let us find out its closest projection on the surface of the OBB :
	BoxShape& box = (BoxShape&)(rb.getShapeReference());
	Mat<float> min((float)0,3,1);
	Mat<float> max((float)0,3,1);
	
	min.set( -box.getHeight()/2.0f, 1,1);
	min.set( -box.getWidth()/2.0f, 2,1);
	min.set( -box.getDepth()/2.0f, 3,1);
	max.set( -min.get(1,1), 1,1);
	max.set( -min.get(2,1), 2,1);
	max.set( -min.get(3,1), 3,1);
	
	
	float distance[6];
	Mat<float> tempProj[6];
	int idxMin=3;
	float distMin=1e3f;
	int line = 1;
	for(int k=3;k--;)
	{
		tempProj[k] = pointL;
		tempProj[k].set( min.get(line,1), line,1);
		tempProj[k+3] = pointL;
		tempProj[k+3].set( max.get(line,1), line,1);
		
		distance[k] = norme2(pointL-tempProj[k]);
		distance[k+3] = norme2(pointL-tempProj[k+3]);
		
		if(distMin > distance[k])
		{
			distMin = distance[k];
			idxMin = k;
		}
		
		if(distMin > distance[k+3])
		{
			distMin = distance[k+3];
			idxMin = k+3;
		}
		
		line++;
	}
	
	pointL = tempProj[idxMin];
}


void innerVoronoiProjectionANDNormal(RigidBody& rb, Mat<float>& pointL, Mat<float>& normalL)
{
	//given an inner point, let us find out its closest projection on the surface of the OBB :
	//and the normal to the corresponding surface : the normal that is directed on the outside.
	BoxShape& box = (BoxShape&)(rb.getShapeReference());
	Mat<float> min((float)0,3,1);
	Mat<float> max((float)0,3,1);
	
	min.set( -box.getHeight()/2.0f, 1,1);
	min.set( -box.getWidth()/2.0f, 2,1);
	min.set( -box.getDepth()/2.0f, 3,1);
	max.set( -min.get(1,1), 1,1);
	max.set( -min.get(2,1), 2,1);
	max.set( -min.get(3,1), 3,1);
	
	
	float distance[6];
	Mat<float> tempProj[6];
	int idxMin=3;
	float distMin=1e3f;
	int line = 1;
	for(int k=3;k--;)
	{
		tempProj[k] = pointL;
		tempProj[k].set( min.get(line,1), line,1);
		tempProj[k+3] = pointL;
		tempProj[k+3].set( max.get(line,1), line,1);
		
		distance[k] = norme2(pointL-tempProj[k]);
		distance[k+3] = norme2(pointL-tempProj[k+3]);
		
		if(distMin > distance[k])
		{
			distMin = distance[k];
			idxMin = k;
		}
		
		if(distMin > distance[k+3])
		{
			distMin = distance[k+3];
			idxMin = k+3;
		}
		
		line++;
	}
	
	pointL = tempProj[idxMin];
	normalL = Mat<float>(0.0f,3,1);
	
	switch(idxMin)
	{
		case 0 :
		//it is min on the z face :
		normalL.set( -1.0f,3,1);
		break;
		
		case 1 :
		//it is min on the y face :
		normalL.set( -1.0f,2,1);
		break;
		
		case 2 :
		//it is min on the x face :
		normalL.set( -1.0f,1,1);
		break;
		
		case 3 :
		//it is max on the z face :
		normalL.set( 1.0f,3,1);
		break;
		
		case 4 :
		//it is max on the y face :
		normalL.set( 1.0f,2,1);
		break;
		
		case 5 :
		//it is max on the x face :
		normalL.set( 1.0f,1,1);
		break;
	}
	
}


