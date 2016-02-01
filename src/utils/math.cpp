#include "math.h"


//#define debuglvl1

//------------------------------------
//		Identity Matrix 3x3 :
//------------------------------------
float id3_val_array[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
Mat<float> Identity3( id3_val_array,3,3);

float xaxis_val_array[3] = {1.0f, 0.0f, 0.0f};
Mat<float> XAxis( xaxis_val_array,3,1);

float yaxis_val_array[3] = {0.0f, 1.0f, 0.0f};
Mat<float> YAxis( yaxis_val_array,3,1);

float zaxis_val_array[3] = {0.0f, 0.0f, 1.0f};
Mat<float> ZAxis( zaxis_val_array,3,1);
//------------------------------------
//------------------------------------
float EPSILON = (float)1e-100;
float INF = (float)1e100;

//------------------------------------
//------------------------------------


//------------------------------------
//------------------------------------

se3::se3() : hasChanged(true)	//so that SE3 would be computed on the first access...
{
	try
	{
		t = new Mat<float>((float)0,3,1);
		w = new Mat<float>((float)0,3,1);
		SE3 = new Mat<float>((float)0, 4,4);
	}
	catch( std::exception& e)
	{
		std::cout << e.what() << std::endl;
		throw e;
	}
}

se3::se3(const Mat<float>& w_, const Mat<float>& t_) : hasChanged(true)	//so that SE3 would be computed on the first access...
{
	try
	{
		t = new Mat<float>(t_);
		w = new Mat<float>(w_);
		SE3 = new Mat<float>((float)0, 4,4);
	}
	catch( std::exception& e)
	{
		std::cout << e.what() << std::endl;
		throw e;
	}
	
}

se3::se3(const Mat<float>& t_) : t( new Mat<float>(t_)),w( new Mat<float>(0.0f,3,1)), SE3( new Mat<float>((float)0, 4,4) ), hasChanged(true)	//so that SE3 would be computed on the first access...
{

}

se3::se3(const float* w_t_array) : SE3( new Mat<float>((float)0,4,4) ), hasChanged(true)
{
	try
	{
		t = new Mat<float>(3,1);
		w = new Mat<float>(3,1);
	}
	catch(std::exception& e)
	{
		std::cout << e.what() << std::endl;
		throw e;
	}
	
	this->w->set( w_t_array[0], 1,1);
	this->w->set( w_t_array[1], 2,1);
	this->w->set( w_t_array[2], 3,1);
	
	this->t->set( w_t_array[3], 1,1);
	this->t->set( w_t_array[4], 2,1);
	this->t->set( w_t_array[5], 3,1);
}
	
se3::se3(const se3& x) : hasChanged(true)
{
	try
	{
		t = new Mat<float>(x.getT());
		w = new Mat<float>(x.getW());
		SE3 = new Mat<float>(x.getSE3());
	}
	catch( std::exception& e)
	{
		std::cout << e.what() << std::endl;
		throw e;
	}
}

se3::~se3()
{
	delete t;
	delete w;
	delete SE3;
}
	
Mat<float> se3::exp()					//compute the (R | t) matrix.
{
	if(hasChanged)
	{
#ifdef debuglvl1	
		std::cout << " EXP : w et t : " << std::endl;
		w->afficher();
		t->afficher();
#endif		
		*SE3 = expM( operatorC( *w, *t) );
		hasChanged = false;
	}
	
	return *SE3;
}

void se3::setT(const Mat<float>& t_)
{
	if(t_.getLine() == 3 && t_.getColumn() == 1)
	{
		*t = t_;
		hasChanged = true;
	}
}

void se3::setW(const Mat<float>& w_)
{
	if(w_.getLine() == 3 && w_.getColumn() == 1)
	{
		*(this->w) = w_;
		hasChanged = true;
	}
}

void se3::setOrientation( const Quat& q)
{
	Mat<float> so3((float)0,3,3);
	Quat2SO3(q,&so3);
	
	Mat<float> wX( logMEuler(so3));
	w->set( -wX.get(2,3), 1,1);
	w->set( wX.get(1,3), 2,1);
	w->set( -wX.get(1,2), 3,1);
	
#ifdef debuglvl1
	std::cout << " SE3 : TEST APRES MODIF : " << std::endl;
	w->afficher();
	wX.afficher();
#endif	
	
	hasChanged = true;
}


se3& se3::operator=(const se3& x)
{
	this->~se3();
	
	try
	{
		this->t = new Mat<float>(x.getT());
		this->w = new Mat<float>(x.getW());
		this->SE3 = new Mat<float>(x.getSE3());
	}
	catch(std::exception& e)
	{
		std::cout << e.what() << std::endl;
		throw e;
	}
	
	this->hasChanged = true;
	
	return *this;
}

Mat<float> se3::getT()	const
{	
	return *t;	
}

Mat<float> se3::getW()	const
{	
	return *w;
}

Mat<float> se3::getSE3()	const
{	
	return *SE3;
}



	//---------------------------------------------------------
	//---------------------------------------------------------
	
	

Mat<float> crossproductV( const Mat<float>& p1, const Mat<float>& p2)
{
	return crossProduct(p1)*p2;
}
