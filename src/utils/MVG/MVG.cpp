#include "MVG.h"

//#define debuglvl1

float trace(const Mat<float>& m)
{
	float sum = 0.0f;
	for(int i=m.getLine();i--;)
		sum += m.get(i+1,i+1);
	return sum;
}

Mat<float> logM( const Mat<float>& m)
{
	float ctheta = (trace(m)-1.0f)/2.0f;
	
#ifdef debuglvl1
	std::cout << " LOGM : COS THETA ROTATION  = " << ctheta << std::endl; 
#endif	

	float precision = (float)1e-10;
	if( ctheta <= 1.0f+precision && ctheta >= 1.0f-precision)
	{
		return Mat<float>(0.0f,3,3); 
	}
	else
	{
		float theta = acosf(ctheta);
		Mat<float> logm( m-transpose(m));
		return ((float)(theta/(2.0f*sinf(theta)))) * logm;
	}
}

Mat<float> rotation(float angle, int axis)
{
	Mat<float> r(0.0f,3,3);
	
	switch(axis)
	{
		case 1 :	//X axis		
		r.set(cos(angle), 2,2);
		r.set(cos(angle), 3,3);
		r.set(sin(angle), 2,3);
		r.set(-sin(angle), 3,2);
		r.set( 1.0f, 1,1);
		return r;
		break;
		
		case 2 : 	//Y axis
		r.set(cos(angle), 1,1);
		r.set(cos(angle), 3,3);
		r.set(sin(angle), 1,3);
		r.set(-sin(angle), 3,1);
		r.set( 1.0f, 2,2);
		return r;
		break;
		
		case 3 :	//Z axis
		r.set(cos(angle), 1,1);
		r.set(cos(angle), 2,2);
		r.set(-sin(angle), 1,2);
		r.set(sin(angle), 2,1);
		r.set( 1.0f, 3,3);
		return r;
		break;
	}
}

Mat<float> logMEuler( const Mat<float>& m)
{
	Mat<float> angles(3,1);
	Rot2Euler( m, angles);

#ifdef debuglvl1	
	cout << " Angles Euler : " << endl;
	angles.afficher();
#endif	
	
	Mat<float> r1( rotation(angles.get(1,1),1) );
	Mat<float> r2( rotation(angles.get(2,1),2) );
	Mat<float> r3( rotation(angles.get(3,1),3) );
	
#ifdef debuglvl1
	std::cout << " LOGMEULERs : ROTATIONS" << std::endl;
	r1.afficher();
	r2.afficher();
	r3.afficher();
#endif	

	r1 = logM(r1);
	r2 = logM(r2);
	r3 = logM(r3);

#ifdef debuglvl1
	std::cout << " LOGMEULERs : " << std::endl;
	r1.afficher();
	r2.afficher();
	r3.afficher();
#endif	
	return r1+r2+r3 ;
}

Mat<float> expW3( const Mat<float>& w)
{
	Mat<float> w1(0.0f,3,1);
	Mat<float> w2(0.0f,3,1);
	Mat<float> w3(0.0f,3,1);
	w1.set( w.get(1,1), 1,1);
	w2.set( w.get(2,1), 2,1);
	w3.set( w.get(3,1), 3,1);		
	
	return expW(w1)*expW(w2)*expW(w3);
}


/* Returns the corresponding roll pitch yaw euler angles from SO(3) matrix.*/
/*void Rot2Euler( const Mat<float>& R, Mat<float>& angles)
{
	float epsilon = (float)1e-10;
	float R31 = R.get(3,1);
	 
	if(  (R31 < 1+epsilon && R31 > 1-epsilon) || ( R31 > -1-epsilon && R31 < -1+epsilon ) )
	{
		angles.set( 0.0f, 3,1);	//yaw = 0;
		
		if( R31 > -1-epsilon && R31 < -1+epsilon )
		{
			angles.set( PI/2, 2,1);	//pitch = PI/2
			angles.set( atan21(R.get(1,2),R.get(1,3)), 1,1 );	//roll = atan2(R12,R13)
		}
		else
		{
			angles.set( -PI/2, 2,1);	//pitch = -PI/2
			angles.set( atan21(-R.get(1,2),-R.get(1,3)), 1,1 );	//roll = atan2(-R12,-R13)
		}
	}
	else
	{
		angles.set( -asin(R31), 2,1);	//pitch = -asin(R31)
		float cTheta1 = cos(angles.get(2,1));
		angles.set( atan21( R.get(3,2)/cTheta1, R.get(3,3)/cTheta1 ), 1,1);	//roll = atan2( R32/cos(pitch), R33/cos(pitch) )
		angles.set( atan21( R.get(2,1)/cTheta1, R.get(1,1)/cTheta1 ), 3,1);	//yaw = atan2( R21/cos(pitch), R11/cos(pitch) )
		
	}
}
*/
