#include "Quaternion.h"


/* Returns quaternion product qL * qR. */
Quat Qt_Mul(Quat qL, Quat qR)
{
	Quat qq;
	qq.w = qL.w*qR.w - qL.x*qR.x - qL.y*qR.y - qL.z*qR.z;
	qq.x = qL.w*qR.x + qL.x*qR.w + qL.y*qR.z - qL.z*qR.y;
	qq.y = qL.w*qR.y + qL.y*qR.w + qL.z*qR.x - qL.x*qR.z;
	qq.z = qL.w*qR.z + qL.z*qR.w + qL.x*qR.y - qL.y*qR.x;
	return (qq);
}

Quat operator*(float val,const Quat& q)
{
	Quat ret;
	ret.x = val*q.x;
	ret.y = val*q.y;
	ret.z = val*q.z;
	ret.w = val*q.w;
	
	return ret;
	
}

Quat operator+(const Quat& q1, const Quat& q2)
{
	Quat ret;
	ret.x=q1.x+q2.x;
	ret.y=q1.y+q2.y;
	ret.z=q1.z+q2.z;
	ret.w=q1.w+q2.w;
	
	return ret;
}

/* Returns the quaternion which correspond to the given euler angles.*/
Quat Euler2Qt(float roll, float pitch, float yaw)
{
	//Quat qx(cos(roll/2),sin(roll/2),0.0f,0.0f);
	//Quat qy(cos(pitch/2),0.0f,sin(pitch/2),0.0f);
	//Quat qz(cos(yaw/2),0.0f,0.0f,sin(yaw/2));
	Quat qx(sin(roll/2),0.0f,0.0f,cos(roll/2));
	Quat qy(0.0f,sin(pitch/2),0.0f,cos(pitch/2));
	Quat qz(0.0f,0.0f,sin(yaw/2),cos(yaw/2));
	
	Quat q = Qt_Mul(qy,qx);
	q = Qt_Mul(qz,q);
	
	return q;

}

/* Returns the corresponding euler angles from a quaternion */
void Qt2Euler( const Quat& q, float* roll, float* pitch, float* yaw)
{
	*roll = atan2( 2*(q.w*q.x+q.y*q.z), 1.0f-2*(q.x*q.x+q.y*q.y) );
	*pitch = asin( 2*(q.w*q.y-q.z*q.x) );
	*yaw = atan2( 2*(q.w*q.z+q.x*q.y), 1.0f-(q.y*q.y+q.z*q.z) );
}

/* Returns the corresponding SO(3) matrix from Euler angles*/
Mat<float> Euler2Rot( const float& roll, const float& pitch, const float& yaw)
{
	Mat<float> rx(0.0f, 3,3);
	rx.set( cos(roll), 2,2);
	rx.set( cos(roll), 3,3);
	rx.set( -sin(roll), 2,3);
	rx.set( sin(roll), 3,2);
	rx.set( 1.0f, 1,1);
	
	Mat<float> ry(0.0f, 3,3);
	ry.set( cos(pitch), 1,1);
	ry.set( cos(pitch), 3,3);
	ry.set( -sin(pitch), 3,1);
	ry.set( sin(pitch), 1,3);
	ry.set( 1.0f, 2,2);
	
	Mat<float> rz(0.0f, 3,3);
	rz.set( cos(yaw), 1,1);
	rz.set( cos(yaw), 2,2);
	rz.set( -sin(yaw), 1,2);
	rz.set( sin(yaw), 2,1);
	rz.set( 1.0f, 3,3);
	
	return rz*(ry*rx);
}

/* Returns the corresponding roll pitch yaw euler angles from SO(3) matrix.*/
void Rot2Euler( const Mat<float>& R, Mat<float>& angles)
{
	float epsilon = (float)1e-3;
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
		if( (R31 < epsilon && R31 > 0.0f-epsilon) || ( R31 > 0.0f-epsilon && R31 < epsilon ) )
			R31 = 0.0f;
			
		float R32 = R.get(3,2);
		if( (R32 < epsilon && R32 > 0.0f-epsilon) || ( R32 > 0.0f-epsilon && R32 < epsilon ) )
			R32 = 0.0f;
		float R33 = R.get(3,3);
		if( (R33 < epsilon && R33 > 0.0f-epsilon) || ( R33 > 0.0f-epsilon && R33 < epsilon ) )
			R33 = 0.0f;
		float R21 = R.get(2,1);
		if( (R21 < epsilon && R21 > 0.0f-epsilon) || ( R21 > 0.0f-epsilon && R21 < epsilon ) )
			R21 = 0.0f;
		float R11 = R.get(1,1);
		if( (R11 < epsilon && R11 > 0.0f-epsilon) || ( R11 > 0.0f-epsilon && R11 < epsilon ) )
			R11 = 0.0f;
			
			
		angles.set( -asin(R31), 2,1);	//pitch = -asin(R31)		
		
		float cTheta1 = cos( angles.get(2,1) );
		
		angles.set( atan21( R32/cTheta1, R33/cTheta1 ), 1,1);	//roll = atan2( R32/cos(pitch), R33/cos(pitch) )
		angles.set( atan21( R21/cTheta1, R11/cTheta1 ), 3,1);	//yaw = atan2( R21/cos(pitch), R11/cos(pitch) )
		
	}
}


/* Construct rotation matrix from (possibly non-unit) quaternion.
* Assumes matrix is used to multiply column vector on the left:
* vnew = mat vold. 3orks correctly for right-handed coordinate system
* and right-handed rotations. */
void Qt_ToMatrix(Quat q, HMatrix mat)
{
	float Nq = Qt_Norm(q);
	float s = (Nq > 0.0) ? (2.0 / Nq) : 0.0;
	float xs = q.x*s,
	ys = q.y*s,
	zs = q.z*s;
	float wx = q.w*xs,
	wy = q.w*ys,
	wz = q.w*zs;
	float xx = q.x*xs,
	xy = q.x*ys,
	xz = q.x*zs;
	float yy = q.y*ys,
	yz = q.y*zs,
	zz = q.z*zs;
	
	/*
	mat[X][X] = 1.0 - (yy + zz); mat[Y][X] = xy + wz; mat[2][X] = xz - wy;
	mat[X][Y] = xy - wz; mat[Y][Y] = 1.0 - (xx + zz); mat[2][Y] = yz + wx;
	mat[X][2] = xz + wy; mat[Y][2] = yz - wx; mat[2][2] = 1.0 - (xx + yy);
	mat[X][3] = mat[Y][3] = mat[2][3] = 0.0;
	mat[3][X] = mat[3][Y] = mat[3][2] = 0.0;
	mat[3][3] = 1.0;
	*/
	mat[0][0] = 1.0 - (yy + zz); mat[1][0] = xy + wz; mat[2][0] = xz - wy;
	mat[0][1] = xy - wz; mat[1][1] = 1.0 - (xx + zz); mat[2][1] = yz + wx;
	mat[0][2] = xz + wy; mat[1][2] = yz - wx; mat[2][2] = 1.0 - (xx + yy);
	mat[0][3] = mat[1][3] = mat[2][3] = 0.0;
	mat[3][0] = mat[3][1] = mat[3][2] = 0.0;
	mat[3][3] = 1.0;
}




/* Construct a unit quaternion from rotation matrix. Assumes matrix is
* used to multiply column vector on the left: vnew = mat vold. Works
* correctly for right-handed coordinate system and right-handed rotations.
* Translation and perspective components ignored. */
Quat Qt_FromMatrix(const HMatrix& mat)
{
	/* This algorithm avoids near-zero divides by looking for a large component
	* — first w, then x, y, or z. When the trace is greater than zero,
	* |w| is greater than 1/2, which is as small as a largest component can be.
	* Otherwise, the largest diagonal entry corresponds to the largest of |x|,
	* |y|, or |z|, one of which must be larger than |w|, and at least 1/2. */
	Quat qu;
	float tr, s;
	tr = mat[0][0] + mat[1][1]+ mat[2][2];
	
	if (tr >= 0.0) 
	{
		s = sqrt(tr + mat[3][3]);
		qu.w = s*0.5;
		s = 0.5 / s;
		qu.x = (mat[2][1] - mat[1][2]) * s;
		qu.y = (mat[0][2] - mat[2][0]) * s;
		qu.z = (mat[1][0] - mat[0][1]) * s;
	} 
	else 
	{
		int h = 0;
		if (mat[1][1] > mat[0][0]) h = 1;
		if (mat[2][2] > mat[h][h]) h = 2;
		
		switch (h) 
		{
		#define caseMacro(i,j,k,I,J,K) \
		case I:\
		s = sqrt( (mat[I][I] - (mat[J][J]+mat[K][K])) + mat[3][3] );\
		qu.i = s*0.5;\
		s = 0.5 / s;\
		qu.j = (mat[I][J] + mat[J][I]) * s;\
		qu.k = (mat[K][I] + mat[I][K]) * s;\
		qu.w = (mat[K][J] - mat[J][K]) * s;\
		break
		caseMacro(x,y,z,0,1,2);
		caseMacro(y,z,x,1,2,0);
		caseMacro(z,x,y,2,0,1);
		#undef caseMacro
		}
	}
	
	if (mat[3][3] != 1.0) 
	{
		s = 1.0/sqrt(mat[3][3]);
		qu.w *= s;
		qu.x *= s;
		qu.y *= s;
		qu.z *= s;
	}
	
	return (qu);
}


