#ifndef QUAT_H
#define QUAT_H

#include "../Mat/Mat.h"

class Quat
{	
	public : 
	
	float x;
	float y;
	float z;
	float w;
				
	Quat(float x_, float y_, float z_, float w_) : x(x_),y(y_),z(z_),w(w_) {}
	Quat() : x(0.0f),y(0.0f),z(0.0f),w(1.0f) {}
	
	~Quat()	{}
	
};


/* Return norm of quaternion, the sum of the squares of the components. */
#define Qt_Norm(q) ((q).x*(q).x + (q).y*(q).y + (q).z*(q).z + (q).w*(q).w)
Quat operator+(const Quat& q1, const Quat& q2);
Quat operator*(float val, const Quat& q);

typedef float HMatrix[4][4];
//#define X 0
//#define Y 1
//#define Z 2
//#define W 3


Quat Qt_Mul(Quat qL, Quat qR);
Quat Euler2Qt(float roll, float pitch, float yaw);
void Qt2Euler( const Quat& q, float* roll, float* pitch, float* yaw);
Mat<float> Euler2Rot( const float& roll, const float& pitch, const float& yaw);
void Rot2Euler( const Mat<float>& R, Mat<float>& angles);

void Qt_ToMatrix(Quat q, HMatrix mat);

template<typename T>
void Qt_ToMatrix(Quat q,Mat<T>* mat)
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
	
	*mat = Mat<T>((T)0,4,4);
	
	mat->set( (T)(1.0 - (yy + zz)), 1,1); 
	mat->set( (T)(xy + wz), 2,1);
	mat->set( (T)(xz - wy), 3,1);
	
	mat->set( (T)(xy - wz), 1,2); 
	mat->set( (T)(1.0 - (xx + zz)), 2,2); 
	mat->set( (T)(yz + wx), 3,2);
	
	mat->set( (T)(xz + wy), 1,3);
	mat->set( (T)(yz - wx), 2,3); 
	mat->set( (T)(1.0 - (xx + yy)), 3,3);
	
	/*mat[X][3] = mat[1][3] = mat[2][3] = 0.0;
	mat[3][X] = mat[3][1] = mat[3][2] = 0.0;*/
	mat->set( (T)1.0, 4,4);
}

Quat Qt_FromMatrix(const HMatrix& mat);

template<typename T>
Quat Qt_FromMat(const Mat<T>& mat)
{
	HMatrix m;
	for(int i=4;i--;)
	{
		for(int j=4;j--;)
			m[i][j] = mat.get(i+1,j+1);
	}
	
	return Qt_FromMatrix(m);
}

template<typename T>
Quat Qt_FromNVect(Mat<T> vect)
{
	Quat r;
	float theta = norme2(vect);
	vect = (1.0/theta)*vect;
	
	r.x = vect.get(1,1)*sin(theta/2);
	r.y = vect.get(2,1)*sin(theta/2);
	r.z = vect.get(3,1)*sin(theta/2);
	r.w = cos(theta/2);
	
	return r;	
}


template<typename T>
Quat Mat2Qt(Mat<T> vect)
{
	Quat r;
	r.x = (float)vect.get(1,1);
	r.y = (float)vect.get(2,1);
	r.z = (float)vect.get(3,1);
	r.w = (float)vect.get(4,1);
	
	return r;
}

template<typename T>
Mat<T> Qt2Mat(Quat q)
{
	Mat<T> r(4,1);
	r.set( (T)q.x, 1,1);
	r.set( (T)q.y, 2,1);
	r.set( (T)q.z, 3,1);
	r.set( (T)q.w, 4,1);
	return r;
}

template<typename T>
void Quat2SO3(Quat q,Mat<T>* mat)
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
	
	*mat = Mat<T>((T)0,3,3);
	
	mat->set( (T)(1.0 - (yy + zz)), 1,1); 
	mat->set( (T)(xy + wz), 2,1);
	mat->set( (T)(xz - wy), 3,1);
	
	mat->set( (T)(xy - wz), 1,2); 
	mat->set( (T)(1.0 - (xx + zz)), 2,2); 
	mat->set( (T)(yz + wx), 3,2);
	
	mat->set( (T)(xz + wy), 1,3);
	mat->set( (T)(yz - wx), 2,3); 
	mat->set( (T)(1.0 - (xx + yy)), 3,3);
	
}





#endif


