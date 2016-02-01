#ifndef LCPSOLVER_H
#define LCPSOLVER_H

#include "../../utils/Mat/Mat.h"

class LCPSolver
{
	private :
	Mat<float> A;
	Mat<float> b;
	Mat<float> x;
	int N;
	float gamma;
	
	//-----------
	float precision;
	
	public :
	
	LCPSolver(int N_, const Mat<float>& A_, const Mat<float>& b_, const Mat<float>& initX);
	~LCPSolver();
	
	virtual Mat<float> solve(const int N_=0);
	
	Mat<float> getA()	const;
	Mat<float> getb()	const;
	Mat<float> getX()	const;
	float getGamma()	const;
	
	void setA(const Mat<float>& A_);
	void setb(const Mat<float>& b_);
	void setGamma(const float gamma_);
	
	
	void setPrecision(const float prec);
	
};

#endif
