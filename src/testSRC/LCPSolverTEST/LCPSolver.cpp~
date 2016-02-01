#include "LCPSolver.h"

LCPSolver::LCPSolver(int N_, const Mat<float>& A_, const Mat<float>& b_, const Mat<float>& initX) : N(N_), A(A_), b(b_), x(initX)
{
	precision = 1e-20f;
	gamma = 1.0f;
}

LCPSolver::~LCPSolver()
{

}

Mat<float> LCPSolver::solve(const int N_)
{
	if(N_ > 0)
		N = N_;
	
	if(A.getLine() != b.getLine() || A.getColumn() != x.getLine())
		return Mat<float>(0.0f,1,1);
		
	Mat<float> r(A.getLine(),1);
	int nbrLine = A.getLine();
	
	//assert nonzero values on the diagonal :
	for(int k=nbrLine;k--;)
	{
		if( fabs(A.get(k+1,k+1)) < numeric_limits<float>::epsilon())
			A.set( precision, k+1, k+1);
	}
	
	for(int k = N;k--;)
	{
		
		for(int i=nbrLine;i--;)
		{
			float ri = (Line(A,i+1)*x).get(1,1)+b.get(i+1,1);			
			float Aii = A.get(i+1,i+1);
			ri = x.get(i+1,1)-gamma*ri/Aii;
			
			if( ri < 0.0f)
				ri = 0.0f; 
			
			x.set( ri, i+1,1);
		}
	}
	
	return x;
}

Mat<float> LCPSolver::getA()	const
{
	return A;
}
Mat<float> LCPSolver::getb()	const
{
	return b;
}
Mat<float> LCPSolver::getX()	const
{
	return x;
}
float LCPSolver::getGamma()	const
{
	return gamma;
}

void LCPSolver::setA(const Mat<float>& A_)
{
	A = A_;
}
void LCPSolver::setb(const Mat<float>& b_)
{
	b=b_;
}
void LCPSolver::setGamma(const float gamma_)
{
	gamma = gamma_;
}
void LCPSolver::setPrecision(const float prec)
{
	precision = fabs(prec);
}



