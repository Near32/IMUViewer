#include <iostream>
#include "LCPSolver.h"

int main(int argc, char* argv[])
{
	
	Mat<float> A( 10,10, (char)1);
	A=absM(transpose(A)*A);
	Mat<float> x(0.0f,10,1);
	Mat<float> b( 10,1, (char)1);
	
	int N = 10;
	
	LCPSolver instanceLCPSolver(N,A,b,x);
	clock_t time = clock();
	x = instanceLCPSolver.solve();
	std::cout << " La resolution a prise : " << (float)(clock()-time)/CLOCKS_PER_SEC << std::endl;
	
	A = instanceLCPSolver.getA();
	b.afficher();
	x.afficher();
	
	Mat<float> y(A*x+b);
	y.afficher();
	
	A.afficher();
	
	(transpose(x)*y).afficher();
	
	return 0;
}
