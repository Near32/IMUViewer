#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <time.h>
#include "../../utils/SparseMat/Matrix.h"

using namespace std;

int init(Matrix<long double> &A)
{
    ifstream fin("matin.txt");
    int n,length,i,j;
    if(fin.is_open())
    {
        fin >> n;
	fin >> length;
        int k=0;
        while(k<length){ fin >> i >> j; fin >> A(i-1,j-1); k++; }
        fin.close();
    }
    else return 0;
    return n;
}

int main()
{

    Matrix<long double> A(1806);
    vector<long double> x(1806,1), y;
    long double sum;
    int n,k=0;
    clock_t t0,tf;
    cout.precision(17);

	srand(time(NULL));
    //n=init(A);
    n = 1806;
    t0=clock();
    for(int i=1;i<n;i++)
	{
		for(int j=1;j<n;j++)
		{
			if( (rand()%10) == 0 )
			{
				A(i,j) = (long double)(rand()%10);
				//A.set(i,j,(float)(rand()%10));
			}
			
			/*if( (rand()%10) == 0 )
			{
				//B(i,j) = (float)(rand()%10);
				B.set(i,j, (float)(rand()%10) );
			}*/
		}
	}
    
    tf=clock()-t0;
    cout << "\n\n" << (double)tf/((double)(CLOCKS_PER_SEC)) << " secondes" << std::endl;
    
    
    //timed multiplication
    t0=clock();
    
    for(int i=0;i<10;i++){ y=A*x; }
    
    tf=clock()-t0;
    
    //output last element
    cout << y[1805];
    //output time to multiply
    cout << "\n\n" << (double)tf/((double)(CLOCKS_PER_SEC)) << " secondes" << std::endl;

    //do naive multiply for last row to check last element of y
    /*long double tmp=0;
    for(int i=0;i<1806;i++) tmp+=A(1805,i);
    cout << "\n" << tmp <<"\n";
    */

    return 0;
}
