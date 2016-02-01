#include "../../utils/SparseMat/SparseMat.h"
#include "../../utils/Mat/Mat.h"
#include <time.h>

int main(void)
{
	int n = 100;
	int m = 100;
	SparseMat<float> A(n);
	SparseMat<float> B(n);
	SparseMat<float> AA;
	AA.operator=(A);
	Mat<float> mA(n,m,(char)1);
	Mat<float> mB(m,n,(char)1);
	clock_t t0,tf;
	
	//srand(time(NULL));
	t0 = clock();
	for(int i=1;i<=n;i++)
	{
		for(int j=1;j<=n;j++)
		{
			if( (rand()%5) == 0 )
			{
				//A(i,j) = (float)(rand()%10);
				A.set(i,j,(float)(rand()%10));
			}
			if( (rand()%5) == 0 )
			{
				//B(i,j) = (float)(rand()%10);
				B.set(i,j, (float)(rand()%10) );
			}
		}
	}
	
	tf=clock()-t0;
	std::cout << "\n\n" << (double)tf/((double)(CLOCKS_PER_SEC)) << std::endl;
	//A.afficher();
	//B.print();
	
	t0 = clock();
	//(A*B).print();
	//SparseMat<float> C =
	A*B;
	tf=clock()-t0;
	std::cout << "\n\n" << (double)tf/((double)(CLOCKS_PER_SEC)) << std::endl;
	//(A*B).afficher();
	
	t0 = clock();
	mA*mB;
	tf=clock()-t0;
	std::cout << "\n\n" << (double)tf/((double)(CLOCKS_PER_SEC)) << std::endl;
	
	t0 = clock();
	A*mA;
	tf=clock()-t0;
	std::cout << "\n\n" << (double)tf/((double)(CLOCKS_PER_SEC)) << std::endl;
	
}


int main1(void)
{
  std::map<char,int> mymap;
  char c;

  mymap ['a']=101;
  mymap ['c']=202;
  mymap ['f']=303;

  for (c='a'; c<'h'; c++)
  {
    std::cout << c;
    if (mymap.count(c)>0)
      std::cout << " is an element of mymap.\n";
    else 
      std::cout << " is not an element of mymap.\n";
  }

  return 0;
}

