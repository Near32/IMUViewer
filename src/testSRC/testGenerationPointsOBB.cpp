#include "../utils/math.h"
#include "../physics/IShape.h"


int main(int argc, char* argv[])
{
	Mat<float> hwd((float)4,3,1);
	
	BoxShape box( NULL, hwd);
	
	
	Mat<float> pointsB1[8];
	for(int i=8;i--;)	pointsB1[i] = Mat<float>(0.0f,3,1);
	
	//register the position of the 8 points composing b1:
	Mat<float> min1((float)0,3,1);
	Mat<float> max1((float)0,3,1);
	
	min1.set( -box.getHeight()/2.0f, 1,1);
	min1.set( -box.getWidth()/2.0f, 2,1);
	min1.set( -box.getDepth()/2.0f, 3,1);
	max1.set( -min1.get(1,1), 1,1);
	max1.set( -min1.get(2,1), 2,1);
	max1.set( -min1.get(3,1), 3,1);
	
	int col = 0;
	
	for(int pm1=2;pm1--;)
	{
		for(int pm2=2;pm2--;)
		{
			for(int pm3=2;pm3--;)
			{
				pointsB1[col].set( pm1*( min1.get(1,1) ) + (1-pm1)*( max1.get(1,1) ), 1,1);
				pointsB1[col].set( pm2*( min1.get(2,1) ) + (1-pm2)*( max1.get(2,1) ), 2,1);
				pointsB1[col].set( pm3*( min1.get(3,1) ) + (1-pm3)*( max1.get(3,1) ), 3,1);
				
				col++;
			}
		}
	}
	
	
	for(int i=8;i--;)	pointsB1[i].afficher();
	
	
	return 0;
}
