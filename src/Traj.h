#ifndef TRAJ_H
#define TRAJ_H

#include <iostream>
#include <vector>
#include "utils/Mat/Mat.h"

#include <GL/gl.h>
#include <GL/glu.h>

class Traj
{
	private :
	std::vector<Mat<float> > pos;
	
	public :
	
	Traj()
	{
	
	}
	
	~Traj()
	{
	
	}
	
	void draw()
	{
		for(int i=1;i<pos.size();i++)
		{
			glBegin(GL_TRIANGLES);
			
			int color = 100;
			glColor3ub(color,color,color);
		
			glVertex3d( pos[i-1].get(1,1), pos[i-1].get(2,1), pos[i-1].get(3,1));
			glVertex3d( pos[i-1].get(1,1), pos[i-1].get(2,1), pos[i-1].get(3,1));
			glVertex3d( pos[i+1].get(1,1), pos[i+1].get(2,1), pos[i+1].get(3,1));
		
			glEnd();
		}
	}
	
	void add(const Mat<float>& pos)
	{
		this->pos.insert( this->pos.end(), pos);
	}
};

#endif
