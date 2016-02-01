#include <iostream>
#include <SDL/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <cstdlib>
#include "TrackBallCamera.h"


void Dessiner();
void stop();
void drawGunvarrel();

double angleZ = 0;

double angleX = 0;

TrackBallCamera* camera;


int main(int argc, char *argv[])

{

    SDL_Event event;
    SDL_Init(SDL_INIT_VIDEO);
    atexit(stop);
    SDL_WM_SetCaption("Gunvarrel", NULL);
    SDL_SetVideoMode(800, 600, 32, SDL_OPENGL);

	//--------------------
	//Camera 
	camera = new TrackBallCamera();
	
	//-------------------------
	
	glClearColor(1.0f,1.0f,1.0f,1.0f);
    glMatrixMode( GL_PROJECTION );

    glLoadIdentity();

    gluPerspective(45,(double)800/600,1,1000);


    glEnable(GL_DEPTH_TEST);


    Dessiner();


    Uint32 last_time = SDL_GetTicks();
    Uint32 current_time,ellapsed_time;
    Uint32 start_time;


    for (;;)
    {
        start_time = SDL_GetTicks();
        while (SDL_PollEvent(&event))
        {
            switch(event.type)
            {
                case SDL_QUIT:
                exit(0);
                break;
                
                case SDL_KEYDOWN:
                switch(event.key.keysym.sym)
                {
                	case SDLK_ESCAPE:
                	exit(0);
                	break;
                	
                	default:
                	camera->onKeyboard(event.key);
                	break;
                }
                break;
                
                case SDL_MOUSEMOTION:
                camera->onMouseMotion(event.motion);
                break;
                
                case SDL_MOUSEBUTTONUP:
                case SDL_MOUSEBUTTONDOWN:
                camera->onMouseButton(event.button);
                break;
            }
        }


        current_time = SDL_GetTicks();
        ellapsed_time = current_time - last_time;
        last_time = current_time;


        angleZ += 0.05 * ellapsed_time;
        angleX += 0.05 * ellapsed_time;


        Dessiner();
        
        
        ellapsed_time = SDL_GetTicks() - start_time;
        if (ellapsed_time < 20)
        {
            SDL_Delay(20 - ellapsed_time);
        }
        
    }

    return 0;
}


void Dessiner()
{
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity( );

	//---------------------------
	camera->look();
	//----------------------------
	
    //gluLookAt(3,4,2,0,0,0,0,0,1);

	drawGunvarrel();
	
	
	glBegin(GL_QUADS);
	glColor3ub(250,250,250);
    glVertex3d(-100,-100,-2);
    glVertex3d(100,-100,-2);
    glVertex3d(100,100,-2);
    glVertex3d(-100,100,-2);
	glEnd();
	
	
    glRotated(angleZ,0,0,1);
    glRotated(angleX,1,0,0);

    glBegin(GL_QUADS);
    glColor3ub(0,0,0); 
    glVertex3d(1,1,1);
    glVertex3d(1,1,-1);
    glVertex3d(-1,1,-1);
    glVertex3d(-1,1,1);

    glColor3ub(10,10,10); 
    glVertex3d(1,-1,1);
    glVertex3d(1,-1,-1);
    glVertex3d(1,1,-1);
    glVertex3d(1,1,1);

    glColor3ub(20,20,20); 
    glVertex3d(-1,-1,1);
    glVertex3d(-1,-1,-1);
    glVertex3d(1,-1,-1);
    glVertex3d(1,-1,1);

    glColor3ub(30,30,30);
    glVertex3d(-1,1,1);
    glVertex3d(-1,1,-1);
    glVertex3d(-1,-1,-1);
    glVertex3d(-1,-1,1);

    glColor3ub(40,40,40);
    glVertex3d(1,1,-1);
    glVertex3d(1,-1,-1);
    glVertex3d(-1,-1,-1);
    glVertex3d(-1,1,-1);

    glColor3ub(50,50,50);
    glVertex3d(1,-1,1);
    glVertex3d(1,1,1);
    glVertex3d(-1,1,1);
    glVertex3d(-1,-1,1);

    glEnd();

    glFlush();
    SDL_GL_SwapBuffers();
}


void stop()
{
	delete camera;
	SDL_Quit();
}

void drawGunvarrel()
{
	std::vector<glm::vec3> v;
	std::vector<glm::vec2> uv;
	std::vector<glm::vec3> n;
	bool res = loadOBJ("../res/gunvarrel_scaled.obj", v,uv,n);
	
	
	//glBegin(GL_TRIANGLES);
	glRotated(90,1,0,0);
	glTranslated(2,2.8,0);
	
	for(int i=0;i<v.size();i++)
	{
		if(i%3 == 0)
		{
			glBegin(GL_TRIANGLES);
			int color = (i)%255;
			glColor3ub(color,color,color);
		}
		
		glVertex3d( v[i].x, v[i].y, v[i].z);
		
		if( (i+1)%3 == 0)
		{
			glEnd();
		}
	}
	
	//glEnd();
	glTranslated(-2,-2.8,0);
	glRotated(-90,1,0,0);
	
	//------------------------
	//------------------------
}


