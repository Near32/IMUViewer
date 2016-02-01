#ifndef TRACKBALLCAMERA_H
#define TRACKBALLCAMERA_H

#include <SDL/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <iostream>


class TrackBallCamera
{
	private :
	double motionSensitivity;
	double scrollSensitivity;
	bool hold;
	
	double distance;
	double angleY;
	double angleZ;
	
	SDL_Cursor* hand1;
	SDL_Cursor* hand2;
	
	public :
	
	TrackBallCamera();
	
	~TrackBallCamera();
	
	void onMouseMotion( const SDL_MouseMotionEvent& mmevent);
	void onMouseButton( const SDL_MouseButtonEvent& mbevent);
//	void onMouseWheel( const SDL_MouseWheelEvent& mwevent);
	void onKeyboard(const SDL_KeyboardEvent& kevent);
	
	void look();
	
	void setMotionSensitivity( double sensitivity);
	void setScrollSensitivity( double sensitivity);
	
};

#endif
