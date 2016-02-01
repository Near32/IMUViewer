#ifndef VUEENGINE_H
#define VUEENGINE_H

#include "IEngine.h"

#include <SDL/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <cstdlib>
#include "TrackBallCamera.h"
#include "common/objloader.h"
#include "utils/math.h"
#include <thread>
#include <mutex>

#include <X11/Xlib.h>

class VueEngine : public IEngine
{
	public :
	
	SDL_Surface* ecran;
	TrackBallCamera* camera;
	
	std::map< std::string, std::vector<glm::vec3> > containerV;
	std::map< std::string, std::vector<glm::vec2> > containerUV;
	std::map< std::string, std::vector<glm::vec3> > containerN;
	
	
	VueEngine(Game* game_, GameState gameState_);
	~VueEngine();
	
	void loop() override ;
	void init();
	
	void Dessiner(float angleX, float angleZ);
	
	void drawGunvarrel();
	void drawElement(const std::string& path);
	void drawElement(const std::vector<glm::vec3>& v, const std::vector<glm::vec2>& uv, const std::vector<glm::vec3>& n);
	void loadElement( const std::string& path);
};

#endif
