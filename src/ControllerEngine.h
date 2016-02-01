#ifndef CONTROLLERENGINE_H
#define CONTROLLERENGINE_H

#include "IEngine.h"

#include <SDL/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "EtatEngine.h"
#include "VueEngine.h"

#include <mutex>

class Game;

class ControllerEngine : public IEngine
{
	public :
	
	ControllerEngine(Game* game_, EtatEngine* ptrEtat_, VueEngine* ptrVue_, const GameState& gameState);
	~ControllerEngine();
	
	void loop() override;
	void init();
	
	private :
	
	EtatEngine* ptrEtat;
	VueEngine* ptrVue;
	
};

#endif
