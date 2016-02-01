#ifndef GAME_H
#define GAME_H

#include <iostream>

#include "GameState.h"
#include "EtatEngine.h"
#include "VueEngine.h"
#include "ControllerEngine.h"
#include "ICommand.h"

#include <thread>
#include <mutex>

class Game
{
	public :
	
	EtatEngine* ptrEtat;
	VueEngine* ptrVue;
	ControllerEngine* ptrController;
	
	GameState gameState;
	bool gameON;
	
	//TrackBallCamera* camera;
	
	std::vector<ICommand> currentCommands;
	std::vector<ICommand> waitingCommands;
	int nbrCurrentCommandsHandled;
	
	//---------------------------------------
	//---------------------------------------
	
	Game();
	~Game();
	
	
	void loop();	
	
	void init();
	
	
	void incrementNbrHandledCommands();
	
};

#endif
