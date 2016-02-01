#ifndef IENGINE_H
#define IENGINE_H

#include <iostream>
#include <memory>
#include <vector>
#include "GameState.h"
#include "ICommand.h"

class Game;

class IEngine
{
	public :
	
	Game* game;
	GameState gameState;
	std::vector<std::unique_ptr<ICommand> > commandsToHandle;
	
	//-----------------------------------------------------------
	//-----------------------------------------------------------
	
	IEngine();
	IEngine(Game* game_, GameState gameState_);
	virtual ~IEngine();
	
	
	//---------------------------------------
	
	virtual void loop()=0;
	
	//---------------------------------------
	
	void addCommandToHandle(const ICommand* command);
};

#endif
