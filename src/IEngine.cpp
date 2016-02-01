#include "IEngine.h"

#include "Game.h"

IEngine::IEngine()
{
	gameState = UNDEFINED;
}

IEngine::IEngine(Game* game_, GameState gameState_) : game(game_), gameState(gameState_)
{

}

IEngine::~IEngine()
{

}

void IEngine::addCommandToHandle(const ICommand* command)
{
	commandsToHandle.insert( commandsToHandle.end(), std::unique_ptr<ICommand>( (ICommand*)command) );
}
