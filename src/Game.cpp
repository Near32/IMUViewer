#include "Game.h"

#define debug

#define threadUse

std::mutex ressourcesMutex;

Game::Game() : gameON(true), gameState( MENUINIT ), ptrEtat( new EtatEngine(this, MENUINIT) ), ptrVue( new VueEngine(this, MENUINIT) )
{
	nbrCurrentCommandsHandled = 0;
	init();
}

Game::~Game()
{
	delete ptrEtat;
	delete ptrVue;
	delete ptrController;
}
	
	
void Game::loop()
{
/*
	Uint32 last_time = SDL_GetTicks();
    Uint32 current_time,ellapsed_time;
    Uint32 start_time;
    SDL_Event event;
    
    float angleX = 0.0f;
    float angleZ = 0.0f;
    
    */
    //--------------------------------------------
    //--------------------------------------------
    //--------------------------------------------
    //--------------------------------------------
    
    //Mise en place des threads :
    
    
    thread tEtat( &EtatEngine::loop, std::ref(*ptrEtat) );
    thread tController( &ControllerEngine::loop, std::ref(*ptrController) );
    thread tVue( &VueEngine::loop, std::ref(*ptrVue) );
    
    
    if(tEtat.joinable())
    	tEtat.join();
    if( tController.joinable())
    	tController.join();
    if( tVue.joinable())
    	tVue.join();
    	

    
    /*
	while(gameON)
	{
		start_time = SDL_GetTicks();
		
		//traitement/récupération des events.
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
                	
                	case SDLK_SPACE:
                	{
#ifdef debug
std::cout << "GAME : add command : TCSimulateStride : DEBUG." << std::endl;
#endif                	
                	ptrEtat->addCommandToHandle( (const ICommand&)SimulateStrideCommand() );
                	}
                	break;
                	
                	default:
                	{
                	//camera->onKeyboard(event.key);
                	ptrVue->addCommandToHandle( (const ICommand&)CameraOnKeyboardCommand(event.key));
                	}
                	break;
                }
                break;
                
                case SDL_MOUSEMOTION:
                {
                //camera->onMouseMotion(event.motion);
                ptrVue->addCommandToHandle( (const ICommand&)CameraOnMouseMotionCommand(event.motion) );
                }
                break;
                
                case SDL_MOUSEBUTTONUP:
                case SDL_MOUSEBUTTONDOWN:
                {
                //camera->onMouseButton(event.button);
                ptrVue->addCommandToHandle( (const ICommand&)CameraOnMouseButtonCommand(event.button) );
                }
                break;
            }
        }

		
        //--------------------------------------
        //--------------------------------------
        //--------------------------------------
        
        current_time = SDL_GetTicks();
        ellapsed_time = current_time - last_time;
        last_time = current_time;
        
        
        //--------------------------------------
        //--------------------------------------
        //--------------------------------------
        
        
        //MAJ DES ENGINES :
        
        angleZ += 0.05 * ellapsed_time;
        angleX += 0.05 * ellapsed_time;
#ifndef threadUse        
        ptrVue->Dessiner(angleX,angleZ);
#endif        
        
        //--------------------------------------
        //--------------------------------------
        //--------------------------------------
                        
        ellapsed_time = SDL_GetTicks() - start_time;
        if (ellapsed_time < 20)
        {
            SDL_Delay(20 - ellapsed_time);
        }
		
	}*/
	
	
}

void Game::init()
{
   ptrController = new ControllerEngine(this,ptrEtat,ptrVue, MENUINIT); 
}


void Game::incrementNbrHandledCommands()
{
	nbrCurrentCommandsHandled++;
}


