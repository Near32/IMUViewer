cmake_minimum_required (VERSION 2.6)
project (Gunvarrel)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g")

#########################################################
# FIND SDL
#########################################################
find_package(SDL REQUIRED)
include_directories(${SDL_INCLUDE_DIRS})
link_directories(${SDL_LIBRARY_DIRS})
add_definitions(${SDL_DEFINITIONS})
if(NOT SDL_FOUND)
    message(ERROR " SDL not found!")
endif(NOT SDL_FOUND)
#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)
#########################################################
# Include Files
#########################################################
#add_executable(Gunvarrel src/mainENGINE.cpp src/EtatEngine.cpp src/IEngine.cpp src/VueEngine.cpp src/Game.cpp src/Environnement.cpp src/IElement.cpp src/TrackBallCamera.cpp  src/utils/math.cpp src/utils/Quaternion/Quaternion.cpp src/common/objloader.cpp)
#add_executable(Gunvarrel src/mainENGINE.cpp src/EtatEngine.cpp src/IEngine.cpp src/VueEngine.cpp src/Game.cpp src/Environnement.cpp src/IElement.cpp src/IElementFixe.cpp src/Obstacle.cpp src/OrbeBonus.cpp src/IFabricant.cpp src/Fabriques.cpp src/TrackBallCamera.cpp  src/utils/math.cpp src/utils/Quaternion/Quaternion.cpp src/common/objloader.cpp )
add_executable(Gunvarrel src/mainENGINE.cpp src/EtatEngine.cpp src/IEngine.cpp src/VueEngine.cpp src/ControllerEngine.cpp src/Game.cpp src/Environnement.cpp src/IElement.cpp src/IElementFixe.cpp src/Obstacle.cpp src/OrbeBonus.cpp src/IFabricant.cpp src/Fabriques.cpp src/IElementMobile.cpp src/ElementMobile.cpp src/TrackBallCamera.cpp  src/utils/math.cpp src/utils/Quaternion/Quaternion.cpp src/utils/MVG/MVG.cpp src/common/objloader.cpp src/physics/IConstraint.cpp src/physics/IForceEffect.cpp src/physics/IMoveable.cpp src/physics/IShape.cpp src/physics/ISimulationObject.cpp src/physics/RigidBody.cpp src/physics/Simulation.cpp src/physics/eval/IBroadPhaseStrategy.cpp src/physics/eval/CollisionDetector.cpp src/physics/eval/IUpdater.cpp src/physics/eval/IMidNarrowPhaseStrategy.cpp src/physics/eval/IIntegrator.cpp src/physics/eval/IContactGenerator.cpp src/physics/eval/IConstraintSolverStrategy.cpp src/physics/eval/Contact.cpp src/physics/eval/HelperFunctions.cpp src/utils/LCPSolver/LCPSolver.cpp )

########################################################
# Linking & stuff
#########################################################

# create the program "Gunvarrel"
target_link_libraries(Gunvarrel ${OPENGL_LIBRARIES} ${SDL_LIBRARY} )

