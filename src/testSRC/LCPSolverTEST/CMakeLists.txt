cmake_minimum_required (VERSION 2.6)
project (LCPSolverTEST)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./bin)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")



#########################################################
# Include Files
#########################################################
#add_executable(Gunvarrel src/mainENGINE.cpp src/EtatEngine.cpp src/IEngine.cpp src/VueEngine.cpp src/Game.cpp src/Environnement.cpp src/IElement.cpp src/TrackBallCamera.cpp  src/utils/math.cpp src/utils/Quaternion/Quaternion.cpp src/common/objloader.cpp)
add_executable(outputLCPSolver testLCPSolver.cpp LCPSolver.cpp )#../../utils/math.cpp) #../../utils/Quaternion/Quaternion.cpp ../../utils/MVG/MVG.cpp )

########################################################
# Linking & stuff
#########################################################

# create the program "Gunvarrel"
#target_link_libraries(Gunvarrel ${OPENGL_LIBRARIES} ${SDL_LIBRARY} )

