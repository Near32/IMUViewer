#include "Obstacle.h"

Obstacle::Obstacle() : IElementFixe()
{
	
}

Obstacle::Obstacle(std::string name_, std::unique_ptr<se3> pose_) : IElementFixe(name_,std::move(pose_))
{
	
}

Obstacle::Obstacle(std::string name_, std::unique_ptr<se3> pose_, const Mat<float>& hwd_) : IElementFixe(name_,std::move(pose_), hwd_)
{
	
}
	
Obstacle::~Obstacle()
{

}
	
bool Obstacle::isObstacle()
{
	return true;
}
