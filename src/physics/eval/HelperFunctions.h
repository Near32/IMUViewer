#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include "../RigidBody.h"

Mat<float> closestPointLOfBOXGivenPointL(RigidBody& rb, const Mat<float>& pointL);
Mat<float> closestPointWOfBOXGivenPointL(RigidBody& rb, const Mat<float>& pointL);
Mat<float> closestPointWOfBOXGivenPointW(RigidBody& rb, const Mat<float>& pointW);
bool testOBBPlane( RigidBody& box, RigidBody& plane);

Mat<float> testOBBOBB( RigidBody& b1, RigidBody& b2, bool& intersect);
bool equals(const Mat<float>& a, const Mat<float>& b, float precision);

void innerVoronoiProjection(RigidBody& rb, Mat<float>& pointL);
void innerVoronoiProjectionANDNormal(RigidBody& rb, Mat<float>& pointL, Mat<float>& normalL);



#endif
