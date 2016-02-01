#ifndef ICONSTRAINTSOLVERSTRATEGY
#define ICONSTRAINTSOLVERSTRATEGY


#include "../RigidBody.h"
#include "../IConstraint.h"
#include "../../utils/math.h"


class Simulation;

class IConstraintSolverStrategy
{

	protected :
	
	Mat<float> C;
	Mat<float> constraintsJacobian;
	Mat<float> constraintsImpulse;
	Mat<float> offset;
	Mat<float> lambda;
	Mat<float> M;
	
	float dt;
	
	Simulation* sim;
	
	public :
	
	IConstraintSolverStrategy(Simulation* sim);
	
	~IConstraintSolverStrategy();
	
	
	virtual void Solve(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext) = 0;

	//------------------------------------
	
	Mat<float> getConstraints()	const;
	
	Mat<float> getConstraintsJacobian() const;
	
	Mat<float> getLambda()	const;
	
	
		
	
};




//-----------------------------------------
//-----------------------------------------
//-----------------------------------------
//-----------------------------------------



class SimultaneousImpulseBasedConstraintSolverStrategy : public IConstraintSolverStrategy
{
	private :
	
	//std::vector<std::unique_ptr<IConstraint> > c;
	
	
	//-----------------------------
	
	public :
	
	SimultaneousImpulseBasedConstraintSolverStrategy(Simulation* sim);
	
	~SimultaneousImpulseBasedConstraintSolverStrategy();
	
	
	void computeConstraintsJacobian(std::vector<std::unique_ptr<IConstraint> >& c);
	void computeConstraintsJacobian(std::vector<std::unique_ptr<IConstraint> >& c, const Mat<float>& q, const Mat<float>& qdot);
	void computeConstraintsANDJacobian(std::vector<std::unique_ptr<IConstraint> >& c, const Mat<float>& q, const Mat<float>& qdot);
		
	virtual void Solve(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext ) override;

};


#endif
