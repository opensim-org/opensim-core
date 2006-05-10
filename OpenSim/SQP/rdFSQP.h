#ifndef _rdFSQP_h_
#define _rdFSQP_h_
// rdFSQP.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	Author:  Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//=============================================================================
// INCLUDES
//=============================================================================
#include <ostream>
#include "rdSQPDLL.h"
#include "rdOptimizationTarget.h"


//=============================================================================
//=============================================================================
/**
 * This class provides methods for finding the optimal controls of a redundant
 * system by applying sequential quadratic programming techniques.  The core
 * algorithm is called fsqp, "Fast Sequential Quadratic Programming".
 */
namespace OpenSim { 

class RDSQP_API rdFSQP
{
//=============================================================================
// DATA
//=============================================================================
private:

	// ** Optimization target. */
	rdOptimizationTarget *_target;

	// PARAMETETERS
	/** Mode. */
	int _mode;
	/** Print level. */
	int _printLevel;
	/** Maximum number of iterations. */
	int _maxIter;
	/** Variable that contains information about the status of an
	optimization.*/
	int _inform;
	/** Value used for infinity or a very large number. */
	double _infinity;
	/** Convergence criterion. */
	double _eps;
	/** Convergence criterion for nonlinear equality constraints. */
	double _epseqn;
	/** I don't know. */
	double _udelta;

	// SEQUENTIAL INFO
	int _ncsrl;
	int _ncsrn;
	int _nfsr;
	int *_mesh;

	// ALLOCATIONS
	/** Lower bounds on controls. */
	double *_bl;
	/** Upper bounds on controls. */
	double *_bu;
	/** Controls. */
	double *_x;
	/** Array of performance criteria. */
	double *_p;
	/** Array of constraints. */
	double *_c;
	/** Lagrange multipliers for constraints. */
	double *_lambda;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	virtual ~rdFSQP();
	//static void Delete(void *aPtr);
	rdFSQP(rdOptimizationTarget *aTarget);
private:
	void setNull();

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
public:
	rdOptimizationTarget* setTarget(rdOptimizationTarget *aTarget);
	rdOptimizationTarget* getTarget();
	void setMode(int aMode);
	int getMode();
	void setPrintLevel(int aLevel);
	int getPrintLevel();
	void setMaxIterations(int aMaxIter);
	int getMaxIterations();
	void setInfinity(double aInfinity);
	double getInfinity();
	void setConvergenceCriterion(double aEPS);
	double getConvergenceCriterion();
	void setNonlinearEqualityConstraintTolerance(double aEPSEQN);
	double getNonlinearEqualityConstraintTolerance();
	void setLowerBound(double aLower);
	void setLowerBound(int aIndex,double aLower);
	void setLowerBound(double aLower[]);
	double* getLowerBound();
	void setUpperBound(double aUpper);
	void setUpperBound(int aIndex,double aUpper);
	void setUpperBound(double aUpper[]);
	double* getUpperBound();

	//--------------------------------------------------------------------------
	// OPTIMAL CONTROLS
	//--------------------------------------------------------------------------
	int computeOptimalControls(const double *xstart,double *x);

	//--------------------------------------------------------------------------
	// DERIVATIVES
	//--------------------------------------------------------------------------
	static int
		CentralDifferences(rdOptimizationTarget *aTarget,
		double *dx,double *x,double *dpdx,double *dcdx);
	static int
		CentralDifferences(rdOptimizationTarget *aTarget,
		double *dx,double *x,double *dpdx);
	static int
		CentralDifferencesConstraint(rdOptimizationTarget *aTarget,
		double *dx,double *x,int ic,double *dcdx);

	//--------------------------------------------------------------------------
	// STATIC FUNCTIONS USED AS INPUT TO cfsqp()
	//--------------------------------------------------------------------------
	static void
		pFunc(int nparam,int j,double *x,double *pj,void *cd);
	static void
		cFunc(int nparam,int j,double *x,double *cj,void *cd);
	static void
		dpdxFunc(int nparam,int j,double *x,double *dpdx,
		void (*dummy)(int,int,double *,double *,void *),void *cd);
	static void
		dcdxFunc(int nparam,int j,double *x,double *dcdx,
		void (*dummy)(int,int,double *,double *,void *),void *cd);

	//--------------------------------------------------------------------------
	// PRINT
	//--------------------------------------------------------------------------
	static void
		PrintInform(int aInform,std::ostream &aOStream);

//=============================================================================
};	// END class rdFSQP

}; //namespace
//=============================================================================
//=============================================================================

#endif // __rdFSQP_h__
