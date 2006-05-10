#ifndef _DecompTaylor_h_
#define _DecompTaylor_h_
// DecompTaylor.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include "suAnalysesDLL.h"
#include "Contact.h"
#include "Actuation.h"
#include "Decomp.h"
#include "ActuatorPerturbation.h"


//=============================================================================
//=============================================================================
/**
 * A class for computing induced reaction forces using the
 * "perturbed integration" methodology.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class SUANALYSES_API DecompTaylor : public Decomp
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Flag to indicate that contact has just been established. */
	bool *_contactJustEstablished;
	/** Induced contact point velocities. */
	double *_v;
	/** Induced contact point accelerations. */
	double *_a;
	/** Component forces due to position changes. */
	double *_fp;
	/** Component forces due to velocity changes. */
	double *_fv;
	/** Array for holding history of acceleration times. */
	double *_at;
	/** Array for holding acceleration histories. */
	double *_ah;
	/** Internal work array for holding the states. */
	double *_yTmp;
	/** Internal work array for holding the controls. */
	double *_xTmp;
	/** Internal work array for the generalized coordinates of the
	previous time step. */
	double *_qPrev;
	/** Internal work array for the generalized speeds of the
	previous time step. */
	double *_uPrev;
	/** Internal work array for the generalized coordinates. */
	double *_q;
	/** Internal work array for the generalized speeds. */
	double *_u;
	/** Internal work array for the time derivative of the generalized
	coordinates. */
	double *_dqdt;
	/** Internal work array for the time derivative of the generalized
	speeds. */
	double *_dudt;
	/** Internal work array for the 2nd time derivative of the generalized
	speeds. */
	double *_ddudt;
	/** Internal work array for the 3rd time derivative of the generalized
	speeds. */
	double *_dddudt;
	/** Internal work array for the change in generalized speeds. */
	double *_du;
	/** Current contact points. */
	double *_pa;
	/** Contact points prior to fricitonal sliding. */
	double *_paPrev;
	/** Current contact velocities. */
	double *_svel;
	/** Current contact forces. */
	double *_sfrc;
	/** Current contact forces due to elastic terms. */
	double *_pfrc;
	/** Frictional change in conatct force due to sliding of spring
	setpoints. */
	double *_dfpFric;
	/** Frictional change in conatct force against damping forces. */
	double *_dfvFric;
	/** Array used to adjust for changes in elastic contact forces due to
	sliding of spring setpoints. */
	double *_pAlpha;
	/** Array used to adjust for changes in viscous contact forces due to
	enforcement of friction constraints. */
	double *_vAlpha;
	/** Translational Jacobian. */
	double *_J;
	/** Previous translational Jacobian. */
	double *_JPrev;
	/** Difference between previous and curent Jacobians. */
	double *_JDiff;

//=============================================================================
// METHODS
//=============================================================================
public:
	DecompTaylor(Model *aModel);
	virtual ~DecompTaylor();
private:
	void setNull();
	void constructManager();
	void constructContactAnalysis();
	void constructDescription();
	void updateStorageDescriptions();
	void allocate();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// DECOMPOSITION
	//--------------------------------------------------------------------------
	virtual void
		compute(double *aXPrev,double *aYPrev,
		int aStep,double aDT,double aT,double *aX,double *aY);
	virtual void
		applyActionForce(int aC,double aT,double *aX,double *aY);
	virtual void
		initializeForceElements();
	virtual void
		initializeVelocityElements();
	virtual void
		computeFrictionFactors(double aT,double *aX,double *aY);
	virtual void
		computeHigherOrderDerivatives(int c,double t,double *dudt,
		double *ddudt,double *dddudt);
	virtual void
		computeContactVelocities(double *rVels);
	virtual void
		computeContactForces(double *rFrcs);

	//--------------------------------------------------------------------------
	// CALLBACKS
	//--------------------------------------------------------------------------
	virtual int
		begin(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData=NULL);

//=============================================================================
};	// END of class DecompTaylor

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __DecompTaylor_h__
