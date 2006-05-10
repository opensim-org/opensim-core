#ifndef _ActuatorPerturbationIndependent_h_
#define _ActuatorPerturbationIndependent_h_
// ActuatorPerturbationIndependent.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, Saryn R. Goldberg, May Q.Liu
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/DerivCallback.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include "suAnalysesDLL.h"
#include "Contact.h"
#include "Decomp.h"
#include "ActuatorPerturbation.h"


//=============================================================================
//=============================================================================
/**
 * A derivatives callback used for perturbing the actuator forces during a
 * simulation.  The "Independent" in the name refers to the fact that all
 * of the unperturbed muscles are forced to exert the force that they did 
 * during the nominal simulation during which they were recorded (ie, the 
 * intrinsic properties of the muscles are not taken into account).  This
 * is also true for the perturbed muscle - the perturbation is made to the 
 * nominal force without taking into account the intrinsic properties of the
 * muscle. 
 * NOTES:  When you use this callBack to make a perturbation the actuator force,
 *        this change in the force IS NOT recoreded in the state file.  If you
 *			 want to run an induced accleration analysis using results from a 
 * 		 perturbation, you must first alter the states field to accurately
 *			 reflect the changes made to the forces.
 * 
 *			 This callBack requires that two unperturbed integration be performed 
 * 		 prior to running a perturbation. The first is to establish the 
 *			 correct number of timesteps used in the simulation. The integrator
 *			 should be set to use the existing DTVector prior to running the 
 * 		 second unperturbed simulation, during which the unperturbed forces
 * 		 should be recorded. The user is responsible for running 
 *			 these unperturbed integrations.The callBack should be reset between
 *			 running simulations.
 *
 *			 This callBack will only work properly when the integration start
 *			 time is t = 0.0.
 *
 * @author Frank C. Anderson, Saryn R. Goldberg, May Q. Liu
 * @version 1.0
 * @todo Make the manager a global static variable so that any class can
 * get the manager and query it for information.  For example, this class
 * would like to know whether the integrator has been set to use a specified
 * set of time steps.  If the manager were available, the integrator could
 * be gotten and queried for this information.
 */
namespace OpenSim { 

class SUANALYSES_API ActuatorPerturbationIndependent :
	public ActuatorPerturbation  
{
//=============================================================================
// DATA
//=============================================================================
public:


protected:
	/** Storage for holding unperturbed forces. */
	Storage *_unperturbedForceStorage;
	/** Storage for holding perturbed forces. */
	Storage *_perturbedForceStorage;
	/** Flag that determines whether the nominal forces should be recorded. */
	bool _recordUnperturbedForces;
	/** Vector of nominal forces at current time step. */
	double *_forces;
	/** Counter to track the number of integration steps. */
	int _step;
//=============================================================================
// METHODS
//=============================================================================
public:
	ActuatorPerturbationIndependent(Model *aModel);
	virtual ~ActuatorPerturbationIndependent();
	
private:
	void setNull();
	void constructColumnLabels();
	void constructDescription();
	void deleteStorage();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setRecordUnperturbedForces(bool aTrueFalse);
	bool getRecordUnperturbedForces();
	Storage* getUnperturbedForceStorage();
	Storage* getPerturbedForceStorage();
	virtual void reset(); 
	int getStep();
	
	//--------------------------------------------------------------------------
	// CALLBACKS
	//--------------------------------------------------------------------------
	virtual void
		computeActuation(double aT,double *aX,double *aY);
	virtual void
		applyActuation(double aT,double *aX,double *aY);

//=============================================================================
};	// END of class ActuatorPerturbationIndependent

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __ActuatorPerturbationIndependent_h__
