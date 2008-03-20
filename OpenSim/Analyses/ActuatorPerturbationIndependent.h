#ifndef _ActuatorPerturbationIndependent_h_
#define _ActuatorPerturbationIndependent_h_
// ActuatorPerturbationIndependent.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, Saryn R. Goldberg, May Q.Liu
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Simulation/Model/DerivCallback.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include "osimAnalysesDLL.h"
#include "Contact.h"
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

class OSIMANALYSES_API ActuatorPerturbationIndependent :
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
	/** Counter to track the number of integration steps. */
	int _step;
private:
	/** Buffer to store nominal forces at current time step (basically used as 
	 *  a local variable in computeActuation, but is made a member to avoid
	 *  having to allocate a new array in each call). */
	double *_forces;
	/** Saved unperturbed force of actuator currently being perturbed. */
	double _unperturbedForce;
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
