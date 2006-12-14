// rdCMC.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Copyright (c) 2006 Stanford University and Realistic Dynamics, Inc.
// Contributors: Frank C. Anderson
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS,
// CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
// THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef rdCMC_h__
#define rdCMC_h__

//============================================================================
// INCLUDE
//============================================================================
#include "rdCMCDLL.h"
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/SQP/rdSQP.h>
#include <OpenSim/SQP/rdOptimizationTarget.h>
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/Model/VectorFunctionForActuators.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/SQP/rdFSQP.h>
#include "rdCMC_TaskSet.h"

#ifdef SWIG
	#ifdef RDCMC_API
		#undef RDCMC_API
		#define RDCMC_API
	#endif
#endif

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * Computed Muscle Control (CMC) is an optimization-based control technique
 * designed specifically for controlling dynamic models that are actuated
 * by redundant sets of actuators whose force-generating properties may
 * be nonlinear and goverend by differential equaitions (as so have delays
 * in force production).  The cannonical example of such a dynamic system
 * is the musculoskeletal system (human or animal), hence the name
 * Computed Muscle Control.
 *
 * For a complete description of the CMC algorithm consult the following
 * references:\n\n
 *
 * Anderson FC (2004).  Method and system for dynamically filtering the
 * motion of articulated bodies. Realistic Dynamics, Inc.,
 * US Patent No. 6,750,866 (Issue date: June 15, 2004).
 *
 * \nThelen DG, Anderson FC (2006). Using computed muscle control to
 * generate forward dynamic simulations of human walking from experimental
 * data. J Biomech 39: 1107-15.
 *
 * \nThelen DG, Anderson FC, Delp SL (2003). Generating forward dynamic
 * simulations of movement using computed muscle control. J Biomech 36: 321-8.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */

class RDCMC_API rdCMC : public Controller
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Optimizer. */
	rdFSQP *_sqp;
	/** Optimization target for computing the controls. */
	rdOptimizationTarget *_target;
	/** Set of cmc tasks. */
	rdCMC_TaskSet *_taskSet;
	/** Next integration step size that is to be taken by the integrator. */
	double _dt;
	/** Last integration step size that was taken before a new integration
	step size was set in order to step exactly to the target time. */
	double _lastDT;
	/** Flag indicating when the last integration step size should be restored.
	Normally it is restored only following a change to the integration
	step size that was made to step exactly to the end of a target interval. */
	bool _restoreDT;
	/** The target time is the time in the future (in normalized units) for
	which the controls have been calculated.  If an integrator is taking
	steps prior to the target time, the controls should not have to be
	computed again. */
	double _tf;
	/** The step size used to generate a new target time, once the
	old target time has been reached. */
	double _targetDT;
	/** Whether or not to check the target time. */
	bool _checkTargetTime;
	/** Storage object for the position errors. */
	Storage *_pErrStore;
	/** Storage object for the velocity errors. */
	Storage *_vErrStore;
	/** Storage object for the stress term weight. */
	Storage *_stressTermWeightStore;
	/** Control set for the simulation. */
	ControlSet *_controlSet;
	/** Control set for constraining the values of the controls */
	ControlSet *_controlConstraints;
	/** Flag indicating whether or not a curvature filter should be
	applied to the controls. */
	bool _useCurvatureFilter;
	/** Flag indicating whether or not reflexes should be in effect. */
	bool _useReflexes;
	/** Control set for constraining the controls based on reflexes. */
	ControlSet *_controlConstraintsFromReflexes;
	/** List of parameters in the control set that are serving as the
	controls in the optimization problem. */
	Array<int> _paramList;


	/** Vector function for estimating actuator forces over a specified time
	interval. */
	VectorFunctionForActuators *_predictor;
	/** Array of actuator forces for achieving the desired accelerations. */
	Array<double> _f;


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	rdCMC(AbstractModel *aModel,rdCMC_TaskSet *aSet);
	virtual ~rdCMC();
	void setNull();

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setControlConstraints(ControlSet *aCcontrolSet);
	ControlSet* getControlConstraints();
	ControlSet* getControlSet() const;
	rdCMC_TaskSet* getTaskSet() const;
	Array<int>* getParameterList();
	rdFSQP* getOptimizer() const;
	rdOptimizationTarget* setOptimizationTarget(rdOptimizationTarget *aTarget);
	rdOptimizationTarget* getOptimizationTarget() const;
	void setDT(double aDT);
	double getDT() const;
	void setTargetTime(double aTime);
	double getTargetTime() const;
	void setTargetDT(double aDT);
	double getTargetDT() const;
	void setCheckTargetTime(bool aTrueFalse);
	bool getCheckTargetTime() const;
	void setActuatorForcePredictor(VectorFunctionForActuators *aPredictor);
	VectorFunctionForActuators* getActuatorForcePredictor();
	Storage* getPositionErrorStorage() const;
	Storage* getVelocityErrorStorage() const;
	Storage* getStressTermWeightStorage() const;
	void setUseCurvatureFilter(bool aTrueFalse);
	bool getUseCurvatureFilter() const;
	void setUseReflexes(bool aTrueFalse);
	bool getUseReflexes() const;
	virtual void constrainControlsBasedOnReflexes(double time,Array<double> &xmin,Array<double> &xmax);

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	void restoreConfiguration(int nqnu,const double *yi,double *y);
	void obtainActuatorEquilibrium(double tiReal,double dtReal,
		Array<double> &x,Array<double> &y,bool hold);

	//--------------------------------------------------------------------------
	// COMPUTATION
	//--------------------------------------------------------------------------
	virtual void
		computeInitialStates(double &rTI,double *rYI);
	virtual void
		computeControls(double &rDT,double aT,const double *aY,ControlSet &rX);

	//--------------------------------------------------------------------------
	// STATIC
	//--------------------------------------------------------------------------
	static void
		FilterControls(const ControlSet &aControlSet,double aDT,double aT,
			   Array<double> &rControls);


//=============================================================================
};	// END of class rdCMC
//=============================================================================
//=============================================================================

}; // end namespace

#endif // rdCMC_h__


