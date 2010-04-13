// CMC.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Contributors: Frank C. Anderson
//
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

//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef CMC_h__
#define CMC_h__

//============================================================================
// INCLUDE
//============================================================================
#include "osimToolsDLL.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Control/TrackingController.h>

namespace SimTK {
class Optimizer;
}

namespace OpenSim {

class OptimizationTarget;
class VectorFunctionForActuators;
class CMC_TaskSet;

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

class OSIMTOOLS_API CMC : public TrackingController
{
//=============================================================================
// MEMBER VARIABLES	
//=============================================================================
protected:
	/** Optimizer. */
	SimTK::Optimizer *_optimizer;
	/** Optimization target for computing the controls. */
	OptimizationTarget *_target;

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

	ControlSet _controlSet;
	/** List of parameters in the control set that are serving as the
	controls in the optimization problem. */
	Array<int> _paramList;
	/** Flag to indicate whether to use verbose printing. */
	bool _verbose;
 
    bool _useCurvatureFilter;
    CMC_TaskSet *_taskSet;

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
	CMC();
	CMC(Model *aModel,CMC_TaskSet *aTaskSet);
    CMC(const CMC& aCmc);
    CMC(const std::string& aFileName, bool aUpdateFromXMLNode = true);
	virtual ~CMC();
	void setNull();

#ifndef SWIG
    CMC&  operator=(const CMC &aCmc);
#endif
    void copyData(const CMC &aCmc);
    virtual Object* copy() const;

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	Array<int>* getParameterList();
	SimTK::Optimizer* getOptimizer() const;
	OptimizationTarget* setOptimizationTarget(OptimizationTarget *aTarget, SimTK::Optimizer *aOptimizer);
	OptimizationTarget* getOptimizationTarget() const;
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
	bool getUseReflexes() const;
	void setUseVerbosePrinting(bool aTrueFalse);
    bool getUseVerbosePrinting() const;
    void setUseCurvatureFilter(bool aTrueFalse);
    bool getUseCurvatureFilter() const;
    const CMC_TaskSet& getTaskSet() const;
    CMC_TaskSet& updTaskSet() const;


    ControlSet& updControlSet() { return _controlSet; }

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	void restoreConfiguration(SimTK::State&s, const SimTK::State& initialState);
	void obtainActuatorEquilibrium(SimTK::State& s, double tiReal,double dtReal,
		const Array<double> &x,bool hold);

	//--------------------------------------------------------------------------
	// COMPUTATION
	//--------------------------------------------------------------------------
	virtual void computeInitialStates(SimTK::State& s, double &rTI);
	virtual void computeControls(SimTK::State& s, ControlSet &rX);

	//--------------------------------------------------------------------------
	// STATIC
	//--------------------------------------------------------------------------
	static void
		FilterControls(const SimTK::State& s, const ControlSet &aControlSet,double aDT,
		OpenSim::Array<double> &rControls,bool aVerbosePrinting);

    virtual double computeControl(const SimTK::State& s, int index) const;
    virtual void setActuators( Set<Actuator>& actuators );


     virtual void setupProperties();

 protected:
	 // for any post XML deserialization intialization
     virtual void setup(Model& model);

     // for adding any components to the underlying system
     virtual void createSystem( SimTK::MultibodySystem& system) const; 

     // for any intialization requiring a state or the complete system 
     virtual void initState( SimTK::State& s); 

//=============================================================================
};	// END of class CMC
//=============================================================================
//=============================================================================

}; // end namespace

#endif // CMC_h__


