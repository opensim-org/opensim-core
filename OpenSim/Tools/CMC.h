/* -------------------------------------------------------------------------- *
 *                              OpenSim:  CMC.h                               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

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
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Control/TrackingController.h>

namespace SimTK {
class Optimizer;
}

namespace OpenSim {

class Model;
class OptimizationTarget;
class VectorFunctionForActuators;
class CMC_TaskSet;

//=============================================================================
//=============================================================================
/**
 * Computed Muscle Control (CMC) is an optimization-based control technique
 * designed specifically for controlling dynamic models that are actuated
 * by redundant sets of actuators whose force-generating properties may
 * be nonlinear and governed by differential equations (and so have delays
 * in force production).  The canonical example of such a dynamic system
 * is the musculoskeletal system (human or animal), hence the name
 * Computed Muscle Control.
 *
 * For a complete description of the CMC algorithm consult the following
 * references:
 *
 * Anderson FC (2004).  Method and system for dynamically filtering the
 * motion of articulated bodies. Realistic Dynamics, Inc.,
 * US Patent No. 6,750,866 (Issue date: June 15, 2004).
 *
 * Thelen DG, Anderson FC (2006). Using computed muscle control to
 * generate forward dynamic simulations of human walking from experimental
 * data. J Biomech 39: 1107-15.
 *
 * Thelen DG, Anderson FC, Delp SL (2003). Generating forward dynamic
 * simulations of movement using computed muscle control. J Biomech 36: 321-8.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */

class OSIMTOOLS_API CMC : public TrackingController {
OpenSim_DECLARE_CONCRETE_OBJECT(CMC, TrackingController);

//=============================================================================
// MEMBER VARIABLES 
//=============================================================================
private:
    /* Corresponding index of an Actuator's controls in CMC's ControlSet */
    Array<int> _controlSetIndices;

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
    std::shared_ptr<Storage> _pErrStore;
    /** Storage object for the velocity errors. */
    std::shared_ptr<Storage> _vErrStore;
    /** Storage object for the stress term weight. */
    std::shared_ptr<Storage> _stressTermWeightStore;

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
    /**
     * Controller interface
     */
    void computeControls(const SimTK::State& s, SimTK::Vector& controls) const override;
    virtual void computeInitialStates(SimTK::State& s, double &rTI);
    /** CMC algorithm */
    virtual void computeControls(SimTK::State& s, ControlSet &rX);

    //--------------------------------------------------------------------------
    // STATIC
    //--------------------------------------------------------------------------
    static void
        FilterControls(const SimTK::State& s, const ControlSet &aControlSet,double aDT,
        OpenSim::Array<double> &rControls,bool aVerbosePrinting);

     virtual void setupProperties();

 protected:
     // for any post XML deserialization initialization
     void extendConnectToModel(Model& model) override;

     // for adding any components to the underlying system
     void extendAddToSystem( SimTK::MultibodySystem& system) const override; 

//=============================================================================
};  // END of class CMC
//=============================================================================
//=============================================================================

}; // end namespace

#endif // CMC_h__


