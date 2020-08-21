/* -------------------------------------------------------------------------- *
 *                            OpenSim:  CMCTool.h                             *
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
#ifndef CMCTool_h__
#define CMCTool_h__

#include "osimToolsDLL.h"
#include <OpenSim/Simulation/Model/AbstractTool.h>
#include <OpenSim/Simulation/Model/ForceSet.h>

#ifdef SWIG
    #ifdef OSIMTOOLS_API
        #undef OSIMTOOLS_API
        #define OSIMTOOLS_API
    #endif
#endif

namespace OpenSim {

class ControlSet;
class Storage;

//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying the interface for an investigation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API CMCTool: public AbstractTool {
OpenSim_DECLARE_CONCRETE_OBJECT(CMCTool, AbstractTool);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:

    /** Identify the list of forces to be ignored for computing dynamics */
    PropertyStrArray _excludedActuatorsProp;
    Array<std::string> &_excludedActuators;

    /** Name of the file containing the desired kinematic
    trajectories. */
    PropertyStr _desiredPointsFileNameProp;
    std::string &_desiredPointsFileName;

    /** Name of the file containing the desired kinematic trajectories. */       
    PropertyStr _desiredKinematicsFileNameProp;          
    std::string &_desiredKinematicsFileName;

    /** Name of the file containing the tracking tasks. */           
    PropertyStr _taskSetFileNameProp;        
    std::string &_taskSetFileName;       

      /** Name of the file containing the constraints on the controls. */
    PropertyStr _constraintsFileNameProp;
    std::string &_constraintsFileName;

    /** Name of the file containing the actuator controls output by rra. */
    PropertyStr _rraControlsFileNameProp;
    std::string &_rraControlsFileName;
    /** Low-pass cut-off frequency for filtering the desired kinematics. A negative
    value results in no filtering.  The default value is -1.0, so no filtering. */
    PropertyDbl _lowpassCutoffFrequencyProp;
    double &_lowpassCutoffFrequency;

    /** Time window over which the desired actuator forces are achieved */
    PropertyDbl _targetDTProp;             
    double &_targetDT;

    /** Flag indicating whether to use the fast CMC optimization
    target.  The fast target requires the desired accelerations to           
    be met within the tolerance set by the convergence criterion.        
    The optimizer fails if the accelerations constraints cannot be        
    met, so the fast target is less robust.  The regular target          
    does not require the acceleration constraints to be met; it          
    meets them as well as it can. */         
    PropertyBool _useFastTargetProp;         
    bool &_useFastTarget;

    /** Preferred optimizer algorithm. */
    PropertyStr _optimizerAlgorithmProp;
    std::string &_optimizerAlgorithm;
    /** Perturbation size used by the optimizer to compute numerical derivatives. */
    PropertyDbl _numericalDerivativeStepSizeProp;
    double &_numericalDerivativeStepSize;
    /** Convergence criterion for the optimizer. */
    PropertyDbl _optimizationConvergenceToleranceProp;
    double &_optimizationConvergenceTolerance;
    /** Maximum number of iterations for the optimizer. */
    PropertyInt _maxIterationsProp;
    int &_maxIterations;
    /** Print level for the optimizer, 0 - 3.
    0 = no printing, ..., 3 = detailed printing. */
    PropertyInt _printLevelProp;
    int &_printLevel;
    /** Flag for turning on and off verbose printing. */
    PropertyBool _verboseProp;
    bool &_verbose;

    ForceSet _originalForceSet;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    virtual ~CMCTool();
    CMCTool();
    CMCTool(const std::string &aFileName, bool aLoadModel=true) SWIG_DECLARE_EXCEPTION;
    CMCTool(const CMCTool &aObject);

private:
    void setNull();
    void setupProperties();
    /* Get the Set of model actuators for CMC that exclude user specified Actuators */
    Set<Actuator> getActuatorsForCMC(const Array<std::string> &actuatorsByNameOrGroup);


    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    CMCTool&
        operator=(const CMCTool &aCMCTool);
#endif

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    const Array<std::string>& getExcludedActuators() const {
        return _excludedActuators;
    }
    void setExcludedActuators(const Array<std::string> &excludedActs) {
        _excludedActuators = excludedActs;
    }

    const std::string &getDesiredPointsFileName() { return _desiredPointsFileName; }
    void setDesiredPointsFileName(const std::string &aFileName) { _desiredPointsFileName = aFileName; }

    const std::string &getDesiredKinematicsFileName() { return _desiredKinematicsFileName; }
    void setDesiredKinematicsFileName(const std::string &aFileName) { _desiredKinematicsFileName = aFileName; }

    const std::string &getConstraintsFileName() { return _constraintsFileName; }         
    void setConstraintsFileName(const std::string &aFileName) { _constraintsFileName = aFileName; }          
         
    const std::string &getTaskSetFileName() { return _taskSetFileName; }         
    void setTaskSetFileName(const std::string &aFileName) { _taskSetFileName = aFileName; }

    const std::string &getRRAControlsFileName() { return _rraControlsFileName; }
    void setRRAControlsFileName(const std::string &aFileName) { _rraControlsFileName = aFileName; }

    double getLowpassCutoffFrequency() const { return _lowpassCutoffFrequency; }
    void setLowpassCutoffFrequency(double aLowpassCutoffFrequency) { _lowpassCutoffFrequency = aLowpassCutoffFrequency; }

    double getTimeWindow() const { return _targetDT; }           
    void setTimeWindow(double aTargetDT) { _targetDT = aTargetDT; }          

    // External loads get/set
    const std::string &getExternalLoadsFileName() const { return _externalLoadsFileName; }
    void setExternalLoadsFileName(const std::string &aFileName) { _externalLoadsFileName = aFileName; }

    // Target selection
    bool getUseFastTarget() const { return _useFastTarget;};         
    void setUseFastTarget(bool useFastTarget) const {  _useFastTarget=useFastTarget; };

    // Verbosity
    bool getUseVerbosePrinting() const {return _verbose;};
    void setUseVerbosePrinting(bool verbose) const { _verbose=verbose;};


    //--------------------------------------------------------------------------
    // INTERFACE
    //--------------------------------------------------------------------------
    bool run() override SWIG_DECLARE_EXCEPTION;

    //--------------------------------------------------------------------------
    // UTILITY
    //--------------------------------------------------------------------------
    Storage &getForceStorage();

    void setOriginalForceSet(const ForceSet &aForceSet);

#ifndef SWIG
    ControlSet* constructRRAControlSet(ControlSet *aControlConstraints);
    void initializeControlSetUsingConstraints(const ControlSet *aRRAControlSet,const ControlSet *aControlConstraints, ControlSet& rControlSet );
    void addNecessaryAnalyses();

#endif
//=============================================================================
};  // END of class CMCTool
//=============================================================================
//=============================================================================

}; // end namespace

#endif  // CMCTool_h__
