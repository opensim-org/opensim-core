/* -------------------------------------------------------------------------- *
 *                            OpenSim:  RRATool.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 * Contributor(s): Ayman Habib, Frank C. Anderson                             *
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
#ifndef OPENSIM_RRATool_h__
#define OPENSIM_RRATool_h__

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
class OSIMTOOLS_API RRATool: public AbstractTool {
OpenSim_DECLARE_CONCRETE_OBJECT(RRATool, AbstractTool);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:
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

    /** Low-pass cut-off frequency for filtering the desired kinematics. A negative
    value results in no filtering.  The default value is -1.0, so no filtering. */
    PropertyDbl _lowpassCutoffFrequencyProp;
    double &_lowpassCutoffFrequency;
    /** Time window */
    double _targetDT;
    /** Preferred optimizer algorithm. */
    PropertyStr _optimizerAlgorithmProp;
    std::string &_optimizerAlgorithm;
    /** Perturbation size used by the optimizer to compute numerical derivatives. */
    PropertyDbl _numericalDerivativeStepSizeProp;
    double &_numericalDerivativeStepSize;
    /** Convergence criterion for the optimizer. */
    PropertyDbl _optimizationConvergenceToleranceProp;
    double &_optimizationConvergenceTolerance;

    /** Flag indicating whether or not to make an adjustment in the center of
    mass of a body to reduced DC offsets in MX and MZ. */
    PropertyBool _adjustCOMToReduceResidualsProp;
    bool &_adjustCOMToReduceResiduals;
    /** Initial time for computing average residuals used to adjust COM. */
    PropertyDbl _initialTimeForCOMAdjustmentProp;
    double &_initialTimeForCOMAdjustment;
    /** Final time for computing average residuals used to adjust COM. */
    PropertyDbl _finalTimeForCOMAdjustmentProp;
    double &_finalTimeForCOMAdjustment;
    /** Name of the body whose center of mass is adjusted. */
    PropertyStr _adjustedCOMBodyProp;
    std::string &_adjustedCOMBody;
    /** Name of the output model file containing adjustments to anthropometry
    made to reduce average residuals. This file is written if the property
    adjust_com_to_reduce_residuals is set to true. */
    PropertyStr _outputModelFileProp;
    std::string &_outputModelFile;

    /** Flag indicating whether or not to adjust the kinematics in order to reduce residuals. */
    bool _adjustKinematicsToReduceResiduals;
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
    virtual ~RRATool();
    RRATool();
    RRATool(const std::string &aFileName, bool aLoadModel=true) SWIG_DECLARE_EXCEPTION;
    RRATool(const RRATool &aObject);

private:
    void setNull();
    void setupProperties();

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    RRATool&
        operator=(const RRATool &aRRATool);
#endif

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------

    const std::string &getDesiredPointsFileName() { return _desiredPointsFileName; }
    void setDesiredPointsFileName(const std::string &aFileName) { _desiredPointsFileName = aFileName; }

    const std::string &getDesiredKinematicsFileName() { return _desiredKinematicsFileName; }
    void setDesiredKinematicsFileName(const std::string &aFileName) { _desiredKinematicsFileName = aFileName; }

    const std::string &getConstraintsFileName() { return _constraintsFileName; }         
    void setConstraintsFileName(const std::string &aFileName) { _constraintsFileName = aFileName; }          
         
    const std::string &getTaskSetFileName() { return _taskSetFileName; }         
    void setTaskSetFileName(const std::string &aFileName) { _taskSetFileName = aFileName; }

    const std::string &getOutputModelFileName() { return _outputModelFile; }
    void setOutputModelFileName(const std::string &aFileName) { _outputModelFile = aFileName; }

    bool getAdjustCOMToReduceResiduals() { return _adjustCOMToReduceResiduals; }
    void setAdjustCOMToReduceResiduals(bool aAdjust) { _adjustCOMToReduceResiduals = aAdjust; }

    const std::string &getAdjustedCOMBody() { return _adjustedCOMBody; }
    void setAdjustedCOMBody(const std::string &aBody) { _adjustedCOMBody = aBody; }

    double getLowpassCutoffFrequency() const { return _lowpassCutoffFrequency; }
    void setLowpassCutoffFrequency(double aLowpassCutoffFrequency) { _lowpassCutoffFrequency = aLowpassCutoffFrequency; }

    // External loads get/set
    const std::string &getExternalLoadsFileName() const { return _externalLoadsFileName; }
    void setExternalLoadsFileName(const std::string &aFileName) { _externalLoadsFileName = aFileName; }

    //--------------------------------------------------------------------------
    // INTERFACE
    //--------------------------------------------------------------------------
    bool run() override SWIG_DECLARE_EXCEPTION;

    //--------------------------------------------------------------------------
    // UTILITY
    //--------------------------------------------------------------------------
    Storage &getForceStorage();

    void setOriginalForceSet(const ForceSet &aForceSet);

    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1) override;
#ifndef SWIG

    void initializeControlSetUsingConstraints(const ControlSet *aRRAControlSet,const ControlSet *aControlConstraints, ControlSet& rControlSet );
    std::string adjustCOMToReduceResiduals(SimTK::State& s, const Storage &qStore, const Storage &uStore);
    std::string adjustCOMToReduceResiduals(const OpenSim::Array<double> &aFAve,const OpenSim::Array<double> &aMAve);
    void addNecessaryAnalyses();
    void writeAdjustedModel();

    static void computeAverageResiduals(const Storage &aForceStore,OpenSim::Array<double> &rFAve,OpenSim::Array<double> &rMAve);
    static void computeAverageResiduals(SimTK::State& s, Model &aModel,double aTi,double aTf,const Storage &aStatesStore,OpenSim::Array<double>& rFAve,OpenSim::Array<double>& rMAve);
#endif
//=============================================================================
};  // END of class RRATool
//=============================================================================
//=============================================================================

}; // end namespace

#endif  // OPENSIM_RRATool_h__
