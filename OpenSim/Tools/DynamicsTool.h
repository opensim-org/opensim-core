#ifndef __DynamicsTool_h__
#define __DynamicsTool_h__
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  DynamicsTool.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

#include "osimToolsDLL.h"
#include <OpenSim/Simulation/Model/ExternalLoads.h>
#include "Tool.h"

#ifdef SWIG
    #ifdef OSIMTOOLS_API
        #undef OSIMTOOLS_API
        #define OSIMTOOLS_API
    #endif
#endif

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * An abstract Tool for defining tools for performing a dynamics analysis 
 * with a given model. For example, InverseDynamics and ForwardDynamics Tools
 * derive from DynamicsTool, which provides convenient method for performing
 * and dynamics analysis over or to produce a trajectory in time.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMTOOLS_API DynamicsTool: public Tool {
OpenSim_DECLARE_ABSTRACT_OBJECT(DynamicsTool, Tool);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
    
    /** Pointer to the model being investigated. */
    Model *_model;

    /** Name of the xml file used to deserialize or construct a model. */
    PropertyStr _modelFileNameProp;
    std::string &_modelFileName;

    /** The range of time over which to perform the dynamics analysis */
    PropertyDblVec2 _timeRangeProp;
    SimTK::Vec2 &_timeRange;

    /** Identify the list of forces to be ignored for computing dynamics */
    PropertyStrArray _excludedForcesProp;
    Array<std::string> &_excludedForces;

    /** Name of the file containing the external loads applied to the model. */
    OpenSim::PropertyStr _externalLoadsFileNameProp;
    std::string &_externalLoadsFileName;
    /** ExternalLoads member for creating and editing applied external forces
    (e.g. GRFs through the GUI) prior to running the Tool */
    ExternalLoads   _externalLoads;
    // Reference to external loads added to the model but not owned by the Tool
    SimTK::ReferencePtr<ExternalLoads> _modelExternalLoads;


//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    virtual ~DynamicsTool();
    DynamicsTool();
    DynamicsTool(const std::string &aFileName, bool aLoadModel=true) SWIG_DECLARE_EXCEPTION;
    DynamicsTool(const DynamicsTool &aTool);

    /** Modify model to exclude specified forces by disabling those identified by name or group */
    void disableModelForces(Model &model, SimTK::State &s, const Array<std::string> &forcesByNameOrGroup);
    
    const ExternalLoads& getExternalLoads() const { return _externalLoads; }
    ExternalLoads& updExternalLoads() { return _externalLoads; }

    // External loads get/set
    const std::string &getExternalLoadsFileName() const { return _externalLoadsFileName; }
    void setExternalLoadsFileName(const std::string &aFileName) { 
        _externalLoadsFileName = aFileName;
        _externalLoadsFileNameProp.setValueIsDefault(false);
    }

    // Model file name
    void setModelFileName(const std::string &aFileName) {
        _modelFileName = aFileName;
        _modelFileNameProp.setValueIsDefault(false);
    }

    std::string getModelFileName() const { return _modelFileName; };
private:
    void setNull();
    void setupProperties();

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    DynamicsTool& operator=(const DynamicsTool &aDynamicsTool);
#endif


    //--------------------------------------------------------------------------
    // INTERFACE
    //--------------------------------------------------------------------------
    void setStartTime(double d) { _timeRange[0] = d; };
    double getStartTime() const {return  _timeRange[0]; };

    void setEndTime(double d) { _timeRange[1] = d; };
    double getEndTime() const {return  _timeRange[1]; };
    void setModel(Model& aModel) { _model = &aModel; };

    void setExcludedForces(const Array<std::string> &aExcluded) {
        _excludedForces = aExcluded;
    }
    bool createExternalLoads( const std::string &externalLoadsFileName,
                              Model& model);

    bool modelHasExternalLoads() { return !_modelExternalLoads.empty(); }

    void removeExternalLoadsFromModel();

    virtual bool run() override SWIG_DECLARE_EXCEPTION=0;


//=============================================================================
};  // END of class DynamicsTool
//=============================================================================
} // namespace

#endif // __DynamicsTool_h__
