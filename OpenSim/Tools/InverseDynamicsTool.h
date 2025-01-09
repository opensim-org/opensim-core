#ifndef OPENSIM_INVERSE_DYNAMICS_TOOL_H_
#define OPENSIM_INVERSE_DYNAMICS_TOOL_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  InverseDynamicsTool.h                       *
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

#include <OpenSim/Common/Storage.h>
#include "DynamicsTool.h"

#ifdef SWIG
    #ifdef OSIMTOOLS_API
        #undef OSIMTOOLS_API
        #define OSIMTOOLS_API
    #endif
#endif

namespace OpenSim {

class Model;
class JointSet;

//=============================================================================
//=============================================================================
/**
 * A Tool that performs an Inverse Dynamics analysis with a given model.
 * Inverse Dynamics is the solution for the generalized-coordinate forces that
 * generate given generalized-coordinate accelerations at a given state.
 * This Tool determines the state from provided coordinate trajectories as
 * functions as that are twice differentiable to estimate velocities and
 * accelerations.
 *
 * As an additional service, the InverseDynamicsTool can provide an equivalent 
 * body force (torque and force) applied to the joint frame. Since generalized
 * forces include scaling (due to units conversion as well as coupling between
 * translations and rotations, for example) they are not necessarily joint torques
 * or forces.  OpenSim employs a pseudo inverse to find the smallest applied  
 * torque and/or force that will generate the equivalent generalized force.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMTOOLS_API InverseDynamicsTool: public DynamicsTool {
OpenSim_DECLARE_CONCRETE_OBJECT(InverseDynamicsTool, DynamicsTool);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
    Storage* _coordinateValues;
protected:
    
    /** name of storage file that contains coordinate values for inverse dynamics solving */
    PropertyStr _coordinatesFileNameProp;
    std::string &_coordinatesFileName;

    /** Low-pass cut-off frequency for filtering the coordinates (does not apply to states). */
    PropertyDbl _lowpassCutoffFrequencyProp;
    double &_lowpassCutoffFrequency;

    /** name of storage file containing generalized forces from inverse dynamics */
    PropertyStr _outputGenForceFileNameProp;
    std::string &_outputGenForceFileName;

    /** Identify the list of joints for which equivalent body forces acting 
        at the joint frames should be reported */
    PropertyStrArray _jointsForReportingBodyForcesProp;
    Array<std::string> &_jointsForReportingBodyForces;

    /** name of storage file containing equivalent body forces from inverse dynamics */
    PropertyStr _outputBodyForcesAtJointsFileNameProp;
    std::string &_outputBodyForcesAtJointsFileName;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    virtual ~InverseDynamicsTool();
    InverseDynamicsTool();
    InverseDynamicsTool(const std::string &aFileName, bool aLoadModel=true) SWIG_DECLARE_EXCEPTION;
    InverseDynamicsTool(const InverseDynamicsTool &aObject);

    /* Register types to be used when reading an InverseDynamicsTool object from xml file. */
    static void registerTypes();
    /* Handle reading older formats/Versioning */
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1) override;

protected:
    /** helper method to get a list of model joints by name */
    void getJointsByName(Model &model, const Array<std::string> &jointNames, JointSet &joints) const;

private:
    void setNull();
    void setupProperties();
    /* If CoordinatesFile property is populated, load data into a live _coordinateValues
    storage object. */
    bool loadCoordinateValues();

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    InverseDynamicsTool& operator=(const InverseDynamicsTool &aInverseDynamicsTool);
#endif

    //--------------------------------------------------------------------------    
    // GET AND SET
    //--------------------------------------------------------------------------
    void setCoordinateValues(const OpenSim::Storage& aStorage);
    /**
     * get/set the name of the file to be used as output from the tool
     */
    std::string getOutputGenForceFileName() const { return _outputGenForceFileName;}
    void setOutputGenForceFileName(const std::string& desiredOutputFileName) {
        _outputGenForceFileName = desiredOutputFileName;
    }
    /**
     * get/set the name of the file containing coordinates
     */
    const std::string& getCoordinatesFileName() const { return _coordinatesFileName;};
    /** %Set the name of the coordinatesFile to be used. This call resets 
     _coordinateValues as well. */
    void setCoordinatesFileName(const std::string& aCoordinateFile)  { 
        _coordinatesFileName=aCoordinateFile;
        if (_coordinateValues != NULL){
            // there's an old Storage hanging around from potentially different 
            // CoordinatesFile, wipe it out.
            delete _coordinateValues;
            _coordinateValues = NULL;
        }
    }
    double getLowpassCutoffFrequency() const {
        return _lowpassCutoffFrequency;
    };
    void setLowpassCutoffFrequency(double aFrequency) {
        _lowpassCutoffFrequency = aFrequency;
    }
    //--------------------------------------------------------------------------
    // INTERFACE
    //--------------------------------------------------------------------------
    bool run() override SWIG_DECLARE_EXCEPTION;


//=============================================================================
};  // END of class InverseDynamicsTool
//=============================================================================
} // namespace

#endif // OPENSIM_INVERSE_DYNAMICS_TOOL_H_
