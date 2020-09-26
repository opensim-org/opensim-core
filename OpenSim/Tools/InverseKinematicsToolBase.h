#ifndef OPENSIM_INVERSE_KINEMATICS_TOOL_BASE_H_
#define OPENSIM_INVERSE_KINEMATICS_TOOL_BASE_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  InverseKinematicsToolBase.h                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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
#include "Tool.h"
#include <SimTKcommon/internal/ReferencePtr.h> 

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * A Tool that performs an Inverse Kinematics analysis with a given model.
 * Inverse kinematics is the solution of internal coordinates that poses
 * the model such that the landmark locations (markers), or orientations of
 * Sensor (IMUs) affixed to the model, minimize the weighted least-squares 
 * error with observations of markers in spatial coordinates, or Sensor 
 * (IMU) orientations. 
 *
 * @author Ayman Habib
 * @version 1.0
 */
class OSIMTOOLS_API InverseKinematicsToolBase : public Tool {
    OpenSim_DECLARE_ABSTRACT_OBJECT(InverseKinematicsToolBase, Tool);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
    
    /** Pointer to the model being investigated. */
    SimTK::ReferencePtr<Model> _model;

public:
    OpenSim_DECLARE_PROPERTY(model_file, std::string,
            "Name/path to the xml .osim file.");

    OpenSim_DECLARE_PROPERTY(constraint_weight, double,
            "The relative weighting of kinematic constraint errors. By default this "
            "is Infinity, which means constraints are strictly enforced as part of "
            "the optimization and are not appended to the objective (cost) function. "
            "Any other non-zero positive scalar is the penalty factor for "
            "constraint violations.");

    OpenSim_DECLARE_PROPERTY(accuracy, double,
            "The accuracy of the solution in absolute terms, i.e. the number of "
            "significant digits to which the solution can be trusted. Default 1e-5.");

    OpenSim_DECLARE_LIST_PROPERTY_SIZE(
            time_range, double, 2, "The time range for the study.");

    OpenSim_DECLARE_PROPERTY(report_errors, bool,
            "Flag (true or false) indicating whether or not to report "
            "errors from the inverse kinematics solution. Default is true.");

    OpenSim_DECLARE_PROPERTY(output_motion_file, std::string,
            "Name of the resulting inverse kinematics motion (.mot) file.");

    //=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    virtual ~InverseKinematicsToolBase(){};
    InverseKinematicsToolBase() { 
        constructProperties(); 
    }
    InverseKinematicsToolBase(const std::string& aFileName,
            bool aLoadModel = true) SWIG_DECLARE_EXCEPTION : 
            Tool(aFileName, aLoadModel) {
        constructProperties();
    }
 
    //---- Setters and getters for various attributes
    void setModel(Model& aModel) { _model = &aModel; };
    void setStartTime(double d) { upd_time_range(0) = d; };
    double getStartTime() const { return get_time_range(0); };

    void setEndTime(double d) { upd_time_range(1) = d; };
    double getEndTime() const { return get_time_range(1); };

    void setOutputMotionFileName(const std::string aOutputMotionFileName) {
        upd_output_motion_file() = aOutputMotionFileName;
    }
    std::string getOutputMotionFileName() { return get_output_motion_file(); }

private:
    void constructProperties() {
        constructProperty_model_file("");
        constructProperty_constraint_weight(SimTK::Infinity);
        constructProperty_accuracy(1e-5);
        Array<double> range{SimTK::Infinity, 2};
        // Make range -Infinity to Infinity unless limited by data
        range[0] = -SimTK::Infinity; 
        constructProperty_time_range(range);
        constructProperty_output_motion_file("");
        constructProperty_report_errors(true);
    };

//=============================================================================
};  // END of class InverseKinematicsToolBase
//=============================================================================
} // namespace

#endif // OPENSIM_INVERSE_KINEMATICS_TOOL_BASE_H_
