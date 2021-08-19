#ifndef OPENSIM_IMU_INVERSE_KINEMATICS_TOOL_H_
#define OPENSIM_IMU_INVERSE_KINEMATICS_TOOL_H_
/* -------------------------------------------------------------------------- *
 *                    OpenSim:  IMUInverseKinematicsTool.h                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Distribution Statement A – Approved for Public Release; Distribution is    *
 * Unlimited.                                                                 *
 *                                                                            *
 * IMUInverseKinematicsTool is written in part by AMJR consulting under a     *
 * contract and in a collaborative effort with The Johns Hopkins University   *
 * Applied Physics Laboratory for a project sponsored by the United States    *
 * Army Natick Soldier Research Development and Engineering Center and        *
 * supported by a the NAVAL SEA SYSTEMS COMMAND (NAVSEA) under Contract No.   *
 * N00024-13-D-6400, Task Order #VKW02. Any opinions, findings, conclusions   *
 * or recommendations expressed in this material are those of the author(s)   *
 * and do not necessarily reflect the views of NAVSEA.                        *
 *                                                                            *
 * Copyright (c) 2017-2019 AMJR Consulting and the Authors                    *
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
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/ModelDisplayHints.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Point.h>
#include <OpenSim/Simulation/OrientationsReference.h>
#include <OpenSim/Tools/InverseKinematicsToolBase.h>

namespace OpenSim {

class Model;
// Only reason for this class to exist is to have nicer name in XML
class OSIMTOOLS_API OrientationWeightSet : public Set<OrientationWeight> {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            OrientationWeightSet, Set<OrientationWeight>);

public:
    /** Use Super's constructors. */
    using Super::Super;
    // default copy, assignment operator, and destructor
    OrientationWeightSet() = default;
    OrientationWeightSet(const OrientationWeightSet&) = default;
    //=============================================================================
}; 
        //=============================================================================
//=============================================================================
/**
 * A Study that performs an Inverse Kinematics analysis with a given model.
 * Inverse kinematics is the solution of internal coordinates that poses
 * the model such that the body rotations (as measured by IMUs) affixed to the 
 * model minimize the weighted least-squares error with observations of IMU 
 * orientations in their spatial coordinates. 
 *
 * @author Ajay Seth
 */
class OSIMTOOLS_API IMUInverseKinematicsTool
        : public InverseKinematicsToolBase {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            IMUInverseKinematicsTool, InverseKinematicsToolBase);

public:
    OpenSim_DECLARE_PROPERTY(orientations_file, std::string,
        "Name/path to a .sto file of sensor frame orientations as quaternions.");

    OpenSim_DECLARE_PROPERTY(sensor_to_opensim_rotations, SimTK::Vec3,
            "Space fixed Euler angles (XYZ order) from IMU Space to OpenSim."
            " Default to (0, 0, 0).");
    OpenSim_DECLARE_PROPERTY(orientation_weights, OrientationWeightSet,
            "Set of orientation weights identified by orientation name with "
            "weight being a positive scalar. If not provided, all IMU "
            "orientations are tracked with weight 1.0.");

    //=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    virtual ~IMUInverseKinematicsTool();
    IMUInverseKinematicsTool();
    IMUInverseKinematicsTool(const std::string &setupFile);
    //--------------------------------------------------------------------------
    // INTERFACE
    //--------------------------------------------------------------------------
    bool run(bool visualizeResults) SWIG_DECLARE_EXCEPTION;
    bool run() override SWIG_DECLARE_EXCEPTION { 
        return run(false);
    };

    static TimeSeriesTable_<SimTK::Vec3>
        loadMarkersFile(const std::string& markerFile);

    void runInverseKinematicsWithOrientationsFromFile(Model& model,
                            const std::string& quaternionStoFileName, bool visualizeResults=false);

private:
    void constructProperties();

//=============================================================================
};  // END of class IMUInverseKinematicsTool
//=============================================================================
} // namespace

#endif // OPENSIM_IMU_INVERSE_KINEMATICS_TOOL_H_
