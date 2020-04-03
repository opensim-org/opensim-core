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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/ModelDisplayHints.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Point.h>

namespace OpenSim {

class Model;
class IKTaskSet;

//=============================================================================
//=============================================================================
/**
 * A Study that performs an Inverse Kinematics analysis with a given model.
 * Inverse kinematics is the solution of internal coordinates that poses
 * the model such that the landmark locations (markers) and/or body rotations 
 * (as measured by IMUs) affixed to the model minimize the weighted least-
 * squares error with observations of analogous markers and IMU orientations
 * in their spatial coordinates. Observations of coordinates can also be
 * included.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API IMUInverseKinematicsTool : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(IMUInverseKinematicsTool, Object);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(model_file, std::string,
        "Name/path to the xml .osim file used to load a model to be analyzed.");

    OpenSim_DECLARE_PROPERTY(marker_file, std::string,
        "Name/path to a .trc or .sto file of type Vec3 of marker data.");

    OpenSim_DECLARE_PROPERTY(orientations_file, std::string,
        "Name/path to a .sto file of sensor frame orientations as quaternions.");

    OpenSim_DECLARE_PROPERTY(sensor_to_opensim_rotations, SimTK::Vec3,
            "Space fixed Euler angles (XYZ order) from IMU Space to OpenSim."
            " Default to (0, 0, 0).");

    OpenSim_DECLARE_LIST_PROPERTY_SIZE(time_range, double, 2,
        "The time range for the study.");

    OpenSim_DECLARE_PROPERTY(constraint_weight, double,
        "The relative weighting of kinematic constraint errors. By default this "
        "is Infinity, which means constraints are strictly enforced as part of "
        "the optimization and are not appended to the objective (cost) function. "
        "Any other non-zero positive scalar is the penalty factor for constraint "
        "violations.");

    OpenSim_DECLARE_PROPERTY(accuracy, double,
        "The accuracy of the solution in absolute terms, i.e. the number of "
        "significant digits to which the solution can be trusted. Default 1e-6.");

    OpenSim_DECLARE_PROPERTY(results_directory, std::string,
        "Name of the directory where results are written. Be default this is "
        "the directory in which the setup file is be  executed.");

private:
    
    /** Pointer to the model being investigated. */
    SimTK::ReferencePtr<Model> _model;

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
    bool run(bool visualizeResults=false);

    //---- Setters and getters for various attributes
    void setModel(Model& aModel) { _model = &aModel; };
    void setStartTime(double d) { upd_time_range(0) = d; };
    double getStartTime() const {return  get_time_range(0); };

    void setEndTime(double d) { upd_time_range(1) = d; };
    double getEndTime() const {return  get_time_range(1); };

    static TimeSeriesTable_<SimTK::Vec3>
        loadMarkersFile(const std::string& markerFile);

    void runInverseKinematicsWithOrientationsFromFile(Model& model,
                            const std::string& quaternionStoFileName, bool visualizeResults=false);

private:
    void constructProperties();
    // Made private to be used as debugging tool until we have a final place for it.
    void previewExperimentalData(const TimeSeriesTableVec3& markers,
        const TimeSeriesTable_<SimTK::Rotation>& orientations) const;

//=============================================================================
};  // END of class IMUInverseKinematicsTool
//=============================================================================
} // namespace

#endif // OPENSIM_IMU_INVERSE_KINEMATICS_TOOL_H_
