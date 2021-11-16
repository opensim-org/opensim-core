#ifndef OPENSIM_FORSIM_TOOL_H_
#define OPENSIM_FORSIM_TOOL_H_
/* -------------------------------------------------------------------------- *
 *                              ForsimTool.h                                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
 * Author(s): Colin Smith                                                     *
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

#include "osimJAMDLL.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include "OpenSim/Simulation/Model/ExternalLoads.h"
#include "OpenSim/Common/FunctionSet.h"
#include "OpenSim/Simulation/StatesTrajectory.h"

namespace OpenSim { 
//=============================================================================
//                              Forsim Tool
//=============================================================================
/**
The ForsimTool enables forward dynamic simulations of joint mechanics to be 
performed. Practically, this allows simulations of cadaver experiments, 
passive clinical examinations, and simulations where the muscle forces or 
excitations are known inputs. 

Fundamentally, this tool is similar to the ForwardTool in the opensim-core,
but the interface is designed to facilitate forward simulations involving 
joint mechanics and specifically articular contact. Additionally, the 
forsim tool uses an implicit integrator (SimTK::CPODES::BDF), which provides 
superior performance for simulations that involve contact compared to the 
explicit integrators used by the existing OpenSim ForwardTool. 

The following optional input input files can be used to define time varying 
simulation parameters:

actuator_input_file: Define muscle and actuator controls (excitations), 
activations, or forces vs time.

external_loads_file: Define the external loads and the model segments
that they are acting on. 

prescribed_coordinate_file: Define the prescribed coordinates in the 
model and their values vs time.  

## Muscle Modeling 
The controls (excitation), activation, or force in the muscles can be 
explictly defined as constants, or time varying functions. Furthermore,
the underlying models of activation and muscle-tendon dynamics can be
used in full complexity or simplified to improve simulation performance. 
A constant control can be input to all muscles using the 
constant_muscle_control property. If use_activation_dynamics is set to
false, the constant_muscle_control also becomes a constant activation (e = a). 
If use_tendon_compliance is false, then the tendons are set to be rigid.
If use_muscle_physiology is false, then activation dynamics, pennation angle 
and all force-length-velocity 
properties are ignored, the tendon is rigid, and the force in the muscle
is computed F = constant_muscle_control * max_isometric_force. 
Time varying control, activation, or force can be set for individual 
muscles in the actuator_input_file. Any muscles listed in the actuator_input_file
are not affected by the aforementioned property settings. Remember that 
default activation (ie initial) and minimum activation are
not always the same in the OpenSim muscle models. If use_activation_dynamics
is true, the intial activation will not instateously decay to the minimum
activation.
*/

class OSIMJAM_API ForsimTool : public Object {

OpenSim_DECLARE_CONCRETE_OBJECT(ForsimTool, Object);

//=============================================================================
// PROPERTIES
//=============================================================================
public:
    OpenSim_DECLARE_PROPERTY(model_file, std::string,
        "Path to .osim model file to use in the forward simulation.")

    OpenSim_DECLARE_PROPERTY(results_directory, std::string,
        "Path to folder where all results files will be written.")

    OpenSim_DECLARE_PROPERTY(results_file_basename, std::string,
        "Prefix to each results file name.")

    OpenSim_DECLARE_PROPERTY(start_time, double,
        "Time to start simulation. Set to -1 to use initial frame in inputs "
        "files. The default value is 0.0.")

    OpenSim_DECLARE_PROPERTY(stop_time, double,
        "Time to stop simulation. Set to -1 to use last frame in input files. "
        "The default value is 1.0.")

    OpenSim_DECLARE_PROPERTY(report_time_step, double,
        "The timestep interval to report the results. Set to -1 to use step "
        "size in input files. The Default value is 0.01.")

    OpenSim_DECLARE_PROPERTY(minimum_time_step, double,
        "The smallest internal timestep the integrator is allowed to take. "
        "The Default value is 0.0.")

    OpenSim_DECLARE_PROPERTY(maximum_time_step, double,
        "The largest internal timestep the integrator is allowed to take. "
        "The Default value is 0.1.")

    OpenSim_DECLARE_PROPERTY(integrator_accuracy, double,
        "Accuracy setting for BDF integrator. The Default value is 1e-6")

    OpenSim_DECLARE_PROPERTY(internal_step_limit, int,
        "Limit on the number of internal steps that can be taken by BDF "
        "integrator. If -1, then there is no limit. The Default value is -1")

    OpenSim_DECLARE_PROPERTY(constant_muscle_control, double,
        "Constant value (between 0 and 1) input as control to all muscles not "
        "listed in the actuator_input_file. "
        "Set to -1 to ignore. Default is 0.01.")

    OpenSim_DECLARE_PROPERTY(use_activation_dynamics, bool,
        "Set whether activation dynamics should be used. "
        "If false, control (i.e. excitation) = activation."
        "The default value is true.")

    OpenSim_DECLARE_PROPERTY(use_tendon_compliance, bool,
        "Set whether a compliant or rigid tendon is used in "
        "should be used in the contraction dynamics model. "
        "If false, the fiber length is removed as a state variable"
        "and simulation performance is improved."
        "The default value is true.")
    
    OpenSim_DECLARE_PROPERTY(use_muscle_physiology, bool,
        "Set whether activation dynamics, muscle force-length-velocity "
        "and pennation properties should be used. "
        "If false, use_activation_dynamics and use_tendon_compliance "
        "are ignored and muscle-tendon force is calculated using: "
        "Force = activation * max_isometric_force(). "
        "The value should be false if analyzing COMAK results. "
        "The default value is true.")

    OpenSim_DECLARE_PROPERTY(equilibrate_muscles, bool, 
        "Call equilibrateMuscles() before starting the simulation. "
        "The default value is true.")

    OpenSim_DECLARE_LIST_PROPERTY(unconstrained_coordinates, std::string,
        "Paths to the Coordinates that will be unconstrained (unlocked and "
        "not prescribed) in the simulation. All Coordinates that are not " 
        "listed here or in the prescribed_coordinates_file will be locked. "
        "Note coordinates listed here will override the 'locked' and "
        "'prescribed' properties in the .osim file.")

    OpenSim_DECLARE_PROPERTY(actuator_input_file, std::string,
        "Path to storage file (.sto) containing the time varying actuator "
        "controls, activations and forces to be applied during the "
        "simulation. The column labels must be formatted as 'time' and "
        "'ACTUATORNAME_control', 'ACTUATORNAME_activation', "
        "'ACTUATORNAME_force'.")

    OpenSim_DECLARE_PROPERTY(external_loads_file,std::string,
        "Path to .xml file that defines the ExternalLoads to apply to the "
        "model during the simulation.")

    OpenSim_DECLARE_PROPERTY(prescribed_coordinates_file, std::string, 
        "Path to storage file (.sto) containing the time varying values "
        "of the Coordinates that will be prescribed in the forward simulation."
        " The locked and prescribed properties in the model will be overriden "
        "for any Coordinates listed in this file. "
        "The columns labels must be formatted as 'time' and "
        "'/Path/To/Coordinate'")

    OpenSim_DECLARE_PROPERTY(use_visualizer, bool, "Use the SimTK visualizer "
        "to display the simulation. The default value is false.")

    OpenSim_DECLARE_PROPERTY(geometry_folder, std::string, "Optional. "
        "File path to folder containing model geometries.")


    OpenSim_DECLARE_UNNAMED_PROPERTY(AnalysisSet,"Analyses to be performed "
        "throughout the forward simulation.")

//=============================================================================
// METHODS
//=============================================================================
    
public:
    ForsimTool();
    ForsimTool(std::string settings_file);

    void setModel(Model& aModel);
    bool run();
    
private:
    void setNull();
    void constructProperties();
    void initializeCoordinates();
    void initializeActuators();
    void applyExternalLoads();
    void initializeStartStopTimes();
    void printResults();
    void printDebugInfo(const SimTK::State& state);
    
//=============================================================================
// DATA
//=============================================================================

private:
    Model _model;
    bool _model_exists;

    ExternalLoads _external_loads;

    std::vector<std::string> _prescribed_frc_actuator_paths;
    std::vector<std::string> _prescribed_act_actuator_paths;
    std::vector<std::string> _prescribed_control_actuator_paths;

    FunctionSet _frc_functions;
    FunctionSet _act_functions;

    TimeSeriesTable _actuator_table;
    TimeSeriesTable _coord_table;

    std::string _directoryOfSetupFile;

    StatesTrajectory _result_states;
    //=============================================================================
};  // END of class ForsimTool

}; //namespace

#endif // #ifndef OPENSIM_FORSIM_TOOL_H_
