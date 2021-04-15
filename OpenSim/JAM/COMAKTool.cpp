/* -------------------------------------------------------------------------- *
 *                                 COMAKTool.cpp                              *
 * -------------------------------------------------------------------------- *
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

#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/Adapters.h>
#include <OpenSim/Common/SimmSpline.h>
#include "OpenSim/Common/Constant.h"
#include "COMAKTool.h"
#include "COMAKTarget.h"
#include "JAMUtilities.h"
#include "OpenSim/Simulation/Model/Smith2018ArticularContactForce.h"
#include "OpenSim/Simulation/Model/Blankevoort1991Ligament.h"
#include <OpenSim/Common/Stopwatch.h>
#include "OpenSim/Actuators/CoordinateActuator.h"
#include "OpenSim/Common/Constant.h"
#include <OpenSim/Common/GCVSpline.h>
#include "OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h"

using namespace OpenSim;
using namespace SimTK;

COMAKTool::COMAKTool()
{
    constructProperties();
    _directoryOfSetupFile = "";
    _model_exists = false;
}

COMAKTool::COMAKTool(const std::string file) : Object(file) {
    constructProperties();
    updateFromXMLDocument();

    _directoryOfSetupFile = IO::getParentDirectory(file);
    IO::chDir(_directoryOfSetupFile);
}

void COMAKTool::constructProperties()
{
    constructProperty_model_file("");
    constructProperty_coordinates_file("");
    constructProperty_external_loads_file("");
    constructProperty_results_directory("");
    constructProperty_results_prefix("");

    constructProperty_replace_force_set(false);
    constructProperty_force_set_file("");

    constructProperty_start_time(-1);
    constructProperty_stop_time(-1);
    constructProperty_time_step(-1);
    constructProperty_lowpass_filter_frequency(-1);
    constructProperty_print_processed_input_kinematics(false);

    constructProperty_prescribed_coordinates();
    constructProperty_primary_coordinates();
    constructProperty_COMAKSecondaryCoordinateSet(
        COMAKSecondaryCoordinateSet());

    constructProperty_settle_secondary_coordinates_at_start(true);
    constructProperty_settle_threshold(1e-5);
    constructProperty_settle_accuracy(1e-6);
    constructProperty_settle_internal_step_limit(-1);
    constructProperty_print_settle_sim_results(false);
    constructProperty_settle_sim_results_directory("");
    constructProperty_settle_sim_results_prefix("");

    constructProperty_max_iterations(50);
    constructProperty_udot_tolerance(1.0);
    constructProperty_udot_worse_case_tolerance(50.0);
    constructProperty_unit_udot_epsilon(1e-8);
    constructProperty_optimization_scale_delta_coord(1e-3);
    
    constructProperty_ipopt_diagnostics_level(0);
    constructProperty_ipopt_max_iterations(500);
    constructProperty_ipopt_convergence_tolerance(1e-4);
    constructProperty_ipopt_constraint_tolerance(1e-4);
    constructProperty_ipopt_limited_memory_history(6);
    constructProperty_ipopt_nlp_scaling_max_gradient(1000);
    constructProperty_ipopt_nlp_scaling_min_value(1e-9);
    constructProperty_ipopt_obj_scaling_factor(1);

    constructProperty_activation_exponent(2);
    constructProperty_contact_energy_weight(0.0);
    constructProperty_non_muscle_actuator_weight(1000);
    constructProperty_COMAKCostFunctionParameterSet(
        COMAKCostFunctionParameterSet());

    constructProperty_model_assembly_accuracy(1e-12);
    constructProperty_use_visualizer(false);
    constructProperty_verbose(0);

    constructProperty_AnalysisSet(AnalysisSet());
}

void COMAKTool::setModel(Model& model) {
    _model = model;
    set_model_file(model.getDocumentFileName());
    _model_exists = true;
}

bool COMAKTool::run()
{
    printCOMAKascii();

    performCOMAK();
    return true;
}

SimTK::State COMAKTool::initialize()
{
    //Make results directory
    int makeDir_out = IO::makeDir(get_results_directory());
    if (errno == ENOENT && makeDir_out == -1) {
        OPENSIM_THROW(Exception, "Could not create " +
            get_results_directory() +
            "Possible reason: This tool cannot make new folder with "
            "subfolder.");
    }
    
    //Set Model
    if (!_model_exists) {
        if (get_model_file().empty()) {
            OPENSIM_THROW(Exception, "No model was set in the COMAKTool.");
        }
        _model = Model(get_model_file());
    }

    updateModelForces();

    SimTK::State state = _model.initSystem();

    // Verfiy Coordinate Properties
    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        std::string name = coord.getName();
        std::string path = coord.getAbsolutePathString();
        bool isSecondary = false;

        // Reset coordinates listed in settings file to full path
        int ind = getProperty_prescribed_coordinates().findIndex(name);
        if (ind > -1) {
            set_prescribed_coordinates(ind, path);
        }

        ind = getProperty_primary_coordinates().findIndex(name);
        if (ind > -1) {
            set_primary_coordinates(ind, path);
        }

        for (int i = 0; i < get_COMAKSecondaryCoordinateSet().getSize(); ++i) {
            COMAKSecondaryCoordinate secondary_coord = 
                get_COMAKSecondaryCoordinateSet().get(i);
            if (secondary_coord.get_coordinate() == name || 
                secondary_coord.get_coordinate() == path) {
                secondary_coord.set_coordinate(path);
                isSecondary = true;
            }
        }

        bool isPrescribed = 
            getProperty_prescribed_coordinates().findIndex(path) > -1;
        bool isPrimary = 
            getProperty_primary_coordinates().findIndex(path) > -1;
        
        // Set all unlisted coordinates to prescribed
        if (!isPrescribed && !isPrimary && !isSecondary) {
            append_prescribed_coordinates(path);

            log_warn("WARNING: Coordinate ({}"
                ") was not listed in COMAKTool params file. "
                "Assumed PRESCRIBED.", name);
        }

        // Make sure coordinate is not listed multiple times
        int counted = 0;
        if (isPrescribed)
            counted++;
        if (isPrimary)
            counted++;
        if (isSecondary)
            counted++;
    
        if (counted > 1) {
            OPENSIM_THROW(Exception, "Coordinate: " + name + 
                " was listed as multiple COMAKTool coordinate types "
                "(Prescribed, Primary, Secondary).")
        }
    }

    // Make sure Coordinates exist in model and no duplicates
    for (int i = 0; i < getProperty_prescribed_coordinates().size(); ++i) {
        std::string name = get_prescribed_coordinates(i);
        try { _model.getComponent<Coordinate>(name);}
        catch (Exception) {
            OPENSIM_THROW(Exception, "prescribed_coordinate: " 
                + name + " not found in model.")
        }

        int n = 0;
        for (int j = 0; j < getProperty_prescribed_coordinates().size(); ++j) {
            if (name == get_prescribed_coordinates(j)) n++;
        }
        OPENSIM_THROW_IF(n>1, Exception, name + 
            "listed multiple times in prescribed_coordinates")
    }

    for (int i = 0; i < getProperty_primary_coordinates().size(); ++i) {
        std::string name = get_primary_coordinates(i);
        try { _model.getComponent<Coordinate>(name); }
        catch (Exception) {
            OPENSIM_THROW(Exception, "primary_coordinate: " 
                + name + "not found in model.")
        }

        int n = 0;
        for (int j = 0; j < getProperty_primary_coordinates().size(); ++j) {
            if (name == get_primary_coordinates(j)) n++;
        }
        OPENSIM_THROW_IF(n>1, Exception, name + 
            "listed multiple times in primary_coordinates")
    }

    for (int i = 0; i < get_COMAKSecondaryCoordinateSet().getSize(); ++i) {
        std::string path = 
            get_COMAKSecondaryCoordinateSet().get(i).get_coordinate();
        try { _model.getComponent<Coordinate>(path); }
        catch (Exception){
            OPENSIM_THROW(Exception,"secondary_coordinate: " + path +
                "not found in model.")
        }

        int n = 0;
        for (int j = 0; j < get_COMAKSecondaryCoordinateSet().getSize(); ++j) {
            if (path == 
                get_COMAKSecondaryCoordinateSet().get(j).get_coordinate()) n++;
        }
        OPENSIM_THROW_IF(n>1, Exception, path + 
            "listed multiple times in secondary_coordinates")
    }

    // Count number of each coordinate type
    _n_prescribed_coord = getProperty_prescribed_coordinates().size();
    _n_primary_coord = getProperty_primary_coordinates().size();
    _n_secondary_coord = get_COMAKSecondaryCoordinateSet().getSize();

    _prescribed_coord_name.setSize(_n_prescribed_coord);
    _prescribed_coord_path.setSize(_n_prescribed_coord);
    _prescribed_coord_index.setSize(_n_prescribed_coord);

    _primary_coord_name.setSize(_n_primary_coord);
    _primary_coord_path.setSize(_n_primary_coord);
    _primary_coord_index.setSize(_n_primary_coord);

    _secondary_coord_name.setSize(_n_secondary_coord);
    _secondary_coord_path.setSize(_n_secondary_coord);
    _secondary_coord_index.setSize(_n_secondary_coord);

    // Organize coordinates in nice stuctures
    for (int i = 0; i < _n_prescribed_coord; ++i) {
        _prescribed_coord_path[i] = get_prescribed_coordinates(i);
        _prescribed_coord_name[i] = 
            _model.getComponent<Coordinate>(
                get_prescribed_coordinates(i)).getName();
    }

    for (int i = 0; i < _n_primary_coord; ++i) {
        _primary_coord_path[i] = get_primary_coordinates(i);
        _primary_coord_name[i] = _model.getComponent<Coordinate>(
            get_primary_coordinates(i)).getName();
    }

    for (int i = 0; i < _n_secondary_coord; ++i) {
        _secondary_coord_path[i] = 
            get_COMAKSecondaryCoordinateSet().get(i).get_coordinate();
        _secondary_coord_name[i] = 
            _model.getComponent<Coordinate>(
                _secondary_coord_path[i]).getName();
    }

    // Find the index of each coordinate in the updComponentList<Coordinate>
    int nCoord = 0;
    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        std::string path = coord.getAbsolutePathString();

        int ind = _prescribed_coord_path.findIndex(path);
        if ( ind > -1){
            _prescribed_coord_index[ind] = nCoord;
        }

        ind = _primary_coord_path.findIndex(path);
        if (ind > -1) {
            _primary_coord_index[ind] = nCoord;
        }

        ind = _secondary_coord_path.findIndex(path);
        if (ind > -1) {
            _secondary_coord_index[ind] = nCoord;
        }
        nCoord++;
    }


    log_info("Prescribed Coordinates:");
    log_info("-----------------------");
    for (int i = 0; i < _n_prescribed_coord; ++i) {
        log_info(_prescribed_coord_name[i]);
    }

    log_info("Primary Coordinates:");
    log_info("--------------------");
    for (int i = 0; i < _n_primary_coord; ++i) {
        log_info(_primary_coord_name[i]);
    }

    log_info("Secondary Coordinates:");
    log_info("----------------------");
    for (int i = 0; i < _n_secondary_coord; ++i) {
        log_info(_secondary_coord_name[i]);
    }

    //Organize COMAKSecondaryCoordinate Properties
    _secondary_coord_max_change.resize(_n_secondary_coord);
    _secondary_coord_max_change = 0;

    for (int i = 0; i < _n_secondary_coord; ++i) {
        for (int j = 0; j < get_COMAKSecondaryCoordinateSet().getSize(); ++j) {
            
            COMAKSecondaryCoordinate sec_coord = 
                get_COMAKSecondaryCoordinateSet().get(j);

            if (sec_coord.get_coordinate() == _secondary_coord_path[i]) {
                _secondary_coord_max_change[i] = sec_coord.get_max_change();
            }
        }
    }

    //Check for Coordinate Actuators on Prescribed Coordinates
    bool first_replaced = true;
    for (auto& coord_act : _model.updComponentList<CoordinateActuator>()) {

        std::string coord_path = 
            coord_act.getCoordinate()->getAbsolutePathString();

        int ind = _prescribed_coord_path.findIndex(coord_path);
        if (ind > -1) {
            if (first_replaced) {
                first_replaced = false;

                log_warn("The following CoordinateActuators are "
                    " acting on a prescribed coordinate. These actuators will "
                    "be disabled because they will do nothing while adding "
                    "extra variables to the optimization.");
                
            }
            log_warn("{} ({}).",
                coord_act.getAbsolutePathString(), coord_act.get_coordinate());


            coord_act.setAppliesForce(state, false);
        }
    }

    //Check for Coordinate Coupler constrainsts on prescribed coords
    for (CoordinateCouplerConstraint& cc_const : 
        _model.updComponentList<CoordinateCouplerConstraint>()){
        
        std::string cc_coord_name = cc_const.getDependentCoordinateName();

        std::string coord_path = 
            _model.updCoordinateSet().get(cc_coord_name).
            getAbsolutePathString();

        int ind = _prescribed_coord_path.findIndex(coord_path);
        if (ind > -1) {
            cc_const.set_isEnforced(false);
        }
        log_info("CoordinateCouplerConstraint: {}"
            " will not be enforced because the dependent coordinate: {}"
            " is prescribed.", cc_const.getName(), cc_coord_name);

    }

    // TO DO
    //Check for damping on the secondary coordinates
    //
    // HOW? perturbing the speeds and checking for acc change will demonstrate
    // damping is present, except that changing speed
    // has a direct effect on acc ...
    // maybe check nominal acc vs acc w/disable all forces?
    /*
    std::cout << "\n\nDAMPING CHECK "<< std::endl;
    for (int i = 0; i < _n_secondary_coord; ++i) {
        _model.realizeAcceleration(state);
        
        Coordinate& optim_coord = 
            _model.updComponent<Coordinate>(_secondary_coord_path[i]);

        double init_acc = optim_coord.getAccelerationValue(state);
        double init_speed = optim_coord.getSpeedValue(state);

        optim_coord.setSpeedValue(state, init_speed + 1.0);
        
        _model.realizeAcceleration(state);

        double new_acc = optim_coord.getAccelerationValue(state);
        double diff = new_acc - init_acc;
        std::cout << optim_coord.getName() << " " << diff << std::endl;
        if (diff == 0) {
            
        }

        optim_coord.setSpeedValue(state, init_speed);
    }
    */

    //Organize and Compute Optimal Forces For Muscles/Actuators in model
    _n_actuators = 0;
    _n_muscles = 0;
    _n_non_muscle_actuators = 0;

    for (ScalarActuator &actuator : 
        _model.updComponentList<ScalarActuator>()) {

        if (actuator.appliesForce(state)) {
            _n_actuators++;
        }
    }
    _optimal_force.resize(_n_actuators);

    int i = 0;
    for (const Muscle& msl : _model.updComponentList<Muscle>()) {
        if (msl.appliesForce(state)) {
            _optimal_force[i] = msl.getMaxIsometricForce();
            i++;
            _n_muscles++;
            _muscle_path.append(msl.getAbsolutePathString());
        }
    }

    for (ScalarActuator &actuator : 
        _model.updComponentList<ScalarActuator>()) {

        if (Object::isObjectTypeDerivedFrom< Muscle >(
            actuator.getConcreteClassName())) {
            continue;
        }

        if (actuator.appliesForce(state)) {
            _optimal_force[i] = actuator.getOptimalForce();
            i++;
            _n_non_muscle_actuators++;
            _non_muscle_actuator_path.append(actuator.getAbsolutePathString());
        }
    }

    computeMuscleVolumes();

    if (get_verbose() > 1) {
        int w = 15;
        log_info("\nMuscle Properties");
        std::cout << std::setw(15) << "name " << std::setw(15) << "Fmax" << 
            std::setw(15) << "Volume" << std::endl;
        i = 0;
        for (const Muscle& msl : _model.getComponentList<Muscle>()) {
            double l0 = msl.get_optimal_fiber_length();
            double fmax = msl.get_max_isometric_force();

            std::cout << std::setw(15) << msl.getName() <<  std::setw(15) << 
                fmax << std::setw(15) << _muscle_volumes[i] << std::endl;
            i++;
        }
    }

    // Setup optimization parameters
    // parameters vector ordered
    // muscle activations 
    // non muscle activations
    // secondary coord values

    _optim_parameters.resize(_n_actuators + _n_secondary_coord);
    _optim_parameters = 0;

    _n_parameters = _optim_parameters.size();
    _prev_parameters = _optim_parameters;

    _optim_parameter_names.setSize(_n_parameters);

    int p = 0;
    for (const Muscle &msl : _model.updComponentList<Muscle>()) {
        _optim_parameter_names[p] = msl.getName();
        p++;
    }
    for (const auto &actuator : _model.updComponentList<ScalarActuator>()) {
        if (Object::isObjectTypeDerivedFrom<Muscle>(
            actuator.getConcreteClassName())) {
            continue;
        }
        _optim_parameter_names[p] = actuator.getName();
        p++;
    }
    for (int p = 0; p < _n_secondary_coord; ++p) {
        _optim_parameter_names[_n_actuators + p] = _secondary_coord_name[p];
    }
   
    //Optimization Constraints
    _n_optim_constraints = 0;
    for (const Coordinate& coord : _model.getComponentList<Coordinate>()) {

        bool is_primary =
            _primary_coord_path.findIndex(coord.getAbsolutePathString()) > -1;
        bool is_secondary =
            _secondary_coord_path.findIndex(coord.getAbsolutePathString())> -1;

        if (is_primary) {
            _optim_constraint_names.append(coord.getName());
            _n_optim_constraints++;
        }
        if(is_secondary) {
            _optim_constraint_names.append(coord.getName());
            _n_optim_constraints++;
        }
    }

    if (get_verbose() > 5) {
        std::cout << "\nIPOPT Parameter Names" << std::endl;
        for (int p = 0; p < _n_parameters; p++) {
            std::cout << p << "\t"<< _optim_parameter_names[p] <<std::endl;
        }

        std::cout << "\nIPOPT Constraint Names" << std::endl;
        for (int p = 0; p < _n_optim_constraints; p++) {
            std::cout << p << "\t"<< _optim_constraint_names[p] <<std::endl;
        }
    }

    //Organize COMAKCostFunctionParameters
    for (Muscle &msl : _model.updComponentList<Muscle>()) {
        bool found_msl = false;
        
        int size_cost_fcn_param_set =
            get_COMAKCostFunctionParameterSet().getSize();

        for (int j = 0; j < size_cost_fcn_param_set; ++j) {
            COMAKCostFunctionParameter parameter = 
                get_COMAKCostFunctionParameterSet().get(j);

            if (parameter.get_actuator() == msl.getAbsolutePathString()) {
                found_msl = true;
                _cost_muscle_weights.cloneAndAppend(parameter.get_weight());
            }
        }
        if(found_msl == false){
            _cost_muscle_weights.cloneAndAppend(Constant(1.0));
            
        }
    }

    //Add Analysis set
    AnalysisSet aSet = get_AnalysisSet();
    int size = aSet.getSize();

    for(int i=0;i<size;i++) {
        Analysis *analysis = aSet.get(i).clone();
        _model.addAnalysis(analysis);
    }

    return state;
}

void COMAKTool::performCOMAK()
{


    SimTK::State state = initialize();

    //Read Kinematics and Compute Desired Accelerations
    extractKinematicsFromFile();

    //Initialize Secondary Kinematics
    SimTK::Vector init_secondary_values(_n_secondary_coord);

    if (get_settle_secondary_coordinates_at_start()) {
        init_secondary_values = equilibriateSecondaryCoordinates();
    }
    else {
        for (int i = 0; i < _n_secondary_coord; ++i) {
            init_secondary_values(i) = 
                _q_matrix(_start_frame, _secondary_coord_index[i]);
        }
    }

    log_info("Initial Secondary Coordinate Values:");

    for (int i = 0; i < _n_secondary_coord; ++i) {
        log_info("{} :\t {}",
            _secondary_coord_name[i], init_secondary_values(i));
    }
        
    

    //Apply External Loads
    applyExternalLoads();

    //Set Prescribed Coordinates
    int nCoord = 0;
    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        bool is_prescribed =
            getProperty_prescribed_coordinates().findIndex(
                coord.getAbsolutePathString()) > -1;

        if (is_prescribed) {
            SimTK::Vector data = _q_matrix(nCoord);
            GCVSpline func = 
                GCVSpline(5, _n_frames, &_time[0], &data[0], coord.getName());
            coord.set_prescribed_function(func);
            coord.set_prescribed(true);
        }
        nCoord++;
    }



    if (get_use_visualizer()) {
        _model.setUseVisualizer(true);
    }

    _model.set_assembly_accuracy(get_model_assembly_accuracy());

    state = _model.initSystem();

    //Visualizer Settings
    SimTK::Visualizer* viz=NULL;
    if (get_use_visualizer()) {
        viz = &_model.updVisualizer().updSimbodyVisualizer();
        viz->setBackgroundColor(SimTK::White);
        viz->setShowSimTime(true);
    }

    //Setup Results Storage
    initializeResultsStorage();

    AnalysisSet& analysisSet = _model.updAnalysisSet();

    //Prepare for Optimization
    log_info("\nPerforming COMAK...\n");

    _model.setAllControllersEnabled(false);

    for (ScalarActuator &actuator : 
        _model.updComponentList<ScalarActuator>()) {
        actuator.overrideActuation(state, true);
    }
    
    //Initialize optimization parameters
    for (int i = 0; i < _n_muscles; ++i) {
        _optim_parameters[i] = 0.02;
    }
    for (int i = 0; i < _n_secondary_coord; ++i) {
        _optim_parameters[i + _n_actuators] = 0.0;
    }
    _prev_parameters = _optim_parameters;

    //Set initial Secondary Qs
    _prev_secondary_value.resize(_n_secondary_coord);

    for (int j = 0; j < _n_secondary_coord; ++j) {
        Coordinate& coord = 
            _model.updComponent<Coordinate>(_secondary_coord_path[j]);

        coord.setValue(state, init_secondary_values(j), false);
        _prev_secondary_value(j) = init_secondary_values(j);
    }
    _prev_state = state;

    //Loop over each time step
    //------------------------

    _consecutive_bad_frame = 0;
    int frame_num = 0;
    for (int i = 0; i < _n_frames; ++i) {
        if (_time[i] < get_start_time()) { continue; }
        if (_time[i] > get_stop_time()) { break; };

        //Set Time
        state.setTime(_time[i]);

        log_info("Frame: {} / {}", ++frame_num, _n_out_frames);
        log_info("Time: {}", _time[i]);

        //Set Primary Qs and Us to experimental values
        for (int j = 0; j < _n_primary_coord; ++j) {
            Coordinate& coord = 
                _model.updComponent<Coordinate>(_primary_coord_path[j]);

            coord.setValue(state,_q_matrix(i, _primary_coord_index[j]), false);
            coord.setSpeedValue(state, _u_matrix(i, _primary_coord_index[j]));
        }

        //Project Secondary values based on speeds 
        for (int j = 0; j < _n_secondary_coord; ++j) {
            Coordinate& coord = 
                _model.updComponent<Coordinate>(_secondary_coord_path[j]);
            
            double prev_value = coord.getValue(state);
            double prev_speed = coord.getSpeedValue(state);

            double new_value = prev_value + prev_speed * _dt;
            coord.setValue(state, new_value);
        }

        //Reset the Secondary Us to zero
        for (int j = 0; j < _n_secondary_coord; ++j) {
            Coordinate& coord = 
                _model.updComponent<Coordinate>(_secondary_coord_path[j]);
            coord.setSpeedValue(state, 0.0);
        }

        _model.assemble(state);
        _model.realizeVelocity(state);

        //Iterate for COMAK Solution
        double max_udot_error = SimTK::Infinity;
        SimTK::Vector iter_max_udot_error(get_max_iterations(), 0.0);
        std::vector<std::string> iter_max_udot_coord(get_max_iterations(), "");
        SimTK::Matrix iter_parameters(get_max_iterations(),_n_parameters, 0.0);
        std::vector<SimTK::State> iter_states(get_max_iterations());
        int n_iter = 0;

        for (int iter = 0; iter < get_max_iterations(); ++iter) {
            n_iter++;

            log_info("---------------------------------------"
                    "---------------------------------------");
            log_info("Frame: {} \t Time: {} \t Iteration: {}", frame_num);

            log_info("---------------------------------------"
                    "---------------------------------------");
            

            //Reset the optimized delta coords to zero
            for (int m = 0; m < _n_secondary_coord; ++m) {
                _optim_parameters[m + _n_actuators] = 0.0;
            }
            
            //Reset Reserve actuators
            for (int m = 0; m < _n_non_muscle_actuators ; ++m) {
                _optim_parameters[m + _n_muscles] = 0.0;
            }

            ComakTarget target = ComakTarget(state, &_model, 
                ~_udot_matrix[i],
                _optim_parameters,
                _dt, get_unit_udot_epsilon(),
                _primary_coord_path, _secondary_coord_path,
                _muscle_path, _non_muscle_actuator_path,
                _secondary_coord_max_change);

            target.setOptimalForces(_optimal_force);
            target.setMuscleVolumes(_normalized_muscle_volumes);
            target.setContactEnergyWeight(get_contact_energy_weight());
            target.setNonMuscleActuatorWeight(
                get_non_muscle_actuator_weight());
            target.setScaleDeltaCoordinates(
                get_optimization_scale_delta_coord());
            target.setActivationExponent(get_activation_exponent());
            target.setParameterNames(_optim_parameter_names);
            target.setVerbose(get_verbose());

            SimTK::Vector msl_weight(_n_muscles);
            for (int m = 0; m < _n_muscles; ++m) {
                msl_weight(m) = 
                    _cost_muscle_weights.get(m).calcValue(
                        SimTK::Vector(1, _time[i]));
            }

            target.setCostFunctionWeight(msl_weight);
            target.initialize();

            SimTK::OptimizerAlgorithm algorithm = SimTK::InteriorPoint;
            SimTK::Optimizer optimizer(target, algorithm);

            optimizer.setDiagnosticsLevel(get_ipopt_diagnostics_level());
            optimizer.setMaxIterations(get_ipopt_max_iterations());
            optimizer.setConvergenceTolerance(
                get_ipopt_convergence_tolerance());
            optimizer.setConstraintTolerance(
                get_ipopt_constraint_tolerance());
            optimizer.useNumericalGradient(false);
            optimizer.useNumericalJacobian(false);

            // Some IPOPT-specific settings
            optimizer.setLimitedMemoryHistory(
                get_ipopt_limited_memory_history()); 
            optimizer.setAdvancedBoolOption("warm_start",true);
            optimizer.setAdvancedRealOption("nlp_scaling_max_gradient",
                get_ipopt_nlp_scaling_max_gradient());
            optimizer.setAdvancedRealOption("nlp_scaling_min_value",
                get_ipopt_nlp_scaling_min_value());
            optimizer.setAdvancedRealOption("obj_scaling_factor",
                get_ipopt_obj_scaling_factor());
            optimizer.setAdvancedStrOption("expect_infeasible_problem", "yes");
           

            //optimizer.setAdvancedStrOption("hessian_approximation", "exact");
            
            // For debugging cost and constraint changes
            /*optimizer.setAdvancedStrOption("derivative_test", "first-order");
            optimizer.setAdvancedBoolOption(
                "derivative_test_print_all", true);
            optimizer.setAdvancedRealOption(
                "derivative_test_perturbation", 1e-6);
            */
            
            try {
                optimizer.optimize(_optim_parameters);
            }
            catch (SimTK::Exception::Base ex) {
                log_error("COMAK Optimization failed: {}", ex.getMessage());
            }

            //Account for optimization scale factors
            for (int m = 0; m < _n_secondary_coord; ++m) {
                _optim_parameters[m + _n_actuators] *= 
                    get_optimization_scale_delta_coord();
            }

            setStateFromComakParameters(state, _optim_parameters);
            
            //Save iteration in case of no convergence
            iter_states[iter] = state;
            iter_parameters[iter] = ~_optim_parameters;

            //Compute udot linearized error
            _model.realizeAcceleration(state);

            //Output Optimization Results
            printOptimizationResultsToConsole(_optim_parameters, state);

            if (get_verbose() > 1) {
                std::cout << "\nOptimized Acceleration Errors:" << std::endl;
                std::cout 
                    << std::setw(20) << "Name" 
                    << std::setw(20) << "Experimental" 
                    << std::setw(20) << "Simulated" 
                    << std::setw(20) << "Error" << std::endl;
            }
            
            std::string max_udot_coord = "";
            int k = 0;
            max_udot_error = 0;

            for (const auto& coord : _model.getComponentList<Coordinate>()) {
                std::string path = coord.getAbsolutePathString();
                bool is_secondary = _secondary_coord_path.findIndex(path) > -1;
                bool is_primary = _primary_coord_path.findIndex(path) > -1;

                double coord_udot = coord.getAccelerationValue(state);
                double udot_error;
                double observed_udot;

                if (is_secondary) {
                    //Cannot get kinematics from mocap
                    observed_udot = 0.0;
                }
                else {
                    //Accelerations from spline fit to IK joint angles
                    observed_udot = _udot_matrix(i, k);
                }

                //Difference between simulated and observed accelerations:
                //Errors in simulated accelerations occur because effect of
                //secondary coordinate change (optimized variable) on joint
                //accelerations (optimization constraint) is linearized so 
                //that the slow contact model isn't solved in every call 
                //to the optimization constraint and constraint jacobian 
                //functions. Iterations drive this error to zero once the
                //optimization solution is only a small change from the 
                //initial secondary coordinates that you linearize around. 

                udot_error = abs(observed_udot - coord_udot);

                k++;

                if (!is_secondary && !is_primary) {
                    continue;
                }

                if (udot_error > max_udot_error) {
                    max_udot_coord = coord.getName();
                    max_udot_error = udot_error;
                }

                if (get_verbose() > 1) {
                    std::cout 
                        << std::setw(20) << coord.getName() 
                        << std::setw(20) << observed_udot 
                        << std::setw(20) << coord_udot 
                        << std::setw(20) << udot_error << std::endl;
                }
            }
            
            log_info("Max udot Error: {} \t Max Error Coord: {}", 
                max_udot_error, max_udot_coord);
            iter_max_udot_error(iter) = max_udot_error;
            iter_max_udot_coord[iter] = max_udot_coord;

            
            //Check for convergence
            if (max_udot_error < get_udot_tolerance()) {
                _consecutive_bad_frame = 0; //converged so reset
                break;
            }
            
        }// END COMAK ITERATION
                
        if (max_udot_error > get_udot_tolerance()) {

            std::cout << std::endl;
            std::cout << "COMAK failed to converge." << std::endl;

            _consecutive_bad_frame++;

            //Reset to best iteration solution
            double min_val = get_udot_worse_case_tolerance();
            int min_iter = -1;
            std::string bad_coord;
            for (int m = 0; m < get_max_iterations(); ++m) {
                if (iter_max_udot_error(m) < min_val) {
                    min_val = iter_max_udot_error(m);
                    min_iter = m;
                    bad_coord = iter_max_udot_coord[m];
                }
            }
            if (min_iter > -1) {
                _optim_parameters = ~iter_parameters[min_iter];
                state = iter_states[min_iter];

                std::cout << "Using best iteration (" << min_iter 
                    << ") with max udot error: " << min_val << std::endl;
            }
            else {
                _optim_parameters = _prev_parameters;

                setStateFromComakParameters(state, _prev_parameters);

                state.setTime(_time[i]);

                std::cout << "No iteration has max udot error less than "
                    "worst case tolerance (" 
                    << get_udot_worse_case_tolerance() << ").\n " <<
                    "Resetting optimization parameters to previous "
                    "time step solution." << std::endl;
            }

            //Save data about failed convergence
            _bad_frames.push_back(i);
            _bad_times.push_back(_time[i]);
            _bad_udot_errors.push_back(min_val);
            _bad_udot_coord.push_back(bad_coord);
        }
        else {
            std::cout << "COMAK Converged! Number of iterations: " 
                << n_iter << std::endl << std::endl;
        }

        //Store Solution
        _model.realizeAcceleration(state);
        _prev_parameters = _optim_parameters;
        _prev_state = state;

        for (int m = 0; m < _n_secondary_coord; ++m) {
            Coordinate& coord = 
                _model.updComponent<Coordinate>(_secondary_coord_path[m]);
            _prev_secondary_value(m) = coord.getValue(state);
        }

        //Save the results
        recordResultsStorage(state,i);
 
        //Visualize the Results
        if (get_use_visualizer()) {
            int k = 0;
            for (int m = 0; m < _n_muscles; ++m) {
                Muscle &msl = _model.updComponent<Muscle>(_muscle_path[m]);
                msl.overrideActuation(state, false);
                _model.realizePosition(state);
                msl.setActivation(state, _optim_parameters[k]);
                k++;
            }

            _model.realizeAcceleration(state);
            viz->drawFrameNow(state);

            for (int m = 0; m < _n_muscles; ++m) {
                Muscle &msl = _model.updComponent<Muscle>(_muscle_path[m]);
                msl.overrideActuation(state, true);
            }
        }
    } //END of COMAK timestep

    //Print Convergence Summary
    std::cout << "Convergence Summary:" << std::endl;
    std::cout << "--------------------" << std::endl;

    if (_bad_frames.empty()) {
        std::cout << "All frames converged!" << std::endl;
    }
    else {
        std::cout 
            << std::setw(15) << "Bad Times" 
            << std::setw(15) << "Bad Frames" 
            << std::setw(15) << "udot error" << std::endl;

        for (int i = 0; i < (int)_bad_frames.size(); i++) {
            std::cout 
                << std::setw(15) << _bad_times[i] 
                << std::setw(15) << _bad_frames[i] 
                << std::setw(15) << _bad_udot_errors[i] << std::endl;
        }
    }
    //Print Results
    printResultsFiles();
}

void COMAKTool::setStateFromComakParameters(
    SimTK::State& state, const SimTK::Vector& parameters) {

    //Set Muscle Activations to Optimized
    int j = 0;
    for (int m = 0; m < _n_muscles; ++m) {
        Muscle &msl = _model.updComponent<Muscle>(_muscle_path[m]);
        msl.overrideActuation(state, true);
        double force = _optimal_force[j] * parameters[j];
        msl.setOverrideActuation(state,force);
        j++;
    }

    //Set Reserve Activations to Optimized
    for (int m = 0; m < _n_non_muscle_actuators; ++m) {
        ScalarActuator &actuator = 
            _model.updComponent<ScalarActuator>(_non_muscle_actuator_path[m]);
        actuator.overrideActuation(state, true);
        double force = _optimal_force[j] * parameters[j];
        actuator.setOverrideActuation(state,force);
        j++;
    }

    //Set Secondary Kinematics to Optimized
    for (int m = 0; m < _n_secondary_coord; ++m) {
        Coordinate& coord = 
            _model.updComponent<Coordinate>(_secondary_coord_path[m]);

        double value = parameters(_n_actuators + m) + coord.getValue(state);
        coord.setValue(state, value, false);

        double speed = (value - _prev_secondary_value(m)) / _dt;
        coord.setSpeedValue(state, speed);
    }
    _model.assemble(state);
 }

void COMAKTool::initializeResultsStorage() {

    std::vector<std::string> actuator_names;
    
    for (int m = 0; m < _n_muscles; ++m) {
        actuator_names.push_back(_muscle_path[m]);
    }
    for (int m = 0; m < _n_non_muscle_actuators; ++m) {
        actuator_names.push_back(_non_muscle_actuator_path[m]);
    }

    _result_activations.setColumnLabels(actuator_names);
    _result_forces.setColumnLabels(actuator_names);

    std::vector<std::string> kinematics_names;
    std::vector<std::string> values_names;

    for (Coordinate &coord : _model.updComponentList<Coordinate>()) {
        std::string name = coord.getAbsolutePathString();
        kinematics_names.push_back(name + "/value");
        kinematics_names.push_back(name + "/speed");
        kinematics_names.push_back(name + "/acc");

        values_names.push_back(coord.getName());
    }
    _result_kinematics.setColumnLabels(kinematics_names);
    _result_values.setColumnLabels(values_names);
}

void COMAKTool::recordResultsStorage(const SimTK::State& state, int frame) {
    if (frame == 0) {
        _model.updAnalysisSet().begin(state);
    }
    else {
        _model.updAnalysisSet().step(state, frame);
    }

    _result_states.append(state);

    SimTK::RowVector activations(_n_actuators);
    SimTK::RowVector forces(_n_actuators);

    for (int m = 0; m < _n_actuators; ++m) {
        activations(m) = _optim_parameters(m);
        forces(m) = _optim_parameters(m)*_optimal_force(m);
    }

    _result_activations.appendRow(_time[frame], activations);
    _result_forces.appendRow(_time[frame], forces);

    SimTK::RowVector kinematics(_model.getNumCoordinates() * 3);
    SimTK::RowVector values(_model.getNumCoordinates());

    int k = 0;
    int v = 0;
    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        kinematics(k) = coord.getValue(state);
        kinematics(k + 1) = coord.getSpeedValue(state);
        kinematics(k + 2) = coord.getAccelerationValue(state);
        k = k + 3;

        values(v) = coord.getValue(state);
        v++;
    }
    _result_kinematics.appendRow(_time[frame], kinematics);
    _result_values.appendRow(_time[frame], values);

}

void COMAKTool::printResultsFiles() {
    int makeDir_out = IO::makeDir(get_results_directory());
    if (errno == ENOENT && makeDir_out == -1) {
        OPENSIM_THROW(Exception, "Could not create " +
            get_results_directory() + "Possible reason: This tool cannot "
            "make new folder with subfolder.");
    }

    STOFileAdapter sto;
    TimeSeriesTable states_table = _result_states.exportToTable(_model);
    states_table.addTableMetaData("header", std::string("COMAK Model States"));
    states_table.addTableMetaData("nRows",
        std::to_string(states_table.getNumRows()));
    states_table.addTableMetaData("nColumns",
        std::to_string(states_table.getNumColumns() + 1));
    sto.write(states_table, get_results_directory() + "/"
        + get_results_prefix() + "_states.sto");

    _result_activations.addTableMetaData("header",
        std::string("COMAK Actuator Activations"));
    _result_activations.addTableMetaData("nRows",
        std::to_string(_result_activations.getNumRows()));
    _result_activations.addTableMetaData("nColumns",
        std::to_string( _result_activations.getNumColumns() + 1));

    sto.write(_result_activations, get_results_directory() + "/"
        + get_results_prefix() + "_activation.sto");

    _result_forces.addTableMetaData("header", 
        std::string("COMAK Actuator Forces"));
    _result_forces.addTableMetaData("nRows", 
        std::to_string(_result_forces.getNumRows()));
    _result_forces.addTableMetaData("nColumns", 
        std::to_string(_result_forces.getNumColumns() + 1));

    sto.write(_result_forces, get_results_directory() + "/"
        + get_results_prefix() + "_force.sto");

    _result_kinematics.addTableMetaData("inDegrees", std::string("no"));
    _model.getSimbodyEngine().convertRadiansToDegrees(_result_kinematics);
    _result_kinematics.addTableMetaData("header", 
        std::string("COMAK Model Kinematics"));
    _result_kinematics.addTableMetaData("nRows", 
        std::to_string(_result_kinematics.getNumRows()));
    _result_kinematics.addTableMetaData("nColumns", 
        std::to_string(_result_kinematics.getNumColumns() + 1));

    sto.write(_result_kinematics, get_results_directory() + "/" 
        + get_results_prefix() + "_kinematics.sto");

    _result_values.addTableMetaData("inDegrees", std::string("no"));
    _model.getSimbodyEngine().convertRadiansToDegrees(_result_values);
    _result_values.addTableMetaData("header", 
        std::string("COMAK Model Values"));
    _result_values.addTableMetaData("nRows", 
        std::to_string(_result_values.getNumRows()));
    _result_values.addTableMetaData("nColumns", 
        std::to_string(_result_values.getNumColumns() + 1));

    sto.write(_result_values, get_results_directory() + "/" 
        + get_results_prefix() + "_values.sto");
}

SimTK::Vector COMAKTool::equilibriateSecondaryCoordinates() 
{
    Model settle_model = _model;
    SimTK::State state = settle_model.initSystem();

    std::cout << std::endl;
    std::cout << 
        "----------------------------------------"
        "----------------------------------------"
        << std::endl;
    std::cout << 
        "Performing forward simulation to equilibriate secondary kinematics" 
        << std::endl;
    std::cout << 
        "----------------------------------------"
        "----------------------------------------"
        << std::endl;

    for (Muscle& msl : settle_model.updComponentList<Muscle>()) {
        if (msl.getConcreteClassName() == "Millard2012EquilibriumMuscle") {
            msl.set_ignore_activation_dynamics(true);
            msl.set_ignore_tendon_compliance(true);
        }
    }

    if (get_use_visualizer()) {
        settle_model.setUseVisualizer(true);
    }

    state = settle_model.initSystem();
    
    if (get_use_visualizer()) {
        SimTK::Visualizer& viz = 
            settle_model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundColor(SimTK::White);
        viz.setShowSimTime(true);
    }

    // Set initial pose and Lock all coordinates except secondary 
    int nCoord = 0;
    for (Coordinate& coord : settle_model.updComponentList<Coordinate>()) {
        coord.setValue(state, _q_matrix(_start_frame, nCoord), false);
        coord.setSpeedValue(state, _u_matrix(_start_frame, nCoord));
        
        bool is_secondary =
            _secondary_coord_path.findIndex(coord.getAbsolutePathString()) >-1;

        if (is_secondary) {
            coord.setLocked(state, false);
        }
        else {
            coord.setLocked(state, true);
        }
        nCoord++;
    }

    // Don't lock coordinates that are constrained by 
    // CoordinateCouplerConstraint
    for(auto& cc_const : 
        settle_model.updComponentList<CoordinateCouplerConstraint>()){

        std::string cc_coord_name = cc_const.getDependentCoordinateName();
        Coordinate& coord = settle_model.updCoordinateSet().get(cc_coord_name);

        if (cc_const.get_isEnforced()) {
            coord.setLocked(state, false);
        }

    }

    settle_model.assemble(state);
    settle_model.realizeVelocity(state);

    // Prescribe Muscle Force
    for (Muscle& msl : settle_model.updComponentList<Muscle>()) {
        msl.overrideActuation(state, true);
        double value = msl.getMaxIsometricForce()*0.02;
        msl.setOverrideActuation(state, value);
    }

    StatesTrajectory result_states;

    // Store Secondary Coordinate Values (to check if simulation is settled)
    SimTK::Vector prev_sec_coord_value(_n_secondary_coord);

    for (int k = 0; k < _n_secondary_coord; k++) {
        Coordinate& coord = 
            settle_model.updComponent<Coordinate>(_secondary_coord_path[k]);
        prev_sec_coord_value(k) = coord.getValue(state);
    }

    // Perform settling simulation
    SimTK::CPodesIntegrator integrator(settle_model.getSystem(),
        SimTK::CPodes::BDF, SimTK::CPodes::Newton);

    integrator.setAccuracy(get_settle_accuracy());
    integrator.setInternalStepLimit(get_settle_internal_step_limit());
    SimTK::TimeStepper timestepper(settle_model.getSystem(), integrator);
    timestepper.initialize(state);
    
    double dt = 0.01;

    double max_coord_delta = SimTK::Infinity;
    int i = 1;
    while (max_coord_delta > get_settle_threshold()){
        timestepper.stepTo(i*dt);
        state = timestepper.getState();
        result_states.append(state);
        
        if (get_verbose() > 0) {
            std::cout << std::endl;
            std::cout << "Time: " << state.getTime() << std::endl;
            std::cout << std::setw(15) << "Secondary Coord" 
                << std::setw(15) << "VALUE" 
                << std::setw(15) << "DELTA" << std::endl;
        }

        //Compute Delta Coordinate
        max_coord_delta = 0;
        for (int k = 0; k < _n_secondary_coord; k++) {
            Coordinate& coord = 
                settle_model.updComponent<Coordinate>(
                    _secondary_coord_path[k]);

            double value = coord.getValue(state);
            double delta = abs(value - prev_sec_coord_value(k));
            
            if (delta > max_coord_delta) {
                max_coord_delta = delta;
            }
            prev_sec_coord_value(k) = value;

            if (get_verbose() > 0) {
                std::cout << std::setw(15) << coord.getName() 
                    << std::setw(15) << value 
                    << std::setw(15) << delta <<std::endl;
            }
        }
        i++;
    }
    
    //Print Results
    if (get_print_settle_sim_results()) {
        TimeSeriesTable states_table = 
            result_states.exportToTable(settle_model);
        states_table.addTableMetaData(
            "header", std::string("COMAK Settle Simulation States"));
        states_table.addTableMetaData(
            "nRows", std::to_string(states_table.getNumRows()));
        states_table.addTableMetaData(
            "nColumns", std::to_string(states_table.getNumColumns()+1));
        states_table.addTableMetaData("inDegrees", std::string("no"));

        int makeDir_out = IO::makeDir(get_settle_sim_results_directory());
        if (errno == ENOENT && makeDir_out == -1) {
            OPENSIM_THROW(Exception, "Could not create " +
                get_settle_sim_results_directory() +
                "Possible reason: This tool cannot make new "
                "folder with subfolder.");
        }

        std::string basefile = get_settle_sim_results_directory() + 
            "/" + get_settle_sim_results_prefix();

        STOFileAdapter sto;
        sto.write(states_table, basefile + "_states.sto");
    }

    //Collect Settled Secondary Q values;
    SimTK::Vector secondary_q(_n_secondary_coord);
    for (int i = 0; i < _n_secondary_coord; ++i) {
        Coordinate& coord = settle_model.updComponent<Coordinate>(
            _secondary_coord_path[i]);
        
        secondary_q(i) = coord.getValue(state);
    }
    return secondary_q;
}

void COMAKTool::extractKinematicsFromFile() {

    Storage store(get_coordinates_file());
    Array<std::string> col_names = store.getColumnLabels();

    //Set Start and Stop Times
    Array<double> in_time;
    store.getTimeColumn(in_time);

    if (get_start_time() == -1) {
        set_start_time(in_time.get(0));
    }
    if (get_stop_time() == -1) {
        set_stop_time(in_time.getLast());
    }

    if (store.isInDegrees()) {
        _model.getSimbodyEngine().convertDegreesToRadians(store);
    }

    if (get_time_step() != -1) {
        store.resampleLinear(get_time_step());
    }

    if (get_lowpass_filter_frequency() != -1) {
        store.pad(store.getSize() / 2);
        store.lowpassIIR(get_lowpass_filter_frequency());
    }

    store.getTimeColumn(_time);
    _dt = _time[1] - _time[0];

    //Set number of Frames
    _n_frames = _time.size();
    _n_out_frames = 0;
    bool first_frame = true;

    for (int i = 0; i < _n_frames; ++i) {
        if (_time[i] < get_start_time()) { continue; }
        if (_time[i] > get_stop_time()) { break; };

        if (first_frame) {
            _start_frame=i;
            first_frame = false;
        }

        _n_out_frames++;
    }

    //Gather Q and U values
    Array<std::string> col_labels = store.getColumnLabels();

    std::vector<int> q_col_map(_model.getNumCoordinates(),-1);
    //q_col_map = -1;

    for (int i = 0; i < col_labels.size(); ++i) {
        std::vector<std::string> split_label = 
            split_string(col_labels[i], "/");

        int j = 0;
        for (const Coordinate& coord : _model.getComponentList<Coordinate>()) {
            if (contains_string(split_label, coord.getName())) {
                if (split_label.back() == "value" || 
                    split_label.back() == coord.getName()) {
                    q_col_map[j] = i;
                }
            }
            j++;
        }

    }

    _q_matrix.resize(_n_frames, _model.getNumCoordinates());
    _u_matrix.resize(_n_frames, _model.getNumCoordinates());
    _udot_matrix.resize(_n_frames, _model.getNumCoordinates());

    _q_matrix = 0;
    _u_matrix = 0;
    _udot_matrix = 0;

    int j = 0;
    for (const Coordinate& coord : _model.getComponentList<Coordinate>()) {

        if (q_col_map[j] != -1) {
            Array<double> data;
            store.getDataColumn(col_labels[q_col_map[j]], data, _time[0]);
            for (int i = 0; i < _n_frames; ++i) {
                _q_matrix(i, j) = data[i];
            }
        }
        else {
            std::cout << "Coordinate Value: " << coord.getName() << 
                " not found in coordinates_file, assuming 0." << std::endl;
        }

        //GCVSpline q_spline;
        //q_spline.setDegree(5);

        SimmSpline q_spline;
        for (int i = 0; i < _n_frames; ++i) {
            q_spline.addPoint(_time[i], _q_matrix(i, j));
        }

        for (int i = 0; i < _n_frames; ++i) {
            SimTK::Vector x(1);
            x(0) = _time[i];

            std::vector<int> u_order = { 0 };
            _u_matrix(i, j) = q_spline.calcDerivative(u_order, x);

            std::vector<int> udot_order = { 0,0 };
            _udot_matrix(i, j) = q_spline.calcDerivative(udot_order, x);
        }
        j++;
    }

    if (get_print_processed_input_kinematics()) {

        std::vector<double> time_vec;
        for (int i = 0; i < _time.size(); ++i) {
            time_vec.push_back(_time[i]);
        }
        
        std::vector<std::string> labels;
        int numCoordinates = _model.countNumComponents<Coordinate>();

        SimTK::Matrix processed_kinematics(_time.size(), numCoordinates * 3);

        int nCol = 0;
        for (auto coord : _model.getComponentList<Coordinate>()) {
            labels.push_back(coord.getAbsolutePathString() + "/value");
            processed_kinematics(nCol) = _q_matrix(nCol);
            nCol++;
        }

        for (auto coord : _model.getComponentList<Coordinate>()) {
            labels.push_back(coord.getAbsolutePathString() + "/speed");
            processed_kinematics(nCol) = _u_matrix(nCol);
            nCol++;
        }

        for (auto coord : _model.getComponentList<Coordinate>()) {
            labels.push_back(coord.getAbsolutePathString() + "/acceleration");
            processed_kinematics(nCol) = _udot_matrix(nCol);
            nCol++;
        }
        
        TimeSeriesTable kin_table = TimeSeriesTable(
            time_vec, processed_kinematics, labels);

        kin_table.addTableMetaData("header", std::string("processed_input_q"));
        kin_table.addTableMetaData("inDegrees", std::string("no"));
        kin_table.addTableMetaData("nColumns", 
            std::to_string(kin_table.getNumColumns() + 1));
        kin_table.addTableMetaData("nRows", 
            std::to_string(kin_table.getNumRows()));

        STOFileAdapter sto;
        sto.write(kin_table, get_results_directory() + "/" + 
            get_results_prefix() + "processed_input_kinematics.sto");
       
        std::cout << std::endl;
        std::cout << "Printed processed input kinematics to results_dir: " + 
            get_results_directory() << std::endl;
    }
}

void COMAKTool::applyExternalLoads()
{
    const std::string& aExternalLoadsFileName = 
        SimTK::Pathname::getAbsolutePathname(get_external_loads_file());

    if (aExternalLoadsFileName == "" || 
        aExternalLoadsFileName == "Unassigned") {

        std::cout << "No external loads will be applied "
            "(external loads file not specified)." << std::endl;
        return;
    }

    // This is required so that the references to other files inside 
    // ExternalLoads file are interpreted as relative paths
    std::string savedCwd = IO::getCwd();
    IO::chDir(IO::getParentDirectory(aExternalLoadsFileName));

    // Create external forces
    ExternalLoads* externalLoads = nullptr;
    try {
        externalLoads = new ExternalLoads(aExternalLoadsFileName, true);

        _model.addModelComponent(externalLoads);
    }
    catch (const Exception &ex) {
        // Important to catch exceptions here so we can restore current 
        // working directory...
        // And then we can re-throw the exception
        std::cout << "Error: failed to construct ExternalLoads from file "
            << aExternalLoadsFileName;
        std::cout << ". Please make sure the file exists and that it "
            "contains an ExternalLoads";
        std::cout << "object or create a fresh one." << std::endl;
        
        if (getDocument()) IO::chDir(savedCwd);
        throw(ex);
    }

    // copy over created external loads to the external loads owned by the tool
    _external_loads = *externalLoads;

    IO::chDir(savedCwd);
    return;
}

void COMAKTool::updateModelForces()
{
    // If replacing force set read in from model file, clear it here
    if (get_replace_force_set()){
        // Can no longer just remove the model's forces.
        // If the model is connected, then the model will
        // maintain a list of subcomponents that refer to garbage.
        _model.cleanup();
        _model.updForceSet().setSize(0);
    }

    // Load force set(s)
    if (!get_force_set_file().empty()) {
        std::cout << "Adding force object set from " <<
            get_force_set_file() << std::endl;

        ForceSet *forceSet = new ForceSet(get_force_set_file(), true);
        _model.updForceSet().append(*forceSet);
    }
}

void COMAKTool::computeMuscleVolumes() {
    double max_iso_stress = 350000.0; //Muscle Specific Tension

    SimTK::Vector msl_volume(_model.getMuscles().getSize(), 0.0);
    
    int i = 0;
    for (const Muscle& msl : _model.getComponentList<Muscle>()) {
        double l0 = msl.get_optimal_fiber_length();
        double fmax = msl.get_max_isometric_force();

        msl_volume[i] = (l0*fmax) / max_iso_stress;
        i++;
    }
    _muscle_volumes = msl_volume;

    _normalized_muscle_volumes = msl_volume/=msl_volume.normInf();
}

void COMAKTool::printCOMAKascii() {
    std::cout <<
"#################################################################################################\n" 
"#################################################################################################\n"
"##                                                                                             ##\n"
"##    WWWWWW WWWW      WWWWWWW        WWWWWWW    WWWWWWWW       WWWWWWWWW    WWWWWWWW  WWWWWWW ##\n"
"##  WWWWWWWWWWWWW   WWWWWWWWWWWWW    WWWWWWWWW   WWWWWWWW        WWWWWWWWW    WWWWWWW  WWWWWWW ##\n"
"## WWWWWW    WWWW  WWWWWW   WWWWWW   WWWWW  WWWWWWW  WWWWW       WWWWWWWWW     WWWWWW  WWWW    ##\n"
"## WWWWWW          WWWWWW   WWWWWW   WWWWW  WWWWWWW  WWWWW       WWW  WWWWW    WWWWWWWWWWW     ##\n"
"## WWWWWW    WWW   WWWWWW   WWWWWW  WWWWWW   WWWWW   WWWWW      WWWWWWWWWWW    WWWWWW WWWWWW   ##\n"
"## WWWWWWW  WWWWW   WWWWWW WWWWWW   WWWWWW  WWWWWW   WWWWWW   WWWWW    WWWWWW WWWWWWW  WWWWWWW ##\n"
"##   WWWWWWWWWW       WWWWWWWWW    WWWWWWW WWWWWWWW  WWWWWWW  WWWWWW  WWWWWWW WWWWWWWW WWWWWWW ##\n"
"##                                                                                             ##\n"
"#################################################################################################\n"
"#################################################################################################\n"
"      ##           Concurrent Optimization of Muscle Activations and Kinematics          ##\n"
"      ##                                                                                 ##\n"
"      ##               Developed by Colin Smith [1,2] and Darryl Thelen [1]              ##\n"
"      ##                       1. University of Wisconsin-Madison                        ##\n"
"      ##                                   2. ETH Zurich                                 ##\n"
"      #####################################################################################\n"
"      #####################################################################################\n\n"

    << std::endl;
};

void COMAKTool::printOptimizationResultsToConsole(
    const SimTK::Vector& parameters, const SimTK::State& state) {
    int w = 20;

    if (get_verbose() > 1) {
        for (int k = 0; k < _n_secondary_coord; ++k) {
            if (parameters[_n_actuators + k] == _secondary_coord_max_change[k]) {
                std::cout << "Warning: " << _secondary_coord_name[k]
                    << " optimized to max change: " 
                    << _secondary_coord_max_change[k] << std::endl;
            }
        }
    }
    if (get_verbose() > 2) {
        std::cout << std::left << "\nOptimized Muscles:" << std::endl;
        std::cout << std::setw(w) << "name"
            << std::setw(w) << "activation"
            << std::setw(w) << "force" << std::endl;

        int p = 0;
        for (int k = 0; k < _n_muscles; ++k) {
            std::cout << std::setw(w) << _optim_parameter_names[p]
                << std::setw(w) << parameters[p]
                << std::setw(w) << parameters[p] * _optimal_force[p] << std::endl;
            p++;
        }

        std::cout << "\nOptimized Non Muscle Actuators:" << std::endl;
        std::cout << std::setw(w) << "name"
            << std::setw(w) << "activation"
            << std::setw(w) << "force" << std::endl;

        for (int k = 0; k < _n_non_muscle_actuators; ++k) {
            std::cout << std::setw(w) << _optim_parameter_names[p]
                << std::setw(w) << parameters[p]
                << std::setw(w) << parameters[p] * _optimal_force[p] << std::endl;
            p++;
        }

        std::cout << "\nOptimized Secondardy Coordinates:" << std::endl;
        std::cout << std::setw(w) << "name" << std::setw(w) << "value"
            << std::setw(w) << "change"
            << std::setw(w) << "damping force" << std::endl;

        for (int k = 0; k < _n_secondary_coord; ++k) {
            double value = _model.updComponent<Coordinate>
                (_secondary_coord_path[k]).getValue(state);

            std::cout << std::setw(w) << _secondary_coord_name[k]
                << std::setw(w) << value
                << std::setw(w) << parameters[p]

                << std::endl;
            p++;
        }
    }

    if (get_verbose() > 3) {
        _model.realizeReport(state);

        std::cout << "\nBlankevoort1991Ligaments:" << std::endl;
        std::cout << std::setw(w) << "Name"
            << std::setw(w) << "Total_Force"
            << std::setw(w) << "Spring_Force"
            << std::setw(w) << "Damping_Force"
            << std::setw(w) << "Lengths"
            << std::setw(w) << "Strains" << std::endl;

        for (auto& lig : _model.updComponentList<Blankevoort1991Ligament>()) {
            double total_frc = lig.getOutputValue<double>(state, "total_force");
            double spring_frc = lig.getOutputValue<double>(state, "spring_force");
            double damping_frc = lig.getOutputValue<double>(state, "damping_force");
            double length = lig.getOutputValue<double>(state, "length");
            double strain = lig.getOutputValue<double>(state, "strain");

            std::cout << std::setw(w) << lig.getName()
                << std::setw(w) << total_frc
                << std::setw(w) << spring_frc
                << std::setw(w) << damping_frc
                << std::setw(w) << length
                << std::setw(w) << strain << std::endl;
        }

        std::cout << "\nSmith2018ArticularContactForce: casting_mesh" << std::endl;
        std::cout << std::setw(w) << "Name"
            << std::setw(w) << "Num_Contacting_Tri"
            << std::setw(w) << "Contact_Force"
            << std::setw(w) << "Contact_Moment"
            << std::setw(w) << "Mean Pressure"
            << std::setw(w) << "Max Pressure"
            << std::setw(w) << "Mean Proximity"
            << std::setw(w) << "Max Proximity"
            << std::endl;

        for (auto& cnt :
            _model.updComponentList<Smith2018ArticularContactForce>()) {

            int num_cnt_tri =
                cnt.getOutputValue<int>(state, "casting_num_contacting_triangles");
            SimTK::Vec3 cnt_frc =
                cnt.getOutputValue<SimTK::Vec3>(state,
                    "casting_total_contact_force");
            SimTK::Vec3 cnt_moment =
                cnt.getOutputValue<SimTK::Vec3>(state,
                    "casting_total_contact_moment");

            double mean_prs =
                cnt.getOutputValue<double>(state, "casting_total_mean_pressure");
            double max_prs =
                cnt.getOutputValue<double>(state, "casting_total_max_pressure");
            double mean_prx =
                cnt.getOutputValue<double>(state, "casting_total_mean_proximity");
            double max_prx =
                cnt.getOutputValue<double>(state, "casting_total_max_proximity");

            std::cout << std::setw(w) << cnt.getName()
                << std::setw(w) << num_cnt_tri
                << std::setw(w) << cnt_frc
                << std::setw(w) << cnt_moment
                << std::setw(w) << mean_prs
                << std::setw(w) << max_prs
                << std::setw(w) << mean_prx
                << std::setw(w) << max_prx
                << std::endl;
        }
    }
}



