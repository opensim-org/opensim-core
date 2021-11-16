/* -------------------------------------------------------------------------- *
 *                                ForsimTool.cpp                              *
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


 //=============================================================================
 // INCLUDES
 //=============================================================================
#include <OpenSim/Simulation/Model/Model.h>
#include "ForsimTool.h"
#include "JAMUtilities.h"
#include <OpenSim/Common/Adapters.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Common/IO.h>
#include "OpenSim/Simulation/Model/Smith2018ArticularContactForce.h"
#include "OpenSim/Simulation/Model/Blankevoort1991Ligament.h"
#include <OpenSim/Common/SimmSpline.h>
#include "OpenSim/Simulation/StatesTrajectory.h"
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Simulation/Control/PrescribedController.h>
#include <OpenSim/Common/Stopwatch.h>
#include <OpenSim/Common/Reporter.h>
using namespace OpenSim;

ForsimTool::ForsimTool() : Object()
{
    setNull();
    constructProperties();
    _directoryOfSetupFile = "";
    _model_exists = false;
}

ForsimTool::ForsimTool(std::string settings_file) : Object(settings_file) {
    constructProperties();
    updateFromXMLDocument();

    //_directoryOfSetupFile = IO::getParentDirectory(settings_file);
    //IO::chDir(_directoryOfSetupFile);
}

void ForsimTool::setNull()
{
    setAuthors("Colin Smith");
}

void ForsimTool::constructProperties()
{
    constructProperty_model_file("");
    constructProperty_results_directory(".");
    constructProperty_results_file_basename("");
    constructProperty_start_time(0.0);
    constructProperty_stop_time(1.0);
    constructProperty_minimum_time_step(0.0);
    constructProperty_maximum_time_step(0.1);
    constructProperty_report_time_step(0.01);
    constructProperty_integrator_accuracy(1e-6);
    constructProperty_internal_step_limit(-1);
    constructProperty_constant_muscle_control(0.02);
    constructProperty_use_activation_dynamics(true);
    constructProperty_use_tendon_compliance(true);
    constructProperty_use_muscle_physiology(true);
    constructProperty_equilibrate_muscles(true);
    constructProperty_unconstrained_coordinates();
    constructProperty_actuator_input_file("");
    constructProperty_external_loads_file("");
    constructProperty_prescribed_coordinates_file("");
    constructProperty_geometry_folder("");
    constructProperty_use_visualizer(false);
    constructProperty_AnalysisSet(AnalysisSet());
}

void ForsimTool::setModel(Model& aModel)
{
    if (!get_geometry_folder().empty()) {
        ModelVisualizer::addDirToGeometrySearchPaths(get_geometry_folder());
    }
    
    _model = aModel;
    set_model_file(aModel.getDocumentFileName());
    _model_exists = true;
}

bool ForsimTool::run()
{
    bool completed = false;

    auto cwd = IO::CwdChanger::changeToParentOf(getDocumentFileName());

    try {
        // Clear Previous Results
        _result_states.clear();

        const Stopwatch stopwatch;
        log_critical("");
        log_critical("==========");
        log_critical("ForsimTool");
        log_critical("==========");
        log_critical("");

        //Make results directory
        int makeDir_out = IO::makeDir(get_results_directory());
        if (errno == ENOENT && makeDir_out == -1) {
            OPENSIM_THROW(Exception, "Could not create " +
                get_results_directory() +
                "Possible reason: This tool cannot make new folder with subfolder.");
        }

        //Set Model
        if (!get_geometry_folder().empty()) {
            ModelVisualizer::addDirToGeometrySearchPaths(get_geometry_folder());
        }

        if (!_model_exists) {
            if (get_model_file().empty()) {
                OPENSIM_THROW(Exception, "No model was set in the ForsimTool.");
            }
            _model = Model(get_model_file());
        }

        _model.initSystem();        
        
        initializeActuators();

        initializeCoordinates();

        initializeStartStopTimes();

        applyExternalLoads();

        //Add Analysis set
        AnalysisSet aSet = get_AnalysisSet();
        int size = aSet.getSize();

        for (int i = 0; i < size; i++) {
            Analysis *analysis = aSet.get(i).clone();
            _model.addAnalysis(analysis);
        }

        if (get_use_visualizer()) {
            _model.setUseVisualizer(true);
        }

        SimTK::State state = _model.initSystem();
        
        for (const auto& mesh : _model.updComponentList<Smith2018ContactMesh>()) {
            mesh.printMeshDebugInfo();
        }        

        // Initialize Muscle States
        for (Muscle& msl : _model.updComponentList<Muscle>()) {
            std::string msl_path = msl.getAbsolutePathString();

            if (contains_string(_prescribed_frc_actuator_paths, msl_path)) {                
                msl.overrideActuation(state, true);
                msl.setOverrideActuation(state, 0.0);

                continue;
            }
            if (contains_string(_prescribed_act_actuator_paths, msl_path)) {
                continue;
            }
            if (contains_string(_prescribed_control_actuator_paths, msl_path)) {
                continue;
            }

            if (get_constant_muscle_control() > -1) {
                if (!get_use_muscle_physiology ()) {
                    msl.overrideActuation(state, true);
                    /*msl.setOverrideActuation(
                            state, get_constant_muscle_control() *
                                           msl.getMaxIsometricForce()); */
                }
            }
        }

        if (get_equilibrate_muscles()) { _model.equilibrateMuscles(state); }

        AnalysisSet& analysisSet = _model.updAnalysisSet();

        //Setup Visualizer
        if (get_use_visualizer()) {
            _model.updMatterSubsystem().setShowDefaultGeometry(false);
            SimTK::Visualizer& viz = _model.updVisualizer().updSimbodyVisualizer();
            viz.setBackgroundColor(SimTK::Black);
            viz.setBackgroundType(SimTK::Visualizer::BackgroundType::SolidColor);
            viz.setMode(SimTK::Visualizer::Mode::Sampling);
            viz.setShowSimTime(true);
            viz.setDesiredFrameRate(100);
        }

        //Setup Integrator
        state.setTime(get_start_time());
    
        SimTK::CPodesIntegrator integrator(_model.getSystem(), SimTK::CPodes::BDF, SimTK::CPodes::Newton);
        integrator.setAccuracy(get_integrator_accuracy());
        integrator.setMinimumStepSize(get_minimum_time_step());
        integrator.setMaximumStepSize(get_maximum_time_step());
        if (get_internal_step_limit()>0) {
            integrator.setInternalStepLimit(get_internal_step_limit());
        }
        SimTK::TimeStepper timestepper(_model.getSystem(), integrator);
        timestepper.initialize(state);
    
       // Record Initial State
        log_debug("Initial State:");
        _model.realizeReport(state);
        printDebugInfo(state);
        analysisSet.begin(state);
        
        _result_states.append(state);

        // Integrate Forward in Time
        double dt = get_report_time_step();
        int nSteps = (int)lround((get_stop_time() - get_start_time()) / dt);

        log_info("start time: {}", get_start_time());
        log_info("stop time: {}", get_stop_time());

        for (int i = 0; i <= nSteps; ++i) {
            
            double t = get_start_time() + (i+1) * dt;
            log_info("time: {}", t);
            
            //Set Prescribed Muscle Forces
            if(_prescribed_frc_actuator_paths.size() > 0){
                for (int j = 0; j < (int)_prescribed_frc_actuator_paths.size();++j) {
                    std::string actuator_path = _prescribed_frc_actuator_paths[j];
                    ScalarActuator& actuator = _model.updComponent<ScalarActuator>(actuator_path);
                    double value = _frc_functions.get(actuator_path +"_frc").calcValue(SimTK::Vector(1,t));
                    actuator.setOverrideActuation(state, value);
                }
                timestepper.initialize(state);
            }

            timestepper.stepTo(t);

            state = timestepper.getState();
            _model.realizeReport(state);
            printDebugInfo(state);

            // Record Result
            /* if (i == 0) {
                analysisSet.begin(state);
            }
            else {*/
                analysisSet.step(state, i);
            //}
            _result_states.append(state);            
        }

        printResults();

        const long long elapsed = stopwatch.getElapsedTimeInNs();

        log_info("\nForsim Tool complete.");
        log_info("Finished in {}", stopwatch.formatNs(elapsed));        
        log_info("");

        completed = true;
    }

    catch(const std::exception& x) {
        log_error("ForsimTool::run() caught an exception: \n {}", x.what());
        cwd.restore();
    }
    catch (...) { // e.g. may get InterruptedException
        log_error("ForsimTool::run() caught an exception.");
        cwd.restore();
    }

    cwd.restore();

    return completed; 
}

void ForsimTool::initializeStartStopTimes() {
    if (get_start_time() != -1 && get_stop_time() != -1) {
        return;
    }

    double start = 0;
    double end = 0;
    double act_start = -2;
    double act_end = -2;
    double coord_start = -2;
    double coord_end = -2;

    if (get_actuator_input_file() != "") {
        std::vector<double> act_time = _actuator_table.getIndependentColumn();
        act_start = act_time[0];
        act_end = act_time.back();
    }
    if (get_prescribed_coordinates_file() != "") {
        std::vector<double> coord_time = _coord_table.getIndependentColumn();
        coord_start = coord_time[0];
        coord_end = coord_time.back();
    }

    if (act_start == -2 && coord_start == -2) {
        OPENSIM_THROW(Exception, "No actuator_input_file or "
        "prescribed_coordinates_files defined." 
        "You must set start_time and stop_time to non-negative values.")
    }

    if (act_start != -2 && coord_start != -2) {
        if (act_start != coord_start || act_end != coord_end) {
            OPENSIM_THROW(Exception, "The start and stop times of the "
                "actuator_input_file and prescribed coordinate files do not match."
                "You must set start_time and stop_time to non-negative values.")
        }
    }

    if (get_start_time() == -1) {
        if (coord_start != -2)
            set_start_time(coord_start);
        else
            set_start_time(act_start);;
    }
    if (get_stop_time() == -1) {
        if (coord_end != -2)
            set_stop_time(coord_end);
        else
            set_stop_time(act_end);;
    }
}

void ForsimTool::initializeActuators() {
    PrescribedController* control = new PrescribedController();

    if (get_actuator_input_file() != ""){
        STOFileAdapter actuator_file;
        _actuator_table = TimeSeriesTable(get_actuator_input_file());

        int nDataPt = (int)_actuator_table.getNumRows();
        std::vector<std::string> labels = _actuator_table.getColumnLabels();
        std::vector<double> time = _actuator_table.getIndependentColumn();
        
        
        for (int i = 0; i < (int)labels.size(); ++i) {
            std::vector<std::string> split_label = split_string(labels[i], "_");

            //Control Prescribed
            if (split_label.back() == "control") {
                std::string actuator_path = erase_sub_string(labels[i], "_control");

                for (const ScalarActuator& actuator : _model.updComponentList<ScalarActuator>()) {
                    if (actuator.getName() == actuator_path) {
                        actuator_path = actuator.getAbsolutePathString();
                    }
                }

                try {
                    ScalarActuator& actuator = _model.updComponent<ScalarActuator>(actuator_path);
                    _prescribed_control_actuator_paths.push_back(actuator_path);
                    SimTK::Vector values = _actuator_table.getDependentColumn(labels[i]);

                    SimmSpline* control_function = new SimmSpline(nDataPt, &time[0], &values[0], actuator_path + "_control");

                    control->addActuator(actuator);

                    control->prescribeControlForActuator(actuator.getName(), control_function);                    
                }
                catch (ComponentNotFoundOnSpecifiedPath) {
                    OPENSIM_THROW(Exception,
                        "Actuator: " + actuator_path + " not found in model. "
                        "Did you use absolute path?")
                }

            }

            //Activation Prescribed
            if (split_label.back() == "activation") {
                std::string actuator_path = erase_sub_string(labels[i], "_activation");
                
                for (const ScalarActuator& actuator : _model.updComponentList<ScalarActuator>()) {
                    if (actuator.getName() == actuator_path) {
                        actuator_path = actuator.getAbsolutePathString();
                    }
                }

                try {
                    Millard2012EquilibriumMuscle& msl = _model.updComponent<Millard2012EquilibriumMuscle>(actuator_path);

                    _prescribed_act_actuator_paths.push_back(actuator_path);

                    SimTK::Vector values = _actuator_table.getDependentColumn(labels[i]);
                    SimmSpline* act_function = new SimmSpline(nDataPt, &time[0], &values[0], actuator_path + "_act");

                    control->addActuator(msl);

                    control->prescribeControlForActuator(msl.getName(), act_function);

                    msl.set_ignore_activation_dynamics(true);
                }
                catch (ComponentNotFoundOnSpecifiedPath) {
                    OPENSIM_THROW(Exception,
                        "Muscle: " + actuator_path + " not found in model. "
                        "Did you use absolute path? Is it a Millard2012EquilibriumMuscle?")
                }
            }

            //Force Prescribed
            if (split_label.back() == "force") {
                std::string actuator_path = erase_sub_string(labels[i], "_force");

                for (const ScalarActuator& actuator : _model.updComponentList<ScalarActuator>()) {
                    if (actuator.getName() == actuator_path) {
                        actuator_path = actuator.getAbsolutePathString();
                    }
                }

                try {
                    ScalarActuator& actuator = _model.updComponent<ScalarActuator>(actuator_path);
                    _prescribed_frc_actuator_paths.push_back(actuator_path);
                    SimTK::Vector values = _actuator_table.getDependentColumn(labels[i]);
                    SimmSpline* frc_function = new SimmSpline(nDataPt, &time[0], &values[0], actuator_path + "_frc");

                    _frc_functions.adoptAndAppend(frc_function); 
                }
                catch (ComponentNotFoundOnSpecifiedPath) {
                    
                    OPENSIM_THROW(Exception,
                        "Actuator: " + actuator_path + " not found in model. "
                        "Did you use absolute path?")
                }
            }

            
        }

        //Output to Screen
        if (_prescribed_frc_actuator_paths.size() > 0) {
            
            log_info("Force Prescribed:");
            for (std::string& name : _prescribed_frc_actuator_paths) {
                log_info("{}", name);
            }
            log_info("");
        }

        if (_prescribed_act_actuator_paths.size() > 0) {
            log_info("Activation Prescribed:");
            for (std::string& name : _prescribed_act_actuator_paths) {
                log_info("{}", name);
            }
            log_info("");
        }

        if (_prescribed_control_actuator_paths.size() > 0) {
            log_info("Control Prescribed:");
            for (std::string& name : _prescribed_control_actuator_paths) {
                log_info("{}", name);
            }
            log_info("");
        }
    }

    // Setup Results Reporters
    const auto act_reporterExists =
            _model.findComponent<TableReporter>("activations");

    TableReporter* report_activations = nullptr;
    if (act_reporterExists == nullptr) {
        
        report_activations = new TableReporter();
        report_activations->setName("activations");

    } else {
        report_activations = &_model.updComponent<TableReporter>("activations");
        report_activations->clearTable();
        report_activations->clearConnections();
    }

    int num_activation_msl = 0;

    const auto frc_reporterExists =
        _model.findComponent<TableReporter>("forces");

    TableReporter* report_forces = nullptr;
    if (frc_reporterExists == nullptr) {

        report_forces = new TableReporter();
        report_forces->setName("forces");

    } else {
        report_forces = &_model.updComponent<TableReporter>("forces");
        report_forces->clearTable();
        report_forces->clearConnections();
    }

    int num_force_msl = 0;

    //Setup Constant Muscle Control    
    log_info("use_activation_dynamics: {}", get_use_activation_dynamics());
    log_info("use_tendon_compliance: {}", get_use_tendon_compliance());
    log_info("use_muscle_physiology : {}", get_use_muscle_physiology ());
    log_info("");

    if (get_constant_muscle_control() > -1) {
        log_info("Constant Muscle Control: {}", get_constant_muscle_control());
    }

    for (Muscle& msl : _model.updComponentList<Muscle>()) {
        std::string msl_path = msl.getAbsolutePathString();

        if (contains_string(_prescribed_frc_actuator_paths, msl_path)) {
            report_forces->addToReport(msl.getOutput("actuation"));
            num_force_msl++;
            continue;
        }
        if (contains_string(_prescribed_act_actuator_paths, msl_path)) {
            report_activations->addToReport(msl.getOutput("activation"));
            num_activation_msl++;
            continue;
        }
        if (contains_string(_prescribed_control_actuator_paths, msl_path)) {
            continue;
        }

        if (msl.getConcreteClassName() == "Millard2012EquilibriumMuscle") {
            msl.set_ignore_activation_dynamics(
                !get_use_activation_dynamics());

            if (!get_use_activation_dynamics() && get_use_muscle_physiology()) {
                num_activation_msl++;
                report_activations->addToReport(msl.getOutput("activation"));
            }

            if (!get_use_muscle_physiology ()) {
                num_force_msl++;
                report_forces->addToReport(msl.getOutput("actuation"));
            }

            msl.set_ignore_tendon_compliance(
                !get_use_tendon_compliance());
        }
        else {
            log_info("{} is not a "
                "Millard2012EquilibriumMuscle, ignore_activation_dynamics "
                "and ignore_tendon_compliance will have no effect.",
                msl.getName());
        }
            
        if (get_constant_muscle_control() > -1) {
            if (get_use_muscle_physiology ()) {
                _prescribed_control_actuator_paths.push_back(msl_path);

                Constant* control_function =
                        new Constant(get_constant_muscle_control());

                control_function->setName(msl_path + "_frc");

                control->addActuator(msl);
                control->prescribeControlForActuator(
                        msl.getName(), control_function);
            } 

            log_info("{}", msl_path);
        }
    }
    log_info("");

    _model.addComponent(control);

    if (num_activation_msl > 0) {     
        _model.addComponent(report_activations);
    } else {
        delete report_activations;
    }
    if (num_force_msl > 0) {
        _model.addComponent(report_forces);
    } else {
        delete report_forces;
    }
}

void ForsimTool::initializeCoordinates() {
    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        coord.set_locked(true);
    }

    log_info("Unconstrained Coordinates:");
    for (int i = 0; i < getProperty_unconstrained_coordinates().size(); ++i) {
        std::string coord_path = get_unconstrained_coordinates(i);

        try {
            Coordinate& coord = _model.updComponent<Coordinate>(coord_path);
            coord.set_locked(false);
            log_info(coord_path);
        }
        catch (ComponentNotFoundOnSpecifiedPath) {
            OPENSIM_THROW(Exception,
                "Unconstrained Coordinate: " + coord_path + "Not found in model."
                "Did you use absolute path?")
        }
    }
    log_info("");

    //Load prescribed coordinates file
    STOFileAdapter coord_file;
    if (get_prescribed_coordinates_file() != "") {

        std::string saveWorkingDirectory = IO::getCwd();
        IO::chDir(_directoryOfSetupFile);

        try {
            _coord_table = TimeSeriesTable(get_prescribed_coordinates_file());

        }
        catch (...) { // Properly restore current directory if an exception is thrown
            IO::chDir(saveWorkingDirectory);
            throw;
        }
        IO::chDir(saveWorkingDirectory);

        std::vector<std::string> labels = _coord_table.getColumnLabels();

        int nDataPt = (int)_coord_table.getNumRows();
        std::vector<double> time = _coord_table.getIndependentColumn();

        log_info("Prescribed Coordinates:");
        for (int i = 0; i < (int)labels.size(); ++i) {

            std::string coord_path = "";
            for (const Coordinate& coordinate : _model.updComponentList<Coordinate>()) {
                if (labels[i] == coordinate.getName() ||
                    labels[i] == coordinate.getAbsolutePathString()) {
                    coord_path = coordinate.getAbsolutePathString();
                    break;
                }
            }

            if (coord_path == ""){
                std::cout << "Warning: Column label: " + labels[i] +
                    "in the prescribed_coordinates_file is not a coordinate "
                    "in the model." << std::endl;
                continue;
            }
            Coordinate& coord = _model.updComponent<Coordinate>(coord_path);
            SimTK::Vector values = _coord_table.getDependentColumn(labels[i]);
            if (coord.getMotionType() == Coordinate::MotionType::Rotational) {
                values *= SimTK::Pi / 180;
            }

            SimmSpline function = SimmSpline(nDataPt, &time[0], &values[0], coord.getName() + "_prescribed");
            coord.set_prescribed(true);
            coord.set_prescribed_function(function);
            coord.set_locked(false);

            log_info(labels[i]);
        }
        log_info("");
    }
}

void ForsimTool::applyExternalLoads()
{
    const std::string& aExternalLoadsFileName = get_external_loads_file();

    if (aExternalLoadsFileName == "" || aExternalLoadsFileName == "Unassigned") {
        log_info(
            "No external loads will be applied (external_loads_file not specified).");                
        return;
    }

    // This is required so that the references to other files inside ExternalLoads file are interpreted 
    // as relative paths
    //std::string savedCwd = IO::getCwd();
    //IO::chDir(IO::getParentDirectory(aExternalLoadsFileName));

    // Create external forces
    ExternalLoads* externalLoads = nullptr;
    try {

        //externalLoads = new ExternalLoads(IO::GetFileNameFromURI( aExternalLoadsFileName), true);
        externalLoads = new ExternalLoads(
                aExternalLoadsFileName, true);
        _model.addModelComponent(externalLoads);
    }
    catch (const Exception &ex) {
        // Important to catch exceptions here so we can restore current working directory...
        // And then we can re-throw the exception
        log_error( "Error: failed to construct ExternalLoads from file {}"
            ". Please make sure the file exists and that it contains an ExternalLoads"
            "object or create a fresh one.", aExternalLoadsFileName);
        //if (getDocument()) IO::chDir(savedCwd);
        throw(ex);
    }

    // copy over created external loads to the external loads owned by the tool
    _external_loads = *externalLoads;

   // IO::chDir(savedCwd);
    return;
}

void ForsimTool::printResults() {
    STOFileAdapter sto;
    std::string basefile =
            get_results_directory() + "/" + get_results_file_basename();

    // States
    TimeSeriesTable states_table = _result_states.exportToTable(_model);
    states_table.addTableMetaData("header", std::string("States"));
    states_table.addTableMetaData(
            "nRows", std::to_string(states_table.getNumRows()));
    states_table.addTableMetaData(
            "nColumns", std::to_string(states_table.getNumColumns() + 1));
    states_table.addTableMetaData("inDegrees", std::string("no"));
    sto.write(states_table, basefile + "_states.sto");

    // Activations
    if (_model.hasComponent<TableReporter>("activations")) {
        TimeSeriesTable activations_table =
            _model.getComponent<TableReporter>("activations").getTable();
    
        std::vector<std::string> activation_labels =
                activations_table.getColumnLabels();
        for (std::string& label : activation_labels) {
            label = replace_string(label, "|activation", "");
        }
        activations_table.setColumnLabels(activation_labels);

        activations_table.addTableMetaData("header", std::string("Forsim Activations"));
        activations_table.addTableMetaData(
                "nRows", std::to_string(activations_table.getNumRows()));
        activations_table.addTableMetaData(
                "nColumns", std::to_string(activations_table.getNumColumns() + 1));
        activations_table.addTableMetaData("inDegrees", std::string("no"));
    
        sto.write(activations_table, basefile + "_activations.sto");
    }

    // Forces
    if (_model.hasComponent<TableReporter>("forces")) {
        TimeSeriesTable forces_table = 
            _model.getComponent<TableReporter>("forces").getTable();
    
        std::vector<std::string> frc_labels = forces_table.getColumnLabels();
        for (std::string& label : frc_labels) {
            label = replace_string(label, "|actuation", "");
        }
        forces_table.setColumnLabels(frc_labels);

        forces_table.addTableMetaData("header", std::string("Forsim Forces"));
        forces_table.addTableMetaData(
                "nRows", std::to_string(forces_table.getNumRows()));
        forces_table.addTableMetaData(
                "nColumns", std::to_string(forces_table.getNumColumns() + 1));
        forces_table.addTableMetaData("inDegrees", std::string("no"));
    
        sto.write(forces_table, basefile + "_forces.sto");
    }

    _model.updAnalysisSet().printResults(
            get_results_file_basename(), get_results_directory());

    log_info("Printed results to: {}", get_results_directory());
}

void ForsimTool::printDebugInfo(const SimTK::State& state) {
    
    //_model.realizeReport(state);

    //Unconstrained Coordinates
    log_debug("{:<30} {:<20} {:<20}",
        "Unconstrained Coordinate", "Value", "Speed");

    for (int i = 0; i < getProperty_unconstrained_coordinates().size(); ++i) {
        std::string coord_path = get_unconstrained_coordinates(i);
        const Coordinate& coord = _model.updComponent<Coordinate>(coord_path);
        log_debug("{:<30} {:<20} {:<20}", coord.getName(),
            coord.getValue(state), coord.getSpeedValue(state));
    }
    log_debug("");

    //Muscle
    log_debug("{:<20} {:<20} {:<20} {:<20} {:<20} {:<20} {:<20}", 
        "Muscle", "Control", "Activation", "Fiber-Length",
        "Force", "Active-frc-AT", "Passive-frc-AT");

    for (const Muscle& msl : _model.updComponentList<Muscle>()) {
        log_debug("{:<20} {:<20} {:<20} {:<20} {:<20} {:<20} {:<20}", 
            msl.getName(), msl.getControl(state),
            msl.getActivation(state), msl.getFiberLength(state),
             msl.getActuation(state), msl.getActiveFiberForceAlongTendon(state),
            msl.getPassiveFiberForceAlongTendon(state));
    }
    log_debug("");

    // Ligament 
    log_debug("{:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12}",
        "Ligament", "Total Force", "Spring Force", 
        "Damping Force", "Strain", "Strain Rate", "Length"); 

    for (const Blankevoort1991Ligament& lig : 
        _model.updComponentList<Blankevoort1991Ligament>()) {

        log_debug("{:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12}",
            lig.getName(),
            lig.getOutputValue<double>(state, "total_force"),
            lig.getOutputValue<double>(state, "spring_force"),
            lig.getOutputValue<double>(state, "damping_force"),
            lig.getOutputValue<double>(state, "strain"),
            lig.getOutputValue<double>(state, "strain_rate"),
            lig.getOutputValue<double>(state, "length"));
    }
    log_debug("");

    // Contact
    log_debug("{:<20} {:<30} {:<30}","Contact","Force","COP");

    for (Smith2018ArticularContactForce& cnt : 
        _model.updComponentList<Smith2018ArticularContactForce>()) {
            
        log_debug("{:<20} {:<30} {:<30}", cnt.getName(),
            cnt.getOutputValue<SimTK::Vec3>(state,
                "casting_total_contact_force"),
            cnt.getOutputValue<SimTK::Vec3>(state,
                "casting_total_center_of_pressure"));
    }
    log_debug("");
/*
    if (get_verbose() >= 3) {
        log_debug("{:<20} {:<20} {:<20}", "Contact " << std::setw(20)
            << "Force" << std::setw(w) << "COP" << std::endl;

        for (Smith2018ArticularContactForce& cnt : _model.updComponentList<Smith2018ArticularContactForce>()) {

            std::cout << std::setw(w) << cnt.getName()
                << std::setw(w) << cnt.getOutputValue<SimTK::Vector>(state, "casting_triangle_proximity")
                << std::setw(w) << cnt.getOutputValue<SimTK::Vector>(state, "casting_triangle_pressure")
                << std::endl;
        }
    }

    if (get_verbose() >= 2) {
        std::cout << "Press Any Key to Continue." << std::endl;
        std::cin.ignore();
    }*/
}