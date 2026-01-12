/* -------------------------------------------------------------------------- *
 *                     COMAKInverseKinematicsTool.cpp                         *
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
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Simulation/StatesTrajectory.h>
#include <OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h>

#include "COMAKInverseKinematicsTool.h"
#include "JAMUtilities.h"


#include <OpenSim/Common/Adapters.h>
#include <OpenSim/Common/SimmSpline.h>
#include "OpenSim/Common/Constant.h"
#include <OpenSim/Common/PolynomialFunction.h>
#include <OpenSim/Common/Stopwatch.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/GCVSplineSet.h>

#include <OpenSim/Tools/IKCoordinateTask.h>
#include <OpenSim/Analyses/Kinematics.h>

#include "OpenSim/Simulation/Model/Smith2018ArticularContactForce.h"
#include "OpenSim/Simulation/Model/Blankevoort1991Ligament.h"

using namespace OpenSim;
//using namespace SimTK;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================

//_____________________________________________________________________________
/**
 * Default constructor.
 */
COMAKInverseKinematicsTool::COMAKInverseKinematicsTool() 
{
    constructProperties();
    //_directoryOfSetupFile = "";
    _model_exists = false;
}

COMAKInverseKinematicsTool::COMAKInverseKinematicsTool(const std::string file) 
    : Object(file) {
    constructProperties();
    updateFromXMLDocument();
    _model_exists = false;
    //_directoryOfSetupFile = IO::getParentDirectory(file);
    //IO::chDir(_directoryOfSetupFile); 
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void COMAKInverseKinematicsTool::constructProperties()
{
    constructProperty_model_file("");
    constructProperty_results_directory("");
    constructProperty_results_prefix("");

    constructProperty_perform_secondary_constraint_sim(true);
    constructProperty_secondary_coordinates();
    constructProperty_secondary_coupled_coordinate("");
    constructProperty_secondary_constraint_sim_settle_threshold(1e-5);
    constructProperty_secondary_constraint_sim_sweep_time(1.0);
    constructProperty_secondary_coupled_coordinate_start_value(0.0);
    constructProperty_secondary_coupled_coordinate_stop_value(0.0);
    constructProperty_secondary_constraint_sim_integrator_accuracy(1e-6);
    constructProperty_secondary_constraint_sim_internal_step_limit(-1);
    constructProperty_constraint_function_num_interpolation_points(20);
    constructProperty_secondary_constraint_function_file(
        "secondary_coordinate_constraint_functions.xml");
    constructProperty_print_secondary_constraint_sim_results(false);

    constructProperty_perform_inverse_kinematics(true);
    constructProperty_IKTaskSet(IKTaskSet());
    constructProperty_marker_file("");
    constructProperty_coordinate_file("");
    constructProperty_output_motion_file("");
    constructProperty_ik_constraint_weight(SimTK::Infinity);
    constructProperty_ik_accuracy(1e-5);
    Array<double> range{SimTK::Infinity, 2};
    range[0] = -SimTK::Infinity; 
    constructProperty_time_range(range);
    constructProperty_report_errors(false);
    constructProperty_report_marker_locations(false);
    constructProperty_constrained_model_file("");
    constructProperty_geometry_folder("");
    constructProperty_use_visualizer(false);
}

void COMAKInverseKinematicsTool::setModel(Model& model) {
    if (!get_geometry_folder().empty()) {
        ModelVisualizer::addDirToGeometrySearchPaths(get_geometry_folder());
    }

    _model = model;
    set_model_file(model.getDocumentFileName());
    _model_exists = true;
}

bool COMAKInverseKinematicsTool::run()
{
    bool completed = false;
    
    auto cwd = IO::CwdChanger::changeToParentOf(getDocumentFileName());

    try {
        const Stopwatch stopwatch;
        log_critical("");
        log_critical("==========================");
        log_critical("COMAKInverseKinematicsTool");
        log_critical("==========================");
        log_critical("");

        initialize();
    
        //Secondary Constraint Simulation
        if (get_perform_secondary_constraint_sim()) {
            performIKSecondaryConstraintSimulation();
        }

        //Inverse Kinematics 
        if (get_perform_inverse_kinematics()) {
            performIK();
        }

        const long long elapsed = stopwatch.getElapsedTimeInNs();

        log_info("COMAKInverseKinematics complete.");
        log_info("Finished in {}", stopwatch.formatNs(elapsed));
        log_info("Printed results to: {}", get_results_directory());
        log_info("");

        completed = true;
    }

    catch(const std::exception& x) {
        log_error("COMAKInverseKinematicsTool::run() caught an exception: \n {}", x.what());
        cwd.restore();
    }
    catch (...) { // e.g. may get InterruptedException
        log_error("COMAKInverseKinematicsTool::run() caught an exception.");
        cwd.restore();
    }

    cwd.restore();

    return completed;
}

bool COMAKInverseKinematicsTool::initialize()
{
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
            OPENSIM_THROW(Exception, "No model was set in the COMAKInverseKinematicsTool.");
        }
        _model = Model(get_model_file());
    }

    std::string function_file = get_secondary_constraint_function_file();

    if (get_secondary_constraint_function_file() == "") {
        OPENSIM_THROW(Exception, "secondary_constraint_function file not set.")
    }
    
    _model.initSystem();

    //Verfiy Coordinate Properties    
    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        std::string name = coord.getName();
        std::string path = coord.getAbsolutePathString();

        //Reset to full path
        if (get_secondary_coupled_coordinate() == name) {
            set_secondary_coupled_coordinate(path);
        }

        int ind = getProperty_secondary_coordinates().findIndex(name);

        if (ind > -1) {
            set_secondary_coordinates(ind, path);
        }
    }

    //Make sure Coordinate exists in model and no duplicates
    std::string name = get_secondary_coupled_coordinate();
    try { _model.getComponent<Coordinate>(name); }
    catch (Exception const&) {
        OPENSIM_THROW(Exception, "secondary_coupled_coord: " + 
            name + " not found in model.")
    }
    
    for (int i = 0; i < getProperty_secondary_coordinates().size(); ++i) {
        std::string name = get_secondary_coordinates(i);
        try { _model.getComponent<Coordinate>(name); }
        catch (Exception const&){
            OPENSIM_THROW(Exception,"secondary_coordinate: " + 
                name + "not found in model.")
        }

        int n = 0;
        for (int j = 0; j < getProperty_secondary_coordinates().size(); ++j) {
            if (name == get_secondary_coordinates(j)) n++;
        }
        OPENSIM_THROW_IF(n>1, Exception, name + 
            "listed multiple times in secondary_coordinates")
    }

    //Count numbers
    _n_secondary_coord = getProperty_secondary_coordinates().size();

    _secondary_coord_name.setSize(_n_secondary_coord);
    _secondary_coord_path.setSize(_n_secondary_coord);
    _secondary_coord_index.setSize(_n_secondary_coord);


    for (int i = 0; i < _n_secondary_coord; ++i) {
        _secondary_coord_path[i] = get_secondary_coordinates(i);
        _secondary_coord_name[i] = _model.getComponent<Coordinate>
            (get_secondary_coordinates(i)).getName();
    }

    int nCoord = 0;
    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        std::string path = coord.getAbsolutePathString();

        int ind = _secondary_coord_path.findIndex(path);
        if (ind > -1) {
            _secondary_coord_index[ind] = nCoord;
        }
        nCoord++;
    }


    log_info("Secondary Coordinates:");
    log_info("----------------------");
    for (int i = 0; i < _n_secondary_coord; ++i) {
        log_info(_secondary_coord_name[i]);
    }
    
    log_info("Secondary Coupled Coordinate: {}",
        get_secondary_coupled_coordinate());

    if (get_perform_secondary_constraint_sim()) {
        log_info("Settle Threshold: {}",
            get_secondary_constraint_sim_settle_threshold());

        log_info("Sweep Time: {}", 
            get_secondary_constraint_sim_sweep_time());

        log_info("Sweep secondary_coupled_coordinate start value: {}",
            get_secondary_coupled_coordinate_start_value());

        log_info("Sweep secondary_coupled_coordinate stop value: {}",
            get_secondary_coupled_coordinate_stop_value());
    }

    _state = _model.initSystem();

    return true;
}




void COMAKInverseKinematicsTool::performIKSecondaryConstraintSimulation() {
    log_info("Performing IK Secondary Constraint Simulation...");

    //Initialize Model
    Model model = *_model.clone();
    
    model.initSystem();

    for (Muscle& msl : model.updComponentList<Muscle>()) {
        if (msl.getConcreteClassName() == "Millard2012EquilibriumMuscle") {
            msl.set_ignore_activation_dynamics(true);
            msl.set_ignore_tendon_compliance(true);
        }
    }

    //Set coordinate types
    for (auto& coord : model.updComponentList<Coordinate>()) {
        if (getProperty_secondary_coordinates().findIndex(coord.getAbsolutePathString()) > -1) {
            coord.set_locked(false);
            coord.set_clamped(false);
        }
        else if (coord.getAbsolutePathString() == get_secondary_coupled_coordinate()){
            coord.set_locked(false);
            coord.set_clamped(false);
            coord.set_prescribed(true);
        }
        else {
            coord.set_locked(true);
        }
    }
    
    for(auto& cc_const : model.updComponentList<CoordinateCouplerConstraint>()){
        std::string cc_coord_name = cc_const.getDependentCoordinateName();
        Coordinate& coord = model.updCoordinateSet().get(cc_coord_name);
        coord.set_locked(false);
        coord.set_clamped(false);
    }

    Coordinate& coupled_coord = 
        model.updComponent<Coordinate>(get_secondary_coupled_coordinate());
    
    double start_value;
    double stop_value;

    if (model.getComponent<Coordinate>(get_secondary_coupled_coordinate()).
            getMotionType() == Coordinate::MotionType::Rotational) {

        start_value = 
            get_secondary_coupled_coordinate_start_value() * SimTK::Pi / 180;
        stop_value = 
            get_secondary_coupled_coordinate_stop_value() * SimTK::Pi / 180;
    }
    else {
        start_value = get_secondary_coupled_coordinate_start_value();
        stop_value = get_secondary_coupled_coordinate_stop_value();
    }

    //Initialize Simulation 
    //---------------------

    
        
    //Perform Settling Simulation
    //---------------------------

    //prescribe coupled coord
    Constant settle_func = Constant(start_value);
    coupled_coord.set_prescribed_function(settle_func);
    
    model.setUseVisualizer(get_use_visualizer());
    SimTK::State state = model.initSystem();

    if (get_use_visualizer()) {
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        viz.setWindowTitle("COMAK IK Settle Simulation");
        viz.setBackgroundColor(SimTK::White);
        viz.setShowSimTime(true);
        viz.setDesiredFrameRate(100);        
    }

    //prescribe muscle force
    for (Muscle& msl : model.updComponentList<Muscle>()) {
        msl.overrideActuation(state, true);
        double value = msl.getMaxIsometricForce()*0.01;
        msl.setOverrideActuation(state, value);
    }
    model.equilibrateMuscles(state);

    //setup integrator
    SimTK::CPodesIntegrator integrator(
        model.getSystem(), SimTK::CPodes::BDF, SimTK::CPodes::Newton);
    integrator.setAccuracy(get_secondary_constraint_sim_integrator_accuracy());

    if (get_secondary_constraint_sim_internal_step_limit() != -1) {
        integrator.setInternalStepLimit(
            get_secondary_constraint_sim_internal_step_limit());
    }
    SimTK::TimeStepper timestepper(model.getSystem(), integrator);

    timestepper.initialize(state);
    
    StatesTrajectory settle_states;
    settle_states.append(state);
    double dt = 0.01;
 

    log_info("Starting Settling Simulation.");
    

    SimTK::Vector prev_sec_coord_value(_n_secondary_coord,0.0);

    for (int k = 0; k < _n_secondary_coord; k++) {
        Coordinate& coord = model.updComponent<Coordinate>(_secondary_coord_path[k]);
        double value = coord.getValue(state);
        prev_sec_coord_value(k) = value;
    }

    double max_coord_delta = SimTK::Infinity;
    int i = 1;
    while (max_coord_delta > get_secondary_constraint_sim_settle_threshold()){
        timestepper.stepTo(i*dt);
        state = timestepper.getState();
        settle_states.append(state);
            
        log_info("Time: {}", state.getTime());
        log_info("{:<30} {:<20} {:<20}","Name", "Value", "Value Change");
        

        //Compute Delta Coordinate
        max_coord_delta = 0;
        for (int k = 0; k < _n_secondary_coord; k++) {
            
            Coordinate& coord = model.updComponent<Coordinate>(_secondary_coord_path[k]);
            double value = coord.getValue(state);
            double delta = abs(value - prev_sec_coord_value(k));
            
            if (delta > max_coord_delta) {
                max_coord_delta = delta;
            }
            prev_sec_coord_value(k) = value;


            log_info("{:<30} {:<20} {:<20}", coord.getName(), value, delta);

        }
        printDebugInfo(model,state);  

        i++;
    }

    SimTK::Vector settled_secondary_values(_secondary_coord_path.getSize());
    SimTK::Vector settled_secondary_speeds(_secondary_coord_path.getSize());

    //Save secondardy coord values to initialize sweep simulation
    for (int c = 0; c < _secondary_coord_path.getSize(); c++) {
        std::string secondary_coord = _secondary_coord_path[c];
        Coordinate& coord = model.updComponent<Coordinate>(secondary_coord);
        settled_secondary_values.set(c, coord.getValue(state));
        settled_secondary_speeds.set(c, coord.getSpeedValue(state));
    }


     log_info("Finished Settling Simulation in {} s.", state.getTime());
     log_info("Starting Sweep Simulation.");


    //Perform Sweep Simulation
    //------------------------

    //setup quadratic sweep function
    double Vx = 0;
    double Vy = start_value;
    double Px = Vx + get_secondary_constraint_sim_sweep_time();
    double Py = stop_value;
    double a = (Py - Vy) / SimTK::square(Px - Vx);

    double C1 = a;
    double C2 = -2 * a * Vx;
    double C3 = a * SimTK::square(Vx) + Vy;

    SimTK::Vector coefficients(3);
    coefficients.set(0, C1);
    coefficients.set(1, C2);
    coefficients.set(2, C3);

    PolynomialFunction sweep_func = PolynomialFunction(coefficients);
    coupled_coord.set_prescribed_function(sweep_func);

    state = model.initSystem();
    if (get_use_visualizer()) {
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        viz.setWindowTitle("COMAK IK Sweep Simulation");
        viz.setBackgroundColor(SimTK::White);
        viz.setShowSimTime(true);
        viz.setDesiredFrameRate(100);
    }

    //prescribe muscle force
    for (Muscle& msl : model.updComponentList<Muscle>()) {
        msl.overrideActuation(state, true);
        double value = msl.getMaxIsometricForce()*0.01;
        msl.setOverrideActuation(state, value);
    }
    model.equilibrateMuscles(state);

    //set settled secondary coordinate values
    for (int c = 0; c < _secondary_coord_path.getSize(); c++) {
        std::string secondary_coord = _secondary_coord_path[c];
        Coordinate& coord = model.updComponent<Coordinate>(secondary_coord);
        coord.setValue(state, settled_secondary_values(c));
        coord.setSpeedValue(state, settled_secondary_speeds(c));
    }

    double sweep_start = 0;
    double sweep_stop = Px;

    int nSteps = (int)lround((sweep_stop - sweep_start) / dt);

    //Setup storage for computing constraint functions
    TimeSeriesTable q_table;
    SimTK::RowVector q_row(model.getNumCoordinates());
    std::vector<std::string> q_names;

    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        q_names.push_back(coord.getAbsolutePathString() + "/value");
    }

    q_table.setColumnLabels(q_names);

    //setup integrator
    SimTK::CPodesIntegrator sweep_integrator(model.getSystem(), SimTK::CPodes::BDF, SimTK::CPodes::Newton);
    sweep_integrator.setAccuracy(get_secondary_constraint_sim_integrator_accuracy());
    if (get_secondary_constraint_sim_internal_step_limit() != -1) {
        sweep_integrator.setInternalStepLimit(
            get_secondary_constraint_sim_internal_step_limit());
    }
    SimTK::TimeStepper sweep_timestepper(model.getSystem(), sweep_integrator);

    sweep_timestepper.initialize(state);
    
    StatesTrajectory sweep_states;
    sweep_states.append(state);

    for (int i = 0; i <= nSteps; ++i) {

        sweep_timestepper.stepTo(sweep_start + i*dt);
        state = sweep_timestepper.getState();

        sweep_states.append(state);

        int j = 0;
        for (const auto& coord : model.getComponentList<Coordinate>()) {
            q_row(j) = coord.getValue(state);
            j++;
        }
        q_table.appendRow(state.getTime(), q_row);

        log_info("{}",state.getTime());
        printDebugInfo(model,state);  
    }

    //Compute Coupled Constraint Functions
    std::vector<double> time = q_table.getIndependentColumn();

    SimTK::Vector ind_data = q_table.getDependentColumn(get_secondary_coupled_coordinate() + "/value");

    SimTK::Matrix data((int)time.size(), _n_secondary_coord);

    for (int j = 0; j < _n_secondary_coord; ++j) {
        std::string path = _secondary_coord_path[j];
        SimTK::Vector col_data = q_table.getDependentColumn(path + "/value");

        for (int i = 0; i < (int)time.size(); ++i) {
            data(i, j) = col_data(i);

        }
    }

    double ind_max = SimTK::max(ind_data);
    double ind_min = SimTK::min(ind_data);

    int npts = get_constraint_function_num_interpolation_points();
    double step = (ind_max - ind_min) / npts;

    SimTK::Vector ind_pt_data(npts);

    for (int i = 0; i < npts+1; ++i) {
        ind_pt_data(i) = ind_min + i * step;
    }

    _secondary_constraint_functions.clearAndDestroy();

    for (int j = 0; j < _n_secondary_coord; ++j) {
        std::string path = _secondary_coord_path[j];

        SimTK::Vector secondary_data = data(j);

        
        
        SimmSpline data_fit = SimmSpline(secondary_data.size(), &ind_data[0], &secondary_data[0]);
        //GCVSpline* spline = new GCVSpline(5, secondary_data.nrow(), &ind_data[0], &secondary_data[0], path, -1);
        //GCVSpline* spline = new GCVSpline();
        //spline->setDegree(5);
        //spline->set
        //SimmSpline* spline = new SimmSpline();
        //spline->setName(path);

        SimTK::Vector dep_interp_pts(npts);

        for (int i = 0; i < npts; ++i) {
            //spline->addPoint(ind_pt_data(i), data_fit.calcValue(SimTK::Vector(1, ind_pt_data(i))));
            dep_interp_pts(i) = data_fit.calcValue(SimTK::Vector(1, ind_pt_data(i)));
        }
        SimmSpline* spline = new SimmSpline(npts, &ind_pt_data[0], &dep_interp_pts[0],path);
        
        //GCVSpline* spline = new GCVSpline(5, npts, &ind_data[0], &dep_interp_pts[0], path, -1);

        _secondary_constraint_functions.cloneAndAppend(*spline);
    }

    //Print Secondardy Constraint Functions to file
    _secondary_constraint_functions.print(get_secondary_constraint_function_file());

    //Write Outputs
    if (get_print_secondary_constraint_sim_results()) {
        log_info("Printing secondary constraint simulation results: {}",
            get_results_directory());

        std::string name = "secondary_constraint_sim_states";

        TimeSeriesTable settle_table = settle_states.exportToTable(model);
        settle_table.addTableMetaData("header", name);
        settle_table.addTableMetaData("nRows", 
            std::to_string(settle_table.getNumRows()));
        settle_table.addTableMetaData("nColumns", 
            std::to_string(settle_table.getNumColumns() + 1));

        TimeSeriesTable sweep_table = sweep_states.exportToTable(model);
        sweep_table.addTableMetaData("header", name);
        sweep_table.addTableMetaData("nRows", 
            std::to_string(sweep_table.getNumRows()));
        sweep_table.addTableMetaData("nColumns", 
            std::to_string(sweep_table.getNumColumns() + 1));

        std::string settle_file =
            get_results_directory() + "/" + get_results_prefix() +
            "_secondary_constraint_settle_states.sto";

        std::string sweep_file =
            get_results_directory() + "/" + get_results_prefix() +
            "_secondary_constraint_sweep_states.sto";

        STOFileAdapter sto_file_adapt;
        sto_file_adapt.write(settle_table, settle_file);
        sto_file_adapt.write(sweep_table, sweep_file);
    }
}

void COMAKInverseKinematicsTool::performIK()
{
    Model model = _model;
    model.setUseVisualizer(false);
    model.initSystem();

    // check marker file exists
    if (!SimTK::Pathname::fileExists(get_marker_file())) {
        OPENSIM_THROW(Exception, "COMAKInverseKinematicsTool: marker_file "
            "does not exist! " + get_marker_file())
    }

    // Check that IK tasks exist as markers

    for (int i = 0; i < get_IKTaskSet().getSize(); i++) {
        std::string iktask_name = get_IKTaskSet().get(i).getName();

        if (get_IKTaskSet().get(i).getConcreteClassName() == "IKMarkerTask"){
            try {
                model.getMarkerSet().get(iktask_name);
            }
            catch (Exception const&) {
                OPENSIM_THROW(Exception, "COMAKInverseKinematics "
                    "IKMarkerTask: " + iktask_name + 
                    " does not exist as a marker in the model.");
            }
            try {
                TimeSeriesTableVec3 trc = TimeSeriesTableVec3(get_marker_file());
                trc.getColumnIndex(iktask_name);
            }
            catch (Exception const&) {
                OPENSIM_THROW(Exception, "COMAKInverseKinematics "
                    "IKMarkerTask: " + iktask_name + 
                    " does not exist as a marker in the .trc marker_file: " +
                    get_marker_file());
            }
            
        }
        if (get_IKTaskSet().get(i).getConcreteClassName() == "IKCoordinateTask"){
            try {
                model.getCoordinateSet().get(iktask_name);
            }
            catch (Exception const&) {
                OPENSIM_THROW(Exception, "COMAKInverseKinematics "
                    "IKCoordinateTask: " + iktask_name + 
                    " does not exist as a coordinate in the model.");
            }
        }
    }

    try {
        _secondary_constraint_functions =
            FunctionSet(get_secondary_constraint_function_file());
    }
    catch (Exception const&) {
        OPENSIM_THROW(Exception, "Function file: " +
            get_secondary_constraint_function_file() + " does not exist.");

    }

    SimTK::Vector coupled_coord_default_value = SimTK::Vector(1,
        model.getComponent<Coordinate>(
            get_secondary_coupled_coordinate()).getDefaultValue());

    const std::string& secondary_coupled_coord_name = 
        model.getComponent<Coordinate>(
            get_secondary_coupled_coordinate()).getName();

   //Add CoordinateCouplerConstraints for Secondary Kinematics
   for (int i = 0; i < getProperty_secondary_coordinates().size(); ++i) {
        std::string path = get_secondary_coordinates(i);
        Coordinate& coord = model.updComponent<Coordinate>(path);
        std::string coord_name = coord.getName();

        std::string ind_coord_name = model.getComponent<Coordinate>(
            get_secondary_coupled_coordinate()).getName();

        std::string joint_path = coord.getJoint().getAbsolutePathString();
        
        const Function& function = _secondary_constraint_functions.get(path);
        CoordinateCouplerConstraint* cc_constraint = new CoordinateCouplerConstraint();

        cc_constraint->setIndependentCoordinateNames(
            Array<std::string>(ind_coord_name, 1, 2));
        cc_constraint->setDependentCoordinateName(coord_name);
        cc_constraint->setFunction(function);
        cc_constraint->setName(coord_name + "_function");

        coord.setDefaultValue(function.calcValue(coupled_coord_default_value));

        model.addConstraint(cc_constraint);
   }

    //Set coordinate types
    for (auto& coord : model.updComponentList<Coordinate>()) {
        if (getProperty_secondary_coordinates().findIndex(coord.getAbsolutePathString()) > -1) {
            coord.set_locked(false);
            coord.set_clamped(false);
            coord.set_prescribed(false);
        }
        else if (coord.getAbsolutePathString() == get_secondary_coupled_coordinate()){
            coord.set_locked(false);
            //coord.set_clamped(true);
        }
    }

    for(auto& cc_const : model.updComponentList<CoordinateCouplerConstraint>()){
        std::string cc_coord_name = cc_const.getDependentCoordinateName();
        Coordinate& coord = model.updCoordinateSet().get(cc_coord_name);
        coord.set_locked(false);
        coord.set_clamped(false);
    }

    SimTK::State state = model.initSystem();

    if (!get_constrained_model_file().empty()) {
        model.print(get_constrained_model_file());
    }

    runInverseKinematics(model);
}

void COMAKInverseKinematicsTool::runInverseKinematics(Model& model) {

     Kinematics* kinematicsReporter = nullptr;

    try{
        // although newly loaded model will be finalized
        // there is no guarantee that the model has not been edited/modified
        model.finalizeFromProperties();
        model.printBasicInfo();

        // Define reporter for output
        kinematicsReporter = new Kinematics();
        kinematicsReporter->setRecordAccelerations(false);
        kinematicsReporter->setInDegrees(true);
        model.addAnalysis(kinematicsReporter);

        log_info("Running Inverse Kinematics\n");

        // Initialize the model's underlying system and get its default state.
        model.setUseVisualizer(get_use_visualizer());
        SimTK::State& s = model.initSystem();

        //Convert old Tasks to references for assembly and tracking
        MarkersReference markersReference;
        SimTK::Array_<CoordinateReference> coordinateReferences;
        // populate the references according to the setting of this Tool
        populateReferences(model, markersReference, coordinateReferences);

        // Determine the start time, if the provided time range is not 
        // specified then use time from marker reference.
        // Adjust the time range for the tool if the provided range exceeds
        // that of the marker data.
        SimTK::Vec2 markersValidTimeRange = markersReference.getValidTimeRange();
        double start_time = (markersValidTimeRange[0] > get_time_range(0)) ?
            markersValidTimeRange[0] : get_time_range(0);
        double final_time = (markersValidTimeRange[1] < get_time_range(1)) ?
            markersValidTimeRange[1] : get_time_range(1);

        SimTK_ASSERT2_ALWAYS(final_time >= start_time,
            "InverseKinematicsTool final time (%f) is before start time (%f).",
            final_time, start_time);

        const auto& markersTable = markersReference.getMarkerTable();
        const int start_ix = int(
            markersTable.getNearestRowIndexForTime(start_time) );
        const int final_ix = int(
            markersTable.getNearestRowIndexForTime(final_time) );
        const int Nframes = final_ix - start_ix + 1;
        const auto& times = markersTable.getIndependentColumn();

        // create the solver given the input data
        InverseKinematicsSolver ikSolver(model, markersReference,
            coordinateReferences, get_ik_constraint_weight());
        ikSolver.setAccuracy(get_ik_accuracy());
        s.updTime() = times[start_ix];

        for (int i = 0; i < 7; ++i) {
            try {
                ikSolver.assemble(s);
                break;
            }
            catch (std::exception const&){
                try {
                    ikSolver.track(s);
                    break;
                }
                catch (std::exception const&) {
                    log_error("Assembly failed... " 
                        "retrying with new initial conditions.");
                }
            }
        }

        kinematicsReporter->begin(s);

        AnalysisSet& analysisSet = model.updAnalysisSet();
        analysisSet.begin(s);
        // Get the actual number of markers the Solver is using, which
        // can be fewer than the number of references if there isn't a
        // corresponding model marker for each reference.
        int nm = ikSolver.getNumMarkersInUse();
        SimTK::Array_<double> squaredMarkerErrors(nm, 0.0);
        SimTK::Array_<SimTK::Vec3> markerLocations(nm, SimTK::Vec3(0));
        
        Storage *modelMarkerLocations = get_report_marker_locations() ?
            new Storage(Nframes, "ModelMarkerLocations") : nullptr;
        Storage *modelMarkerErrors = get_report_errors() ? 
            new Storage(Nframes, "ModelMarkerErrors") : nullptr;

        Stopwatch watch;

        for (int i = start_ix; i <= final_ix; ++i) {
            s.updTime() = times[i];
            ikSolver.track(s);
            // show progress line every 1000 frames so users see progress
            if (std::remainder(i - start_ix, 1000) == 0 && i != start_ix)
                std::cout << "Solved " << i - start_ix << " frames..." << std::endl;
            if(get_report_errors()){
                Array<double> markerErrors(0.0, 3);
                double totalSquaredMarkerError = 0.0;
                double maxSquaredMarkerError = 0.0;
                int worst = -1;

                ikSolver.computeCurrentSquaredMarkerErrors(squaredMarkerErrors);
                for(int j=0; j<nm; ++j){
                    totalSquaredMarkerError += squaredMarkerErrors[j];
                    if(squaredMarkerErrors[j] > maxSquaredMarkerError){
                        maxSquaredMarkerError = squaredMarkerErrors[j];
                        worst = j;
                    }
                }

                double rms = nm > 0 ? sqrt(totalSquaredMarkerError / nm) : 0;
                markerErrors.set(0, totalSquaredMarkerError); 
                markerErrors.set(1, rms);
                markerErrors.set(2, sqrt(maxSquaredMarkerError));
                modelMarkerErrors->append(s.getTime(), 3, &markerErrors[0]);

                std::cout << "Frame " << i << " (t=" << s.getTime() << "):\t"
                    << "total squared error = " << totalSquaredMarkerError
                    << ", marker error: RMS=" << rms << ", max="
                    << sqrt(maxSquaredMarkerError) << " (" 
                    << ikSolver.getMarkerNameForIndex(worst) << ")" << std::endl;
            }

            if(get_report_marker_locations()){
                ikSolver.computeCurrentMarkerLocations(markerLocations);
                Array<double> locations(0.0, 3*nm);
                for(int j=0; j<nm; ++j){
                    for(int k=0; k<3; ++k)
                        locations.set(3*j+k, markerLocations[j][k]);
                }

                modelMarkerLocations->append(s.getTime(), 3*nm, &locations[0]);

            }

            kinematicsReporter->step(s, i);
            analysisSet.step(s, i);
        }

        if (get_output_motion_file() != "" &&
                get_output_motion_file() != "Unassigned") {
            kinematicsReporter->getPositionStorage()->print(
                    get_results_directory() + "/" + get_output_motion_file());
        }
        // Remove the analysis we added to the model, this also deletes it
        model.removeAnalysis(kinematicsReporter);

        if (modelMarkerErrors) {
            Array<std::string> labels("", 4);
            labels[0] = "time";
            labels[1] = "total_squared_error";
            labels[2] = "marker_error_RMS";
            labels[3] = "marker_error_max";

            modelMarkerErrors->setColumnLabels(labels);
            modelMarkerErrors->setName("Model Marker Errors from IK");

            IO::makeDir(get_results_directory());
            std::string errorFileName = get_results_prefix() + 
                "_ik_marker_errors";
            Storage::printResult(modelMarkerErrors, errorFileName,
                                 get_results_directory(), -1, ".sto");

            delete modelMarkerErrors;
        }

        if(modelMarkerLocations){
            Array<std::string> labels("", 3*nm+1);
            labels[0] = "time";
            Array<std::string> XYZ("", 3*nm);
            XYZ[0] = "_tx"; XYZ[1] = "_ty"; XYZ[2] = "_tz";

            for(int j=0; j<nm; ++j){
                for(int k=0; k<3; ++k)
                    labels.set(3*j+k+1, ikSolver.getMarkerNameForIndex(j)+XYZ[k]);
            }
            modelMarkerLocations->setColumnLabels(labels);
            modelMarkerLocations->setName("Model Marker Locations from IK");
    
            IO::makeDir(get_results_directory());
            std::string markerFileName = get_results_prefix() + 
                "_ik_model_marker_locations";
            Storage::printResult(modelMarkerLocations, markerFileName,
                                 get_results_directory(), -1, ".sto");

            delete modelMarkerLocations;
        }

        log_info("InverseKinematicsTool completed {} frames in {}",
            Nframes, watch.getElapsedTimeFormatted());
    }
    catch (const std::exception& ex) {
         log_error("InverseKinematicsTool Failed: {}", ex.what());
        // If failure happened after kinematicsReporter was added, make sure to cleanup
        if (kinematicsReporter!= nullptr)
            model.removeAnalysis(kinematicsReporter);
        throw (Exception("InverseKinematicsTool Failed, "
            "please see messages window for details..."));
    }
}

void COMAKInverseKinematicsTool::populateReferences(const Model& model, 
    MarkersReference& markersReference,
    SimTK::Array_<CoordinateReference>&coordinateReferences) const
{
    FunctionSet *coordFunctions = NULL;
    // Load the coordinate data
    // bool haveCoordinateFile = false;
    if (get_coordinate_file() != "" && get_coordinate_file() != "Unassigned") {
        Storage coordinateValues(get_coordinate_file());
        // Convert degrees to radian (TODO: this needs to have a check that the storage is, in fact, in degrees!)
        model.getSimbodyEngine().convertDegreesToRadians(coordinateValues);
        // haveCoordinateFile = true;
        coordFunctions = new GCVSplineSet(5, &coordinateValues);
    }

    Set<MarkerWeight> markerWeights;
    // Loop through old "IKTaskSet" and assign weights to the coordinate and marker references
    // For coordinates, create the functions for coordinate reference values
    int index = 0;
    for (int i = 0; i < get_IKTaskSet().getSize(); i++) {
        if (!get_IKTaskSet()[i].getApply()) continue;
        if (IKCoordinateTask *coordTask = dynamic_cast<IKCoordinateTask *>(&get_IKTaskSet()[i])) {
            CoordinateReference *coordRef = NULL;
            if (coordTask->getValueType() == IKCoordinateTask::FromFile) {
                if (!coordFunctions)
                    throw Exception("InverseKinematicsTool: value for coordinate " + coordTask->getName() + " not found.");

                index = coordFunctions->getIndex(coordTask->getName(), index);
                if (index >= 0) {
                    coordRef = new CoordinateReference(coordTask->getName(), coordFunctions->get(index));
                }
            }
            else if ((coordTask->getValueType() == IKCoordinateTask::ManualValue)) {
                Constant reference(Constant(coordTask->getValue()));
                coordRef = new CoordinateReference(coordTask->getName(), reference);
            }
            else { // assume it should be held at its default value
                double value = model.getCoordinateSet().get(coordTask->getName()).getDefaultValue();
                Constant reference = Constant(value);
                coordRef = new CoordinateReference(coordTask->getName(), reference);
            }

            if (coordRef == NULL)
                throw Exception("InverseKinematicsTool: value for coordinate " + coordTask->getName() + " not found.");
            else
                coordRef->setWeight(coordTask->getWeight());

            coordinateReferences.push_back(*coordRef);
        }
        else if (IKMarkerTask *markerTask = dynamic_cast<IKMarkerTask *>(&get_IKTaskSet()[i])) {
            if (markerTask->getApply()) {
                // Only track markers that have a task and it is "applied"
                markerWeights.adoptAndAppend(
                    new MarkerWeight(markerTask->getName(), markerTask->getWeight()));
            }
        }
    }

    //Read in the marker data file and set the weights for associated markers.
    //Markers in the model and the marker file but not in the markerWeights are
    //ignored
    markersReference.initializeFromMarkersFile(get_marker_file(), markerWeights);
}

void COMAKInverseKinematicsTool::printDebugInfo(const Model& model, const SimTK::State& state) {

    model.realizeReport(state);

    // Unconstrained Coordinates
    log_debug("{:<30} {:<20} {:<20}", "Unconstrained Coordinate", "Value",
            "Speed");

    for (int i = 0; i < getProperty_secondary_coordinates().size();
            ++i) {
        std::string coord_path = get_secondary_coordinates(i);
        const Coordinate& coord = model.getComponent<Coordinate>(coord_path);
        log_debug("{:<30} {:<20} {:<20}", coord.getName(),
                coord.getValue(state), coord.getSpeedValue(state));
    }
    log_debug("");

    // Muscle
    log_debug("{:<20} {:<20} {:<20} {:<20}", "Muscle", "Force", "Activation",
            "Control");

    for (const Muscle& msl : model.getComponentList<Muscle>()) {
        log_debug("{:<20} {:<20} {:<20} {:<20}", msl.getName(),
                msl.getActuation(state), msl.getActivation(state),
                msl.getControl(state));
    }
    log_debug("");

    // Ligament
    log_debug("{:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12}", "Ligament",
            "Total Force", "Spring Force", "Damping Force", "Strain",
            "Strain Rate", "Length");

    for (const Blankevoort1991Ligament& lig :
            model.getComponentList<Blankevoort1991Ligament>()) {

        log_debug("{:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12}",
                lig.getName(), lig.getOutputValue<double>(state, "total_force"),
                lig.getOutputValue<double>(state, "spring_force"),
                lig.getOutputValue<double>(state, "damping_force"),
                lig.getOutputValue<double>(state, "strain"),
                lig.getOutputValue<double>(state, "strain_rate"),
                lig.getOutputValue<double>(state, "length"));
    }
    log_debug("");

    // Contact
    log_debug("{:<20} {:<20} {:<20}", "Contact", "Force", "COP");

    for (const Smith2018ArticularContactForce& cnt :
            model.getComponentList<Smith2018ArticularContactForce>()) {

        log_debug("{:<20} {:<20} {:<20}", cnt.getName(),
                cnt.getOutputValue<SimTK::Vec3>(
                        state, "casting_total_contact_force"),
                cnt.getOutputValue<SimTK::Vec3>(
                        state, "casting_total_center_of_pressure"));
    }
    log_debug("");
    /*
        if (get_verbose() >= 3) {
            log_debug("{:<20} {:<20} {:<20}", "Contact " << std::setw(20)
                << "Force" << std::setw(w) << "COP" << std::endl;

            for (Smith2018ArticularContactForce& cnt :
       model.updComponentList<Smith2018ArticularContactForce>()) {

                std::cout << std::setw(w) << cnt.getName()
                    << std::setw(w) << cnt.getOutputValue<SimTK::Vector>(state,
       "casting_triangle_proximity")
                    << std::setw(w) << cnt.getOutputValue<SimTK::Vector>(state,
       "casting_triangle_pressure")
                    << std::endl;
            }
        }

        if (get_verbose() >= 2) {
            std::cout << "Press Any Key to Continue." << std::endl;
            std::cin.ignore();
        }*/
}