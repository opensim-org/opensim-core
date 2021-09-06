/* -------------------------------------------------------------------------- *
 *                          JointMechanicsTool.cpp                            *
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

#include <OpenSim/Simulation/Model/Model.h>
#include "JointMechanicsTool.h"
#include "VTPFileAdapter.h"
#include "H5FileAdapter.h"
//#include "H5Cpp.h"
//#include "hdf5_hl.h"
#include "OpenSim/Simulation/Model/Smith2018ArticularContactForce.h"
#include "JAMUtilities.h"
#include "OpenSim/Simulation/Model/Blankevoort1991Ligament.h"
#include <OpenSim/Analyses/StatesReporter.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldConstraint.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include "OpenSim/Tools/IKCoordinateTask.h"
#include "OpenSim/Tools/IKMarkerTask.h"
#include "OpenSim/Tools/IKTaskSet.h"
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Common/TableUtilities.h>
#include <OpenSim/Common/Stopwatch.h>

using namespace OpenSim;

JointMechanicsTool::JointMechanicsTool() : Object()
{
    setNull();
    constructProperties();
    //_directoryOfSetupFile = "";
    _model_exists = false;
}

JointMechanicsTool::JointMechanicsTool(std::string settings_file) : 
    Object(settings_file) {
    constructProperties();
    updateFromXMLDocument();
    //loadModel(settings_file);

    //_directoryOfSetupFile = IO::getParentDirectory(settings_file);
    //IO::chDir(_directoryOfSetupFile);
}
/*
JointMechanicsTool::JointMechanicsTool(Model *aModel, 
    std::string states_file, std::string results_dir) :
    JointMechanicsTool()
{
    if(aModel==NULL) return;
    setModel(*aModel);

    set_input_states_file(states_file);
    set_results_directory(results_dir);
}*/

void JointMechanicsTool::setNull()
{
    setAuthors("Colin Smith");
}

void JointMechanicsTool::constructProperties()
{
    Array<std::string> defaultListAll;
    defaultListAll.append("all");

    Array<std::string> defaultListNone;
    defaultListNone.append("none");

    constructProperty_model_file("");
    constructProperty_input_states_file("");
    constructProperty_use_muscle_physiology("");
    constructProperty_input_transforms_file("");
    constructProperty_input_forces_file("");
    constructProperty_input_activations_file("");
    constructProperty_input_comak_convergence_file("");
    constructProperty_input_inverse_dynamics_file("");

    constructProperty_results_directory(".");
    constructProperty_results_file_basename("");
    constructProperty_start_time(-1);
    constructProperty_stop_time(-1);
    constructProperty_resample_step_size(-1);
    constructProperty_normalize_to_cycle(false);
    constructProperty_lowpass_filter_frequency(-1);
    constructProperty_print_processed_kinematics(false);

    constructProperty_coordinates(defaultListAll);
    constructProperty_coordinate_outputs(defaultListAll);
    constructProperty_contacts(defaultListAll);
    constructProperty_contact_outputs(defaultListAll);
    constructProperty_contact_mesh_properties(defaultListNone);
    constructProperty_ligaments(defaultListAll);
    constructProperty_ligament_outputs(defaultListAll);
    constructProperty_muscles(defaultListNone);
    constructProperty_muscle_outputs(defaultListNone);
    constructProperty_attached_geometry_bodies(defaultListNone);
    
    constructProperty_output_position_frame("ground");
    constructProperty_output_orientation_frame("ground");
    
    constructProperty_write_vtp_files(true);
    constructProperty_vtp_file_format("binary");
    constructProperty_write_h5_file(true);
    constructProperty_h5_states_data(true);
    constructProperty_h5_kinematics_data(true);
    //constructProperty_h5_transforms_data(false);

    constructProperty_write_transforms_file(false);
    constructProperty_output_transforms_file_type("sto");

    constructProperty_AnalysisSet(AnalysisSet());
    constructProperty_use_visualizer(false);
    constructProperty_verbose(0);
}

void JointMechanicsTool::setModel(Model& aModel)
{
    _model = aModel;
    set_model_file(aModel.getDocumentFileName());
    _model_exists = true;
}


bool JointMechanicsTool::run() {
    bool completed = false;
    
    auto cwd = IO::CwdChanger::changeToParentOf(getDocumentFileName());

    try {
        const Stopwatch stopwatch;
        log_critical("");
        log_critical("====================");
        log_critical("Joint Mechanics Tool");
        log_critical("====================");
        log_critical("");

        //Set the max number of points a ligament or muscle path can contain
        _max_path_points = 100;

        //Make results directory
        int makeDir_out = IO::makeDir(get_results_directory());
        if (errno == ENOENT && makeDir_out == -1) {
            OPENSIM_THROW(Exception, "Could not create " +
                get_results_directory() +
                "Possible reason: This tool cannot make new folder with subfolder.");
        }

        //Set Model
        if (!_model_exists) {
            if (get_model_file().empty()) {
                OPENSIM_THROW(Exception, 
                    "No model was set in the JointMechanicsTool.");
            }
            _model = Model(get_model_file());
        }

        initialize();

        SimTK::Visualizer* viz=NULL;
        if (get_use_visualizer()) {
            viz = &_model.updVisualizer().updSimbodyVisualizer();
            viz->setShowSimTime(true);
        }

        //loop over each frame
        for (int i = 0; i < _n_frames; ++i) {
            //Set State
            SimTK::State state = _states[i];

            log_info("Time: {}", state.getTime()); 
            state.invalidateAllCacheAtOrAbove(SimTK::Stage::Time);
            //Record Values
            record(state,i);

            //Perform analyses
            if (i == 0) {
                _model.updAnalysisSet().begin(state);
            }
            else {
                _model.updAnalysisSet().step(state, i);
            }

            //Visualize
            if (get_use_visualizer()) {
                viz->drawFrameNow(state);
            }
        }
        printResults(get_results_file_basename(), get_results_directory());

        const long long elapsed = stopwatch.getElapsedTimeInNs();

        log_info("JointMechanicsTool complete.");
        log_info("Finished in {}", stopwatch.formatNs(elapsed));
        log_info("Printed results to: {}", get_results_directory());
        log_info("");
        completed = true;
    }

    catch(const std::exception& x) {
        log_error("JointMechanicsTool::run() caught an exception: \n {}", x.what());
        cwd.restore();
    }
    catch (...) { // e.g. may get InterruptedException
        log_error("JointMechanicsTool::run() caught an exception.");
        cwd.restore();
    }

    cwd.restore();

    return completed;
}

void JointMechanicsTool::initialize() {
    clearInitializedMemberData();

    //Read input files, trim, filter data
    Storage input_data;

    if (!get_input_transforms_file().empty() &&
        !get_input_states_file().empty()) {
        OPENSIM_THROW(Exception,"Either the input_states_file or the "
            "input_transforms_file must be empty.")
    }
    else if (!get_input_transforms_file().empty()) {
        input_data = processInputStorage(get_input_transforms_file());
    }
    else if (!get_input_states_file().empty()) {
        input_data = processInputStorage(get_input_states_file());
    }
    else {
        OPENSIM_THROW(Exception,"Either the input_states_file or the "
            "input_transforms_file must be set.")
    }

    //States
    if (get_h5_states_data()) {

        /*if (_model.getAnalysisSet.contains("joint_mechanics_states_analysis")) {
            auto& states_rep = _model.getAnalysisSet().get("joint_mechanics_states_analysis");

            //states_rep.set
        }*/
        
            StatesReporter* states_rep = new StatesReporter();
            states_rep->setName("joint_mechanics_states_analysis");
            states_rep->setStepInterval(1);
            states_rep->setPrintResultFiles(false);
            _model.addAnalysis(states_rep);
    }

    //Add Analysis set
    AnalysisSet aSet = get_AnalysisSet();
    int size = aSet.getSize();

    for(int i=0;i<size;i++) {
        Analysis *analysis = aSet.get(i).clone();
        _model.addAnalysis(analysis);
    }

    AnalysisSet& analysisSet = _model.updAnalysisSet();

    if (get_use_visualizer()) {
        _model.setUseVisualizer(true);
    }
    SimTK::State state = _model.initSystem();

    for (auto& cnt : _model.updComponentList<Smith2018ArticularContactForce>()) {
        cnt.setModelingOption(state, "flip_meshes", 1);
    }

    if (!get_input_transforms_file().empty()) {
        assembleStatesTrajectoryFromTransformsData(input_data, state);
    }
    else if (!get_input_states_file().empty()) {
        assembleStatesTrajectoryFromStatesData(input_data, state);
    }

    setupLigamentStorage();

    setupContactStorage(state);

    setupMuscleStorage();

    setupAttachedGeometriesStorage();

    if (get_h5_kinematics_data()) {
        setupCoordinateStorage();
    }

    if (get_write_transforms_file()) {        

        std::vector<std::string> column_labels;

        for (const Frame& frame : _model.updComponentList<Frame>()) {
            std::string name = frame.getName();

            //use non-c index convention in labels
            for (int m = 1; m < 5; ++m) {
                for (int n = 1; n < 5; ++n) {
                    std::string col_label =
                        name + "_T" + std::to_string(m) + std::to_string(n);
                    column_labels.push_back(col_label);
                }
            }
        }
        _model_frame_transforms.setColumnLabels(column_labels);
    }


}

void JointMechanicsTool::clearInitializedMemberData(){
    _contact_force_names.clear();
    _contact_force_paths.clear();

    _states.clear();
    _time = Array<double>();
       

    _q_matrix.clear();
    _u_matrix.clear();

    _contact_force_names.clear();
    _contact_force_paths.clear();
    _contact_mesh_names.clear();
    _contact_mesh_paths.clear();
    _mesh_vertex_locations.clear();
    
    _contact_output_double_names.clear();
    _contact_output_vec3_names.clear();
    _contact_output_vector_double_names.clear();
    _contact_output_vector_vec3_names.clear();
    _contact_output_double_values.clear();
    _contact_output_vec3_values.clear();
    _contact_output_vector_double_values.clear();
    _contact_output_vector_vec3_values.clear();

    _attach_geo_names.clear();
    _attach_geo_frames.clear();
    _attach_geo_meshes.clear();
    _attach_geo_vertex_locations.clear();  
    

    _ligament_names.clear();
    _ligament_paths.clear();
    _ligament_path_nPoints.clear();
    _ligament_path_points.clear();
    _ligament_output_double_names.clear();
    _ligament_output_double_values.clear();

    _muscle_names.clear();
    _muscle_paths.clear();
    _muscle_path_nPoints.clear();
    _muscle_path_points.clear();
    _muscle_output_double_names.clear();
    _muscle_output_double_values.clear();

    _muscle_state_names.clear();
    _muscle_state_data.clear();

    _coordinate_names.clear();
    _coordinate_output_double_names.clear();
    _coordinate_output_double_values.clear();

    _model_frame_transforms = TimeSeriesTable();

    _mesh_names.clear();
    _mesh_transforms.clear();
}

Storage JointMechanicsTool::processInputStorage(std::string file) {
    
    std::string saveWorkingDirectory = IO::getCwd();
    IO::chDir(_directoryOfSetupFile);
    Storage store = Storage(file);
    IO::chDir(saveWorkingDirectory);

    //Set Start and Stop Times
    store.getTimeColumn(_time);
     
    if (get_start_time() == -1) {
        set_start_time(_time.get(0));
    }
    if (get_stop_time() == -1) {
        set_stop_time(_time.getLast());
    }
    
    if (store.isInDegrees()) {
        _model.getSimbodyEngine().convertDegreesToRadians(store);
    }

    else if (get_lowpass_filter_frequency() != -1) {
        store.pad(store.getSize() / 2);
        store.lowpassIIR(get_lowpass_filter_frequency());
    }

    //Cut to start and stop times
    store.crop(get_start_time(), get_stop_time());

    if (get_normalize_to_cycle() == true) {
        double norm_dt = (get_stop_time() - get_start_time()) / 100;
        store.resampleLinear(norm_dt);
    }
    else if (get_resample_step_size() != -1 && get_normalize_to_cycle() == false) {
        store.resampleLinear(get_resample_step_size());
    }
    
    //Update the time 
    store.getTimeColumn(_time);

    //Set number of Frames
    _n_frames = _time.size();

    return store;
}

void JointMechanicsTool::assembleStatesTrajectoryFromTransformsData(
    const Storage& storage, SimTK::State s) {
    //Make a copy of the model so we can make changes
    Model working_model = *_model.clone();
    working_model.set_assembly_accuracy(1e-8);
    working_model.setUseVisualizer(false);
    working_model.initSystem();
    Storage transforms_storage = processInputStorage(get_input_transforms_file());

    Array<std::string> column_labels =
        transforms_storage.getColumnLabels();

    std::vector<std::string> body_names(column_labels.size());

    for (int i = 0; i < column_labels.getSize(); ++i) {
        std::string label = column_labels.get(i);
        body_names[i] = label.erase(label.length() - 4);
    }

    // Add a reporter to get IK computed coordinate values out for 
    // orginal joints in the model 

    TableReporter* ikReporter = new TableReporter();
    ikReporter->setName("ik_reporter");

    for (auto& coord : working_model.getComponentList<Coordinate>()) {
        ikReporter->updInput("inputs").connect(
            coord.getOutput("value"), 
            coord.getAbsolutePathString() + "/value");

        ikReporter->updInput("inputs").connect(
            coord.getOutput("speed"),
            coord.getAbsolutePathString() + "/speed");
    }
    working_model.addComponent(ikReporter);

    // Add Hidden Bodies
    int num_hidden_bodies=0;
    std::vector<std::string> hidden_joint_paths;
    std::vector<std::string> hidden_partner_body_path;
    for (Body& body : working_model.updComponentList<Body>()) {
        int index = 1;
        if (contains_string(body_names, body.getName(), index)) {
            num_hidden_bodies++;

            Body* hidden_body = new Body(body.getName() + 
                "_hidden", 1e-6, SimTK::Vec3(0.0), SimTK::Inertia(1e-6));

            SpatialTransform spat_trans;

            spat_trans[0].setCoordinateNames(OpenSim::Array<std::string>(
                body.getName() + "_ground_r1", 1, 1));
            spat_trans[0].setFunction(new LinearFunction());
            spat_trans[0].setAxis(SimTK::Vec3(1, 0, 0));

            spat_trans[1].setCoordinateNames(OpenSim::Array<std::string>(
                body.getName() + "_ground_r2", 1, 1));
            spat_trans[1].setFunction(new LinearFunction());
            spat_trans[1].setAxis(SimTK::Vec3(0, 1, 0));

            spat_trans[2].setCoordinateNames(OpenSim::Array<std::string>(
                body.getName() + "_ground_r3", 1, 1));
            spat_trans[2].setFunction(new LinearFunction());
            spat_trans[2].setAxis(SimTK::Vec3(0, 0, 1));

            spat_trans[3].setCoordinateNames(OpenSim::Array<std::string>(
                body.getName() + "_ground_t1", 1, 1));
            spat_trans[3].setFunction(new LinearFunction());
            spat_trans[3].setAxis(SimTK::Vec3(1, 0, 0));

            spat_trans[4].setCoordinateNames(OpenSim::Array<std::string>(
                body.getName() + "_ground_t2", 1, 1));
            spat_trans[4].setFunction(new LinearFunction());
            spat_trans[4].setAxis(SimTK::Vec3(0, 1, 0));

            spat_trans[5].setCoordinateNames(OpenSim::Array<std::string>(
                body.getName() + "_ground_t3", 1, 1));
            spat_trans[5].setFunction(new LinearFunction());
            spat_trans[5].setAxis(SimTK::Vec3(0, 0, 1));

            CustomJoint* ground_hidden_joint = new CustomJoint(
                body.getName() + "_hidden_ground",  working_model.updGround(),
                *hidden_body, spat_trans);
                
            WeldConstraint* hidden_weld = new WeldConstraint(
                body.getName() + "_hidden", body, SimTK::Transform(),
                *hidden_body, SimTK::Transform());

            working_model.addBody(hidden_body);
            working_model.addConstraint(hidden_weld);
            working_model.addJoint(ground_hidden_joint);

            hidden_partner_body_path.push_back(
                body.getAbsolutePathString());

            hidden_joint_paths.push_back(
                ground_hidden_joint->getAbsolutePathString());
        }
    }
    /*if (get_use_visualizer()) {
        working_model.setUseVisualizer(true);
    }*/

    SimTK::State state = working_model.initSystem();

    //Collect Transformation matrix values
    std::vector<GCVSplineSet> hidden_coord_splines;

    std::vector<std::string> hidden_coord_labels;
    hidden_coord_labels.push_back("rX");
    hidden_coord_labels.push_back("rY");
    hidden_coord_labels.push_back("rZ");
    hidden_coord_labels.push_back("tX");
    hidden_coord_labels.push_back("tY");
    hidden_coord_labels.push_back("tZ");

    std::vector<double> time(_time.size());
    for (int t = 0; t < _time.size(); t++) {
        time[t] = _time.get(t);
    }

    TimeSeriesTable coords_table(time,SimTK::Matrix(_n_frames,6,0.0),
        hidden_coord_labels);

    for (int i = 0; i < num_hidden_bodies; ++i) {

        const Body& partner_body =
            working_model.updComponent<Body>(hidden_partner_body_path[i]);

        coords_table.updMatrix() = 0.0;
        for (int t = 0; t < transforms_storage.getSize(); ++t) {
            SimTK::Mat33 R_matrix(0.0);
                    
            for (int m = 0; m < 3; ++m) {
                for (int n = 0; n < 3; ++n) {

                    std::string TXX_label =
                        partner_body.getName() + "_T" +
                        std::to_string(m + 1) + std::to_string(n + 1);

                    int col_index = column_labels.findIndex(TXX_label);
                    col_index -= 1; //time listed in labels, not in data
                    if (col_index > -2) {
                        double value; 
                        transforms_storage.getData(t, col_index, value);
                        R_matrix(m, n) = value;
                    }
                    else {
                        OPENSIM_THROW(Exception, TXX_label +
                            "not found in 'transforms_file'.");
                    }
                }
            }

            SimTK::Rotation rot = SimTK::Rotation(R_matrix);
            SimTK::Vec3 xyz_rot = 
                rot.convertThreeAxesRotationToThreeAngles(
                SimTK::BodyOrSpaceType::BodyRotationSequence,
                SimTK::CoordinateAxis(0),SimTK::CoordinateAxis(1),
                SimTK::CoordinateAxis(2));

            //Translations
            SimTK::Vec3 xyz_trans(0);
            for (int m = 0; m < 3; ++m) {

                std::string TXX_label =
                    partner_body.getName() + "_T" + 
                    std::to_string(m + 1) + std::to_string(4);

                int col_index = column_labels.findIndex(TXX_label);
                col_index -= 1; //time listed in labels, not in data

                if (col_index>-2) {
                    double value;
                    transforms_storage.getData(t, col_index,value);
                    xyz_trans(m) = value; 
                }
                else {
                    OPENSIM_THROW(Exception, TXX_label +
                        "not found in 'transforms_file'.");
                }
            }

            SimTK::RowVector coord_values(6);
            for (int j = 0; j < 3; ++j) {
                coord_values[j] = xyz_rot(j);
                coord_values[j+3] = xyz_trans(j);
            }
            coords_table.setRowAtIndex(t, coord_values);
        }
        hidden_coord_splines.push_back(GCVSplineSet(coords_table));
    }
        
    //Use IK to compute org joint angles
    MarkersReference markersReference;
    SimTK::Array_<CoordinateReference> coordinateReferences;

    for (int i = 0; i < num_hidden_bodies; ++i) {
            CustomJoint& hidden_joint = 
                working_model.updComponent<CustomJoint>(hidden_joint_paths[i]);

        for (int j = 0; j < 6; ++j) {
            std::string name = hidden_joint.get_coordinates(j).getName();
            std::cout <<  hidden_joint.get_coordinates(j).getName() << std::endl;
            CoordinateReference* coordRef = 
                new CoordinateReference(name, hidden_coord_splines[i].get(j));
            coordRef->setWeight(1);
            coordinateReferences.push_back(*coordRef);
        }
    }

    // create the solver given the input data
    InverseKinematicsSolver ikSolver(working_model, markersReference,
        coordinateReferences, 1e8);

    ikSolver.setAccuracy(1e-12);
    

    state.updTime() = _time[0];
    ikSolver.assemble(state);
    
    /*SimTK::Visualizer* viz=NULL;
    if (get_use_visualizer()) {
        viz = &working_model.updVisualizer().updSimbodyVisualizer();
        viz->setShowSimTime(true);
    }*/
    double transform_assembly_threshold = 1e-6;

    for (int t = 0; t < _n_frames; ++t) {
        state.updTime() = _time[t];
        double max_error = transform_assembly_threshold;
        bool first_loop = true;
        
        while (max_error >= transform_assembly_threshold) {
            for (int i = 0; i < num_hidden_bodies; ++i) {
                CustomJoint& hidden_joint =
                    working_model.updComponent<CustomJoint>(hidden_joint_paths[i]);

                for (int j = 0; j < 6; ++j) {
                    double value = hidden_coord_splines[i].get(j).calcValue(SimTK::Vector(1, time[t]));
                    hidden_joint.get_coordinates(j).setValue(state, value, false);
                }
            }
            //working_model.assemble(state);
            //if (first_loop) {
                ikSolver.assemble(state);
                //first_loop = false;
            //}
            ikSolver.track(state);

            max_error = 0.0;

            for (int i = 0; i < num_hidden_bodies; ++i) {
                CustomJoint& hidden_joint =
                    working_model.updComponent<CustomJoint>(hidden_joint_paths[i]);

                for (int j = 0; j < 6; ++j) {
                    double desired_value = hidden_coord_splines[i].get(j).calcValue(SimTK::Vector(1, time[t]));
                    double assembled_value = hidden_joint.get_coordinates(j).getValue(state);
                    double value_error = abs(desired_value - assembled_value);
                    if (value_error > max_error) {
                        max_error = value_error;
                        
                    }
                }
            }
            /*if (get_use_visualizer()) {
                viz->drawFrameNow(state);
            }
            for (auto& coord : working_model.getComponentList<Coordinate>() ){
                std::cout << coord.getName() << " " << coord.getValue(state) << std::endl;
            }*/
        }

        //save pose
        working_model.realizeReport(state);
    }
    TimeSeriesTable coordinate_states_table = ikReporter->getTable();

    for (int t = 0; t < _n_frames; ++t) {
        s.setTime(_time[t]);
        for (auto& coord : _model.updComponentList<Coordinate>()) {
            std::string path = coord.getAbsolutePathString();

            int value_index = (int)coordinate_states_table.getColumnIndex(
                path + "/value");

            int speed_index = (int)coordinate_states_table.getColumnIndex(
                path + "/speed");

            double value = coordinate_states_table.getMatrix()(t, value_index);
            double speed = coordinate_states_table.getMatrix()(t, speed_index);
            coord.setValue(s, value, false);
        }
        _model.assemble(s);
        _states.push_back(s);
    }
}

void JointMechanicsTool::assembleStatesTrajectoryFromStatesData(
    const Storage& storage, SimTK::State s) {

    Storage store = processInputStorage(get_input_states_file());
    
    if (get_print_processed_kinematics()) {
        store.print(get_results_directory() + "/" + 
            get_results_file_basename() + "_processed_kinematics.sto");
    }



    TimeSeriesTable table = store.exportToTable();

    // The labels of the columns in the storage file.
    const auto& tableLabels = table.getColumnLabels();
    int numDependentColumns = (int)table.getNumColumns();

    // Error checking.
    // ===============
    // Angular quantities must be expressed in radians.
    // TODO we could also manually convert the necessary coords/speeds to
    // radians.
    OPENSIM_THROW_IF(TableUtilities::isInDegrees(table), Exception, 
        "JointMechanicsTool input_states_file cannot be in degrees.");

    // If column labels aren't unique, it's unclear which column the user
    // wanted to use for the related state variable.
    TableUtilities::checkNonUniqueLabels(tableLabels);

    // Check if states are missing from the Storage.
    // ---------------------------------------------
    const auto& modelStateNames = _model.getStateVariableNames();
    // Working memory for state. Initialize so that missing columns end up as
    // NaN.
    SimTK::Vector statesValues(modelStateNames.getSize(), SimTK::NaN);

    // Initialize so that missing columns end up as NaN.
    s.updY().setToNaN();
    SimTK::State default_state = _model.getWorkingState();
    SimTK::Vector default_state_values = 
        _model.getStateVariableValues(default_state);

    std::vector<std::string> missingColumnNames;
    // Also, assemble the indices of the states that we will actually set in the
    // trajectory.
    std::map<int, int> statesToFillUp;

    for (int is = 0; is < modelStateNames.getSize(); ++is) {
        // getStateIndex() will check for pre-4.0 column names.
        const int stateIndex = TableUtilities::findStateLabelIndex(
                tableLabels, modelStateNames[is]);
        //const int stateIndex = findStateLabelIndexInternal(
        //       tableLabels.data(), tableLabels.data() + tableLabels.size(),
        //        modelStateNames[is]);
        if (stateIndex == -1) {
            missingColumnNames.push_back(modelStateNames[is]);
            statesValues[is] = default_state_values[is];
        } else {
            statesToFillUp[stateIndex] = is;
        }
    }

    for (auto mc : missingColumnNames) {
        log_warn("JointMechanicsTool input_states_file missing state: {}", mc);
    }
    // Fill up trajectory from states in file

    // Reserve the memory we'll need to fit all the states.
    //_states.m_states.reserve(table.getNumRows());



    SimTK::Matrix msl_activations
        ((int)table.getNumRows(), _model.getMuscles().getSize(),-1);

    // Loop through all rows of the Storage
    StatesTrajectory states_from_file;

    for (int itime = 0; itime < (int)table.getNumRows(); ++itime) {
        const auto& row = table.getRowAtIndex(itime);

        // Set the correct time in the state.
        s.setTime(table.getIndependentColumn()[itime]);

        // Fill up current State with the data for the current time.
        for (const auto& kv : statesToFillUp) {
            // 'first': index for Storage; 'second': index for Model.
            statesValues[kv.second] = row[kv.first];
        }
        _model.setStateVariableValues(s, statesValues);
        /*if (assemble) {
            localModel.assemble(state);
        }*/

        // Make a copy of the edited state and put it in the trajectory.
        _model.realizeVelocity(s);
        states_from_file.append(s);

        // Save muscle activation values in case use_muscle_physiology is false
        //_model.realizeVelocity(s);
        int j=0;
        for (auto& msl : _model.updComponentList<Muscle>()) {
            //UGLY way to get around when no muscle states are input and fiber length is zero
            //try {
                msl_activations(itime, j) = msl.getActivation(s);
            //}
            /*catch (Exception) {
                _model.equilibrateMuscles(s);
                msl_activations(itime, j) = msl.getActivation(s);
            }*/
            j++;
        }
    }

    std::vector<std::string> activation_paths;
    std::vector<SimTK::Vector> activation_data;
    
    std::vector<std::string> force_paths;
    std::vector<SimTK::Vector> force_data;

    // Read activations file
    if (!get_input_activations_file().empty()) {
        Storage store = processInputStorage(get_input_activations_file());
        //Storage store = Storage(get_input_states_file());
        int j = 0;
        for (Muscle& msl : _model.updComponentList<Muscle>()) {

            std::string msl_path = msl.getAbsolutePathString();
            Array<int> column_index = 
                store.getColumnIndicesForIdentifier(msl_path);

            if (column_index.size() > 0) {
                double* Tdata=NULL;
                
                //std::cout << msl_path << std::endl;
                
                store.getDataColumn(msl_path, Tdata);

                for (int t = 0; t < store.getSize(); ++t) {
                    msl_activations(t, j) = Tdata[t];
                }
            }
            else {
                log_warn("Muscle: {} was not listed in activations file.", msl.getAbsolutePathString());
            }

            j++;
        }
    }


    // Read forces file
    if (!get_input_forces_file().empty()) {
        Storage store = processInputStorage(get_input_forces_file());
        
        for (ScalarActuator& actuator : _model.updComponentList<ScalarActuator>()) {

            std::string actuator_path = actuator.getAbsolutePathString();
            Array<int> column_index = 
                store.getColumnIndicesForIdentifier(actuator_path);

            if (column_index.size() > 0) {
                force_paths.push_back(actuator_path);
                double* data=NULL;
                
                store.getDataColumn(actuator_path, data);
                SimTK::Vector data_vec(store.getSize(), data);
                force_data.push_back(data_vec);
            }

        }
    }



    // Overide forces in states if settings dictate
    int i = 0;
    for (SimTK::State final_state : states_from_file) {
        if (!get_input_activations_file().empty()) {
            int j = 0;
            for (auto& msl : _model.updComponentList<Muscle>()) {
                msl.setActivation(final_state, msl_activations(i, j));
                ++j;
                //msl.set(final_state, msl_activations(i, j));

                /*msl.overrideActuation(final_state, true);

                double force =
                    msl_activations(i,j) * msl.getMaxIsometricForce();
                msl.setOverrideActuation(final_state, force);*/
            }
        }

        if (!get_use_muscle_physiology()) {
            int j = 0;
            for (auto& msl : _model.updComponentList<Muscle>()) {
                msl.overrideActuation(final_state, true);

                double force =
                    msl_activations(i,j) * msl.getMaxIsometricForce();
                msl.setOverrideActuation(final_state, force);
                ++j;
            }
        }

        if (!get_input_forces_file().empty()) {
            int nForces = 0;
            for (const auto& path : force_paths) {
                ScalarActuator& actuator = 
                    _model.updComponent<ScalarActuator>(path);

                actuator.overrideActuation(final_state, true);
                actuator.setOverrideActuation(final_state, force_data[nForces](i));
                nForces++;
            }
        }

        final_state.invalidateAllCacheAtOrAbove(SimTK::Stage::Time);
        //_states.append(final_state);
        _states.push_back(final_state);
        ++i;
    }
}

void JointMechanicsTool::setupContactStorage(SimTK::State& state) {
    if (_model.countNumComponents<Smith2018ArticularContactForce>() == 0) {
        return;
    }
    //Contact Names
    if (getProperty_contacts().size() == 0 || get_contacts(0) == "none") {
        return;
    }
    else if (get_contacts(0) == "all") {
        for (const auto& contactForce : 
            _model.getComponentList<Smith2018ArticularContactForce>()) {
            _contact_force_names.push_back(contactForce.getName());
            _contact_force_paths.push_back(
                contactForce.getAbsolutePathString());

            std::string casting_mesh_name = 
                contactForce.getConnectee<Smith2018ContactMesh>(
                    "casting_mesh").getName();

            std::string target_mesh_name = 
                contactForce.getConnectee<Smith2018ContactMesh>(
                    "target_mesh").getName();

            std::string casting_mesh_path = 
                contactForce.getConnectee<Smith2018ContactMesh>(
                    "casting_mesh").getAbsolutePathString();

            std::string target_mesh_path = 
                contactForce.getConnectee<Smith2018ContactMesh>(
                    "target_mesh").getAbsolutePathString();
            
            if (!contains_string(_contact_mesh_names, casting_mesh_name)) {
                _contact_mesh_names.push_back(casting_mesh_name);
                _contact_mesh_paths.push_back(casting_mesh_path);
            }
            if (!contains_string(_contact_mesh_names, target_mesh_name)) {
                _contact_mesh_names.push_back(target_mesh_name);
                _contact_mesh_paths.push_back(target_mesh_path);
            }
        }
    }
    else {
        for (int i = 0; i < getProperty_contacts().size(); ++i) {
            try {
                const auto& contactForce = 
                    _model.getComponent
                    <Smith2018ArticularContactForce>(get_contacts(i));

                _contact_force_names.push_back(contactForce.getName());
                _contact_force_paths.push_back(
                    contactForce.getAbsolutePathString());

                std::string casting_mesh_name = 
                    contactForce.getConnectee<Smith2018ContactMesh>
                    ("casting_mesh").getName();

                std::string target_mesh_name = 
                    contactForce.getConnectee<Smith2018ContactMesh>
                    ("target_mesh").getName();

                std::string casting_mesh_path = 
                    contactForce.getConnectee<Smith2018ContactMesh>
                    ("casting_mesh").getAbsolutePathString();

                std::string target_mesh_path = 
                    contactForce.getConnectee<Smith2018ContactMesh>
                    ("target_mesh").getAbsolutePathString();
                
                if (!contains_string(_contact_mesh_names, casting_mesh_name)) {
                    _contact_mesh_names.push_back(casting_mesh_name);
                    _contact_mesh_paths.push_back(casting_mesh_path);
                }

                if (!contains_string(_contact_mesh_names, target_mesh_name)) {
                    _contact_mesh_names.push_back(target_mesh_name);
                    _contact_mesh_paths.push_back(target_mesh_path);
                }
            }
            catch (ComponentNotFoundOnSpecifiedPath){
                OPENSIM_THROW(Exception, "contact_name: " + get_contacts(i)
                    + " was not found as a Smith2018ArticularContactForce path" 
                    " in the model. Did you use absolute path?");
            }
        }
    }

    //Turn on mesh flipping so metrics are computed for casting and target
    /*for (int i = 0; i < (int)_contact_force_paths.size(); ++i) {

        Smith2018ArticularContactForce& contactForce = _model.updComponent
            <Smith2018ArticularContactForce>(_contact_force_paths[i]);

        contactForce.setModelingOption(state, "flip_meshes", 1);
    }*/

    //Realize Report so the sizes of output vectors are known
    _model.realizeReport(state);
    
    //Contact Outputs
    const Smith2018ArticularContactForce& frc0 = _model.getComponent
        <Smith2018ArticularContactForce>(_contact_force_paths[0]);

    if (get_contact_outputs(0) == "all") {
        for (const auto& entry : frc0.getOutputs()) {
            const std::string& output_name = entry.first;
            const AbstractOutput* output = entry.second.get();

            if (output->isListOutput()) { continue; }

            if (output->getTypeName() == "double") {
                _contact_output_double_names.push_back(output->getName());
            }
            else if (output->getTypeName() == "Vec3") {
                _contact_output_vec3_names.push_back(output->getName());
            }
            else if (output->getTypeName() == "Vector") {
                _contact_output_vector_double_names.push_back(
                    output->getName());
            }
            else if (output->getTypeName() == "Vector_<Vec3>") {
                _contact_output_vector_vec3_names.push_back(
                    output->getName());
            }

        }
    }
    else if (get_contact_outputs(0) == "htc") {
        for (const auto& entry : frc0.getOutputs()) {
            const std::string& output_name = entry.first;
            const AbstractOutput* output = entry.second.get();

            if(output->isListOutput()){continue;}

            if (output->getTypeName() == "double") {
                _contact_output_double_names.push_back(output->getName());
            }
            if (output->getTypeName() == "Vec3") {
                _contact_output_vec3_names.push_back(output->getName());
            }
            if (output->getTypeName() == "Vector") {
                if (output->getName() == "casting_triangle_proximity" ||
                    output->getName() == "target_triangle_proximity" ||
                    output->getName() == "casting_triangle_pressure" ||
                    output->getName() == "target_triangle_pressure" ||
                    output->getName() == "casting_triangle_potential_energy" ||
                    output->getName() == "target_triangle_potential_energy") {
                    continue;
                }
                _contact_output_vector_double_names.push_back(
                    output->getName());
            }
        }
    }
    else if (getProperty_contact_outputs().size() != 0 &&
        get_contact_outputs(0) != "none") {
        
        for (int i = 0; i < getProperty_contact_outputs().size(); ++i) {
            try {
                std::string output_name = get_contact_outputs(i);
                const AbstractOutput& output = frc0.getOutput(output_name);

                if (output.getTypeName() == "double") {
                    _contact_output_double_names.push_back(output_name);
                }
                else if (output.getTypeName() == "Vec3") {
                    _contact_output_vec3_names.push_back(output_name);
                }
                else if (output.getTypeName() == "Vector") {
                    _contact_output_vector_double_names.push_back(output_name);
                }
                else if (output.getTypeName() == "Vector_<Vec3>") {
                    _contact_output_vector_vec3_names.push_back(output_name);
                }
            }
            catch (Exception){
                OPENSIM_THROW(Exception, "contact_output: " + 
                    get_contact_outputs(i) + " is not a valid "
                    "Smith2018ArticularContactForce output name")
            }
        }
    }

    // Allocate Output Storage
    int nOutputDouble = (int)_contact_output_double_names.size();
    int nOutputVec3 = (int)_contact_output_vec3_names.size();
    int nOutputVector = (int)_contact_output_vector_double_names.size();
    int nOutputVectorVec3 = (int)_contact_output_vector_vec3_names.size();

    SimTK::Matrix double_data(_n_frames, nOutputDouble,-1);
    SimTK::Matrix_<SimTK::Vec3> 
        vec3_data(_n_frames, nOutputVec3,SimTK::Vec3(-1));

    for (std::string frc_path : _contact_force_paths) {
        const Smith2018ArticularContactForce& frc = 
            _model.updComponent<Smith2018ArticularContactForce>(frc_path);

        _contact_output_double_values.push_back(double_data);
        _contact_output_vec3_values.push_back(vec3_data);

        std::vector<SimTK::Matrix> def_output_vector;

        for (int i = 0; i < nOutputVector; ++i) {
            
            const AbstractOutput& abs_output = 
                frc.getOutput(_contact_output_vector_double_names[i]);
            
            const Output<SimTK::Vector>& vector_output = 
                dynamic_cast<const Output<SimTK::Vector>&>(abs_output);

            int output_vector_size = vector_output.getValue(state).size();
            
            def_output_vector.push_back(
                SimTK::Matrix(_n_frames, output_vector_size,-1));
        }
        _contact_output_vector_double_values.push_back(def_output_vector);

        std::vector<SimTK::Matrix_<SimTK::Vec3>> def_output_vector_vec3;

        for (int i = 0; i < nOutputVectorVec3; ++i) {
            
            const AbstractOutput& abs_output = 
                frc.getOutput(_contact_output_vector_vec3_names[i]);
            
            const Output<SimTK::Vector_<SimTK::Vec3>>& vector_output = 
                dynamic_cast<const Output<SimTK::Vector_<SimTK::Vec3>>&>(abs_output);

            int output_vector_size = vector_output.getValue(state).size();
            
            def_output_vector_vec3.push_back(
                SimTK::Matrix_<SimTK::Vec3>(_n_frames, output_vector_size,SimTK::Vec3(-1.5)));
        }
        _contact_output_vector_vec3_values.push_back(def_output_vector_vec3);
    }


    
    //Vertex location storage
    _mesh_vertex_locations.resize(_contact_mesh_paths.size());

    for (int i = 0; i < (int)_contact_mesh_paths.size(); ++i) {

        int mesh_nVer = _model.getComponent<Smith2018ContactMesh>
            (_contact_mesh_paths[i]).getPolygonalMesh().getNumVertices();

        _mesh_vertex_locations[i].resize(_n_frames, mesh_nVer);
    }

    //Mesh Transform Storage
    _mesh_transforms.resize(_contact_mesh_paths.size());

    for (int i = 0; i < (int)_contact_mesh_paths.size(); ++i) {
        _mesh_transforms[i].resize(_n_frames, 16);
        _mesh_transforms[i] = 0;
    }
}

void JointMechanicsTool::setupAttachedGeometriesStorage() {
    std::vector<std::string> body_path_list;

    if (get_attached_geometry_bodies(0) == "none" ||
        getProperty_attached_geometry_bodies().empty()) {
        return;
    }
    else if (get_attached_geometry_bodies(0) == "all") {
        for (const Frame& frame : _model.updComponentList<Frame>()) {
            body_path_list.push_back(frame.getAbsolutePathString());
        }
    }
    else {
        int nAttachedGeoBodies = getProperty_attached_geometry_bodies().size();
        for (int i = 0; i < nAttachedGeoBodies; ++i) {
            try {
                const Frame& frame = _model.updComponent<Frame>
                    (get_attached_geometry_bodies(i));
                body_path_list.push_back(frame.getAbsolutePathString());
            }
            catch (Exception) {
                OPENSIM_THROW(Exception, "attached_geometry_bodies: " +
                    get_attached_geometry_bodies(i) + "does not exist as a "
                    "Frame component in model. Did you use Absolute Path?");
            }
        }
    }

    for (std::string body_path : body_path_list){
        const Frame& frame = _model.updComponent<Frame>(body_path);

        int nAttachedGeos = frame.getProperty_attached_geometry().size();
        for (int i = 0; i < nAttachedGeos; ++i) {
            const Geometry& geo = frame.get_attached_geometry(i);

            if (geo.getConcreteClassName() != "Mesh") {
                continue;
            }

            if (contains_string(_attach_geo_names, geo.getName())) {
                continue;
            }

            Mesh* mesh = (Mesh*)&geo;
            std::string filename = findMeshFile(mesh->get_mesh_file());

            SimTK::PolygonalMesh ply_mesh;
            ply_mesh.loadFile(filename);

            //Apply Scale Factors
            SimTK::Vec3 scale = mesh->get_scale_factors();
            if (scale != SimTK::Vec3(1)) {
                SimTK::PolygonalMesh scaled_mesh;

                for (int v = 0; v < ply_mesh.getNumVertices(); ++v) {
                    scaled_mesh.addVertex(ply_mesh.
                        getVertexPosition(v).elementwiseMultiply(scale));
                }

                for (int f = 0; f < ply_mesh.getNumFaces(); ++f) {
                    SimTK::Array_<int> facevertex;
                    int numVertex = ply_mesh.getNumVerticesForFace(f);

                    for (int k = 0; k < numVertex; ++k) {
                        facevertex.push_back(ply_mesh.getFaceVertex(f, k));
                    }
                    scaled_mesh.addFace(facevertex);
                }
                ply_mesh.copyAssign(scaled_mesh);
            }

            _attach_geo_names.push_back(geo.getName());
            _attach_geo_frames.push_back(frame.getAbsolutePathString());
            _attach_geo_meshes.push_back(ply_mesh);
            _attach_geo_vertex_locations.push_back(
                SimTK::Matrix_<SimTK::Vec3>
                (_n_frames, ply_mesh.getNumVertices()));
        }
    }
}

std::string JointMechanicsTool::findMeshFile(const std::string& mesh_file) {
    
    std::string model_file = 
        SimTK::Pathname::getAbsolutePathname(get_model_file());
    
    std::string model_dir, dummy1, dummy2;
    bool dummyBool;
    SimTK::Pathname::deconstructPathname(model_file, 
        dummyBool, model_dir, dummy1, dummy2);

    std::string mesh_full_path = mesh_file;

    std::ifstream file(mesh_file);
    if (!file) {
        mesh_full_path = model_dir + mesh_file;
        file.open(mesh_full_path);
    }
    if (!file) {
        mesh_full_path = model_dir + "Geometry/" + mesh_file;
        file.open(mesh_full_path);
    }

    if (!file) {
        OPENSIM_THROW(Exception, "Attached Geometry file doesn't exist:\n" 
            + model_dir + "[Geometry/]" + mesh_file);
    }

    return mesh_full_path;
}

void JointMechanicsTool::setupLigamentStorage() {
    if (_model.countNumComponents<Blankevoort1991Ligament>() == 0) return;

    //Ligament Names
    if (getProperty_ligaments().size() == 0 || get_ligaments(0) == "none") {
        return;
    }
    else if (get_ligaments(0) == "all") {
        for (const Blankevoort1991Ligament& lig :
            _model.updComponentList<Blankevoort1991Ligament>()) {

            _ligament_names.push_back(lig.getName());
            _ligament_paths.push_back(lig.getAbsolutePathString());
        }
    }
    else
    {
        for (int i = 0; i < getProperty_ligaments().size(); ++i)
        {
            try {
                const Blankevoort1991Ligament& lig = _model.updComponent
                    <Blankevoort1991Ligament>(get_ligaments(i));

                _ligament_names.push_back(lig.getName());
                _ligament_paths.push_back(lig.getAbsolutePathString());
            }
            catch (ComponentNotFoundOnSpecifiedPath) {
                OPENSIM_THROW(Exception, "ligament: " + get_ligaments(i) +
                    " was not found in the model. "
                    "Are you using the absolute path?");
            }
        }
    }
        
    //Ligament Outputs
    const Blankevoort1991Ligament& lig0 = _model.
        updComponentList<Blankevoort1991Ligament>().begin().deref();

    if (get_ligament_outputs(0) == "all") {
        for (const auto& entry : lig0.getOutputs()) {
            const AbstractOutput* output = entry.second.get();
            if(output->isListOutput()){continue;}
            if (output->getTypeName() == "double") {
                _ligament_output_double_names.push_back(output->getName());
            }
        }

    }
    else if (getProperty_ligament_outputs().size() != 0 &&
        get_ligament_outputs(0) != "none") {
        
        for (int i = 0; i < getProperty_ligament_outputs().size(); ++i) {
            try {
                std::string output_name = get_ligament_outputs(i);
                lig0.getOutput(output_name);
                _ligament_output_double_names.push_back(output_name);
            }
            catch (Exception){
                OPENSIM_THROW(Exception, "ligament_output: " + 
                    get_ligament_outputs(i) + " is not a valid "
                    "Blankevoort1991Ligament output name")
            }
        }
    }

    int nLigamentOutputs = (int)_ligament_output_double_names.size();
    SimTK::Matrix lig_output_data(_n_frames, nLigamentOutputs,-1);

    //Ligament Storage
    for (std::string lig_path : _ligament_paths) {
        Blankevoort1991Ligament lig = 
            _model.updComponent<Blankevoort1991Ligament>(lig_path);

        //Path Point Storage
        SimTK::Matrix_<SimTK::Vec3> lig_matrix(_n_frames, 
            _max_path_points, SimTK::Vec3(-1));
        SimTK::Vector lig_vector(_n_frames, -1);

        _ligament_path_points.push_back(lig_matrix);
        _ligament_path_nPoints.push_back(lig_vector);

        //Output Data Storage
        _ligament_output_double_values.push_back(lig_output_data);
    }
}

void JointMechanicsTool::setupMuscleStorage() {
    if (_model.countNumComponents<Muscle>() == 0) return;

    //Muscle Names
    if (getProperty_muscles().size() == 0 || get_muscles(0) == "none") {
        return;
    }
    else if (get_muscles(0) == "all") {
        for (const Muscle& msl :
            _model.updComponentList<Muscle>()) {

            _muscle_names.push_back(msl.getName());
            _muscle_paths.push_back(msl.getAbsolutePathString());
        }
    }
    else
    {
        for (int i = 0; i < getProperty_muscles().size(); ++i)
        {
            try {
                const Muscle& msl = _model.updComponent
                    <Muscle>(get_muscles(i));

                _muscle_names.push_back(msl.getName());
                _muscle_paths.push_back(msl.getAbsolutePathString());
            }
            catch (ComponentNotFoundOnSpecifiedPath) {
                OPENSIM_THROW(Exception, "Muscle: " + get_muscles(i) +
                    " was not found in the model. "
                    "Are you using the absolute path?");
            }
        }
    }

    //Muscle Outputs
    const Muscle& msl0 = _model.getMuscles().get(0);
    
    if (!get_use_muscle_physiology() && get_muscle_outputs(0) == "all") {
        _muscle_output_double_names.push_back("activation");
        _muscle_output_double_names.push_back("actuation");
        _muscle_output_double_names.push_back("power");
        _muscle_output_double_names.push_back("speed");
        _muscle_output_double_names.push_back("length");
    }
    else if (get_muscle_outputs(0) == "all") {
        for (const auto& entry : msl0.getOutputs()) {
            const AbstractOutput* output = entry.second.get();
            if(output->isListOutput()){continue;}
            if (output->getTypeName() == "double") {
                _muscle_output_double_names.push_back(output->getName());

            }
        }
        _muscle_output_double_names.push_back("length");
    }
    else if (getProperty_muscle_outputs().size() != 0 &&
                get_muscle_outputs(0) != "none") {
        
        for (int i = 0; i < getProperty_muscle_outputs().size(); ++i) {
            std::string output_name = get_muscle_outputs(i);
            try {
                msl0.getOutput(output_name);
                
            }
            catch (Exception){
                if (output_name != "length") {
                    OPENSIM_THROW(Exception, "muscle_output: " +
                        get_muscle_outputs(i) + " is not a valid "
                        "Muscle output name")
                }
            }
            _muscle_output_double_names.push_back(output_name);
        }
    }
    
    int nMuscleOutputs = (int)_muscle_output_double_names.size();
    SimTK::Matrix msl_output_data(_n_frames, nMuscleOutputs,-1);

    //Muscle Storage
    for (std::string msl_path : _muscle_paths) {
        const Muscle& msl = 
            _model.updComponent<Muscle>(msl_path);

        //Path Point Storage
        SimTK::Matrix_<SimTK::Vec3> msl_matrix(_n_frames, 
            _max_path_points, SimTK::Vec3(-1));
        SimTK::Vector msl_vector(_n_frames, -1);

        _muscle_path_points.push_back(msl_matrix);
        _muscle_path_nPoints.push_back(msl_vector);

        //Output Data Storage
        _muscle_output_double_values.push_back(msl_output_data);
    }

}

void JointMechanicsTool::setupCoordinateStorage() {
    _coordinate_output_double_names.push_back("value");
    _coordinate_output_double_names.push_back("speed");

    for (const Coordinate& coord : _model.updComponentList<Coordinate>()) {
        _coordinate_names.push_back(coord.getName());

        SimTK::Matrix coord_data(_n_frames, 2, -1.0);
        _coordinate_output_double_values.push_back(coord_data);
    }
}

int JointMechanicsTool::record(const SimTK::State& s, const int frame_num)
{
    _model.realizeReport(s);

    //Store mesh vertex locations and transforms
    std::string frame_name = get_output_orientation_frame();
    const Frame& frame = _model.updComponent<Frame>(frame_name);
    std::string origin_name = get_output_position_frame();
    const Frame& origin = _model.updComponent<Frame>(origin_name);

    SimTK::Vec3 origin_pos = 
        origin.findStationLocationInAnotherFrame(s, SimTK::Vec3(0), frame);

    for (int i = 0; i < (int)_contact_mesh_paths.size(); ++i) {
        int nVertex = _mesh_vertex_locations[i].ncol();

        const Smith2018ContactMesh& mesh = 
            _model.getComponent<Smith2018ContactMesh>(_contact_mesh_paths[i]);

        SimTK::Vector_<SimTK::Vec3> ver = mesh.getVertexLocations();

        const SimTK::Transform& T = 
            mesh.getMeshFrame().findTransformBetween(s,frame);

        for (int j = 0; j < nVertex; ++j) {
            _mesh_vertex_locations[i](frame_num, j) = 
                T.shiftFrameStationToBase(ver(j)) - origin_pos;
        }

        //store transform
        SimTK::Mat33 R_matrix = T.R().asMat33();
        int c = 0;
        for (int m = 0; m < 3; ++m) {
            for (int n = 0; n < 3; ++n) {
                _mesh_transforms[i](frame_num, c) = R_matrix(m, n);
                c++;
            }
            _mesh_transforms[i](frame_num, c) = T.p()(m) - origin_pos(m);
            c++;
        }

        _mesh_transforms[i](frame_num, 15) = 1;

    }

    // Store Attached Geometries
    if (!_attach_geo_names.empty()) {
        for (int i = 0; i < (int)_attach_geo_names.size(); ++i) {

            const SimTK::PolygonalMesh& mesh = _attach_geo_meshes[i];

            SimTK::Transform trans = _model.updComponent<PhysicalFrame>(_attach_geo_frames[i]).findTransformBetween(s, frame);
            
            for (int j = 0; j < mesh.getNumVertices(); ++j) {
                _attach_geo_vertex_locations[i](frame_num, j) = trans.shiftFrameStationToBase(mesh.getVertexPosition(j)) - origin_pos;
            }
        }
    }

    //Store Contact data
    if (!_contact_force_paths.empty()) {
        int nFrc = 0;
        for (std::string frc_path : _contact_force_paths) {
            const Smith2018ArticularContactForce& frc = _model.updComponent<Smith2018ArticularContactForce>(frc_path);

            int nDouble = 0;
            for (std::string output_name : _contact_output_double_names) {
                _contact_output_double_values[nFrc].set(frame_num, nDouble, frc.getOutputValue<double>(s, output_name));
                nDouble++;
            }

            int nVec3 = 0;
            for (std::string output_name : _contact_output_vec3_names) {
                _contact_output_vec3_values[nFrc].set(frame_num, nVec3, frc.getOutputValue<SimTK::Vec3>(s, output_name));
                nVec3++;
            }
            
            int nVector = 0;
            for (std::string output_name : _contact_output_vector_double_names) {
                _contact_output_vector_double_values[nFrc][nVector].
                    updRow(frame_num) = 
                    ~frc.getOutputValue<SimTK::Vector>(s, output_name);
                nVector++;
            }

            int nVectorVec3 = 0;
            for (std::string output_name : _contact_output_vector_vec3_names) {
                int nCol = _contact_output_vector_vec3_values[nFrc][nVectorVec3].ncol();
                for (int c = 0; c < nCol; c++) {
                    _contact_output_vector_vec3_values[nFrc][nVectorVec3].updElt(frame_num, c) =
                        //updRow(frame_num) = ~SimTK::Vector_<SimTK::Vec3>(6, SimTK::Vec3(-2));
                    frc.getOutputValue<SimTK::Vector_<SimTK::Vec3>>(s, output_name)(c);
                }
                nVectorVec3++;
            }
            nFrc++;
        }
    }
            
    //Store ligament data
    if (!_ligament_paths.empty()) {
        int nLig = 0;
        for (const std::string& lig_path : _ligament_paths) {
            Blankevoort1991Ligament& lig = 
                _model.updComponent<Blankevoort1991Ligament>(lig_path);

            //Path Points
            const GeometryPath& geoPath = lig.upd_GeometryPath();

            int nPoints = 0;
            SimTK::Vector_<SimTK::Vec3> path_points(_max_path_points, SimTK::Vec3(-1));

            getGeometryPathPoints(s, geoPath, path_points, nPoints);
            for (int i = 0; i < nPoints; ++i) {
                _ligament_path_points[nLig].set(frame_num,i,path_points(i));
            }
            _ligament_path_nPoints[nLig][frame_num] = nPoints;
                
            //Output Data
            int j = 0;
            for (std::string output_name : _ligament_output_double_names) {
                _ligament_output_double_values[nLig].set(frame_num,j,
                    lig.getOutputValue<double>(s, output_name));
                j++;
            }
            nLig++;
        }
    }

    //Store muscle data
    if (!_muscle_paths.empty()) {
        int nMsl = 0;
        for (const std::string& msl_path : _muscle_paths) {
            Muscle& msl = _model.updComponent<Muscle>(msl_path);

            //Path Points
            const GeometryPath& geoPath = msl.upd_GeometryPath();

            int nPoints = 0;

            SimTK::Vector_<SimTK::Vec3> 
                path_points(_max_path_points,SimTK::Vec3(-1));

            getGeometryPathPoints(s, geoPath, path_points, nPoints);
            for (int i = 0; i < nPoints; ++i) {
                _muscle_path_points[nMsl].set(frame_num,i,path_points(i));
            }
            _muscle_path_nPoints[nMsl][frame_num] = nPoints;


            //Output Data
            int j = 0;
            for (std::string output_name : _muscle_output_double_names) {
                if (output_name == "length") {
                    _muscle_output_double_values[nMsl].set(frame_num, j,
                        msl.get_GeometryPath().getOutputValue<double>
                        (s, "length"));
                }
                else {
                    _muscle_output_double_values[nMsl].set(
                        frame_num, j, msl.getOutputValue<double>(s, output_name));
                }
                j++;
            }
            

            nMsl++;
        }
    }
    
    //Store Coordinate Data
    if (get_h5_kinematics_data()) {
        int nCoord = 0;
        for (const Coordinate& coord : _model.updComponentList<Coordinate>()) {
            if (coord.getMotionType() == Coordinate::MotionType::Rotational) {
                _coordinate_output_double_values[nCoord](frame_num, 0) = coord.getValue(s)*180/SimTK::Pi;
                _coordinate_output_double_values[nCoord](frame_num, 1) = coord.getSpeedValue(s)*180/SimTK::Pi;
            }
            else {
                _coordinate_output_double_values[nCoord](frame_num, 0) = coord.getValue(s);
                _coordinate_output_double_values[nCoord](frame_num, 1) = coord.getSpeedValue(s);
            }
            nCoord++;
        }
    }

    //Store Body Transformations in Ground
    if (get_write_transforms_file()) {
        const Frame& out_frame =
            _model.getComponent<Frame>(get_output_orientation_frame());

        SimTK::RowVector row((int)_model_frame_transforms.getColumnLabels().size());

        int c = 0; 
        for (const Frame& frame : _model.updComponentList<Frame>()) {

            SimTK::Mat44 trans_matrix =
                frame.findTransformBetween(s,out_frame).toMat44();

                for (int m = 0; m < 4; ++m) {
                    for (int n = 0; n < 4; ++n) {
                        row(c) = trans_matrix(m, n);
                        c++;
                    }
                }
        }
        _model_frame_transforms.appendRow(_time[frame_num],row);
    }
    return(0);
}

void JointMechanicsTool::getGeometryPathPoints(const SimTK::State& s, const GeometryPath& geoPath, SimTK::Vector_<SimTK::Vec3>& path_points, int& nPoints) {
    const Frame& out_frame = _model.getComponent<Frame>(get_output_orientation_frame());
    
    const Frame& origin = _model.getComponent<Frame>(get_output_position_frame());

    SimTK::Vec3 origin_pos = origin.findStationLocationInAnotherFrame(s, SimTK::Vec3(0), out_frame);

    const Array<AbstractPathPoint*>& pathPoints = geoPath.getCurrentPath(s);
    
    nPoints = 0;
    for (int i = 0; i < pathPoints.getSize(); ++i) {
        AbstractPathPoint* point = pathPoints[i];
        PathWrapPoint* pwp = dynamic_cast<PathWrapPoint*>(point);
        SimTK::Vec3 pos;

        //If wrapping point, need to collect all points on wrap object surface
        if (pwp) {
            Array<SimTK::Vec3>& surfacePoints = pwp->getWrapPath();
            const SimTK::Transform& X_BG = pwp->getParentFrame().findTransformBetween(s, out_frame);
            // Cycle through each surface point and tranform to output frame
            for (int j = 0; j < surfacePoints.getSize(); ++j) {
                pos = X_BG * surfacePoints[j]-origin_pos;
                path_points.set(nPoints, pos);
                nPoints++;
            }
        }
        else { // otherwise a regular PathPoint so just draw its location
            const SimTK::Transform& X_BG = point->getParentFrame().findTransformBetween(s, out_frame);
            pos = X_BG * point->getLocation(s)-origin_pos;

            path_points.set(nPoints, pos);
            nPoints++;
        }
    }
}

int JointMechanicsTool::printResults(const std::string &aBaseName,const std::string &aDir)
{
    std::string file_path = get_results_directory();
    std::string base_name = get_results_file_basename();

    //Analysis Results
    _model.updAnalysisSet().printResults(get_results_file_basename(), get_results_directory());
    
    //Write VTP files
    if (get_write_vtp_files()) {
        //Contact Meshes
        for (int i = 0; i < (int)_contact_mesh_names.size(); ++i) {
            std::string mesh_name = _contact_mesh_names[i];
            std::string mesh_path = _contact_mesh_paths[i];

            log_info( "Writing .vtp files: {}/{}_{}", 
                file_path, base_name, mesh_name);

            writeVTPFile(mesh_path, _contact_force_names, true);
        }

        //Attached Geometries
        if (!_attach_geo_names.empty()) {
            writeAttachedGeometryVTPFiles(true);
        }

        //Ligaments
        if (!_ligament_names.empty()) {
            int i = 0;
            for (std::string lig : _ligament_names) {

                log_info( "Writing .vtp files: {}/{}_{}", 
                file_path, base_name, lig);

                writeLineVTPFiles("ligament_" + lig, _ligament_path_nPoints[i],
                    _ligament_path_points[i], _ligament_output_double_names,
                    _ligament_output_double_values[i]);
                i++;
            }
        }

        //Muscles
        if (!_muscle_names.empty()) {
            int i = 0;
            for (std::string msl : _muscle_names) {

                log_info( "Writing .vtp files: {}/{}_{}", 
                    file_path, base_name, msl);

                writeLineVTPFiles("muscle_" + msl, _muscle_path_nPoints[i],
                    _muscle_path_points[i], _muscle_output_double_names,
                    _muscle_output_double_values[i]);
                i++;
            }
        }
    }

    //Write h5 file
    if (get_write_h5_file()) {
        writeH5File(aBaseName, aDir);
    }

    //Write Transforms file
    if (get_write_transforms_file()) {
        writeTransformsFile();
    }
    return(0);
}

void JointMechanicsTool::collectMeshContactOutputData(
    const std::string& mesh_name,
    std::vector<SimTK::Matrix>& triData,
    std::vector<std::string>& triDataNames,
    std::vector<SimTK::Matrix>& vertexData,
    std::vector<std::string>& vertexDataNames) {

    //Smith2018ContactMesh mesh;

    int nFrc = -1;
    for (std::string frc_path : _contact_force_paths) {
        nFrc++;

        std::string mesh_type = "";
        const Smith2018ArticularContactForce& frc = _model.updComponent<Smith2018ArticularContactForce>(frc_path);

        std::string casting_mesh_name = frc.getConnectee<Smith2018ContactMesh>("casting_mesh").getName();
        std::string target_mesh_name = frc.getConnectee<Smith2018ContactMesh>("target_mesh").getName();

        if (mesh_name == casting_mesh_name) {
            mesh_type = "casting";
            //Smith2018ContactMesh mesh = frc.getConnectee<Smith2018ContactMesh>("casting_mesh");
        }
        else if (mesh_name == target_mesh_name) {
            mesh_type = "target";
            //Smith2018ContactMesh mesh = frc.getConnectee<Smith2018ContactMesh>("target_mesh");
        }
        else {
            continue;
        }
        const Smith2018ContactMesh& mesh = frc.getConnectee<Smith2018ContactMesh>(mesh_type +"_mesh");

        
        int nVectorDouble = -1;
        for (std::string output_name : _contact_output_vector_double_names) {
            nVectorDouble++;
            std::vector<std::string> output_name_split = split_string(output_name, "_");
            std::string output_mesh_type = output_name_split[0];
            std::string output_data_type = output_name_split[1];
            std::string output_data_name = "";

            for (int i = 2; i < (int)output_name_split.size(); ++i) {
                if (i == 2) {
                    output_data_name = output_name_split[i];
                }
                else {
                    output_data_name = output_data_name + "_" + output_name_split[i];
                }
            }

            if (output_mesh_type != mesh_type) {
                continue;
            }

            if (output_data_type == "triangle") {
                //Seperate data for each contact force
                triDataNames.push_back(output_name + "_" + frc.getName());
                triData.push_back(
                    _contact_output_vector_double_values[nFrc][nVectorDouble]);

                //Combined data for all contacts visualized on one mesh
                int data_index;
                if (contains_string(triDataNames, output_name, data_index)) {
                    triData[data_index] += _contact_output_vector_double_values[nFrc][nVectorDouble];
                }
                else {
                    triDataNames.push_back(output_name);
                    triData.push_back(_contact_output_vector_double_values[nFrc][nVectorDouble]);
                }
            }

        }
        
        //Variable Cartilage Properties
        if ((getProperty_contact_mesh_properties().findIndex("thickness") != -1
            || getProperty_contact_mesh_properties().findIndex("all") != -1)
            && !contains_string(triDataNames, "triangle.thickness")) {

            SimTK::Matrix thickness_matrix(_n_frames, mesh.getNumFaces());
            for (int i = 0; i < _n_frames; ++i) {
                for (int j = 0; j < mesh.getNumFaces(); ++j) {
                    thickness_matrix(i, j) = mesh.getTriangleThickness(j);
                }
            }
            triDataNames.push_back("triangle.thickness");
            triData.push_back(thickness_matrix);
        }

        if ((getProperty_contact_mesh_properties().findIndex("elastic_modulus") != -1
            || getProperty_contact_mesh_properties().findIndex("all") != -1)
            && !contains_string(triDataNames, "triangle.elastic_modulus")) {

            SimTK::Matrix E_matrix(_n_frames, mesh.getNumFaces());
            for (int i = 0; i < _n_frames; ++i) {
                for (int j = 0; j < mesh.getNumFaces(); ++j) {
                    E_matrix(i, j) = mesh.getTriangleElasticModulus(j);
                }
            }
            triDataNames.push_back("triangle.elastic_modulus");
            triData.push_back(E_matrix);
        }

        if ((getProperty_contact_mesh_properties().findIndex("poissons_ratio") != -1
            || getProperty_contact_mesh_properties().findIndex("all") != -1)
            && !contains_string(triDataNames, "triangle.poissons_ratio")) {

            SimTK::Matrix v_matrix(_n_frames, mesh.getNumFaces());
            for (int i = 0; i < _n_frames; ++i) {
                for (int j = 0; j < mesh.getNumFaces(); ++j) {
                    v_matrix(i, j) = mesh.getTrianglePoissonsRatio(j);
                }
            }
            triDataNames.push_back("triangle.poissons_ratio");
            triData.push_back(v_matrix);
        }

        if ((getProperty_contact_mesh_properties().findIndex("area") != -1
            || getProperty_contact_mesh_properties().findIndex("all") != -1)
            && !contains_string(triDataNames, "triangle.area")) {

            SimTK::Matrix area_matrix(_n_frames, mesh.getNumFaces());
            for (int i = 0; i < _n_frames; ++i) {
                area_matrix[i] = ~mesh.getTriangleAreas();
            }
            triDataNames.push_back("triangle.area");
            triData.push_back(area_matrix);
        }
    }
}

void JointMechanicsTool::writeVTPFile(const std::string& mesh_path,
    const std::vector<std::string>& contact_names, bool isDynamic) {

    const Smith2018ContactMesh& cnt_mesh = 
        _model.getComponent<Smith2018ContactMesh>(mesh_path);
    std::string mesh_name = cnt_mesh.getName();

    std::string file_path = get_results_directory();
    std::string base_name = get_results_file_basename();

    std::string frame = split_string(get_output_orientation_frame(), "/").back();
    std::string origin = split_string(get_output_position_frame(), "/").back();

    //Collect data
    std::vector<SimTK::Matrix> triData, vertexData;
    std::vector<std::string> triDataNames, vertexDataNames;

    collectMeshContactOutputData(mesh_name,
        triData, triDataNames, vertexData, vertexDataNames);

    //Mesh face connectivity
    const SimTK::PolygonalMesh& mesh = cnt_mesh.getPolygonalMesh();
    
    SimTK::Matrix mesh_faces(mesh.getNumFaces(), mesh.getNumVerticesForFace(0));

    for (int j = 0; j < mesh.getNumFaces(); ++j) {
        for (int k = 0; k < mesh.getNumVerticesForFace(0); ++k) {
            mesh_faces(j, k) = mesh.getFaceVertex(j, k);
        }
    }

    for (int frame_num = 0; frame_num < _n_frames; ++frame_num) {
        //Write file
        VTPFileAdapter* mesh_vtp = new VTPFileAdapter();
        mesh_vtp->setDataFormat(get_vtp_file_format());
        //mesh_vtp->setDataFormat("ascii");
        for (int i = 0; i < (int)triDataNames.size(); ++i) {
            mesh_vtp->appendFaceData(triDataNames[i], ~triData[i][frame_num]);
        }


        if (isDynamic) {
            int mesh_index;
            contains_string(_contact_mesh_names, mesh_name, mesh_index);

            mesh_vtp->setPointLocations(_mesh_vertex_locations[mesh_index][frame_num]);
            mesh_vtp->setPolygonConnectivity(mesh_faces);

            mesh_vtp->write(base_name + "_contact_" + mesh_name + "_dynamic_" + frame + "_" + origin,
                file_path + "/", frame_num);
        }
        else { //static
            SimTK::PolygonalMesh poly_mesh =
                _model.getComponent<Smith2018ContactMesh>(mesh_name).getPolygonalMesh();

            mesh_vtp->setPolygonsFromMesh(poly_mesh);

            mesh_vtp->write(base_name + "_contact_" + mesh_name +
                "_static_" + frame, file_path + "/", frame_num);
        }
        delete mesh_vtp;
    }
}

void JointMechanicsTool::writeAttachedGeometryVTPFiles(bool isDynamic) {
    std::string file_path = get_results_directory();
    std::string base_name = get_results_file_basename();

    std::string frame = split_string(get_output_orientation_frame(), "/").back();
    std::string origin = split_string(get_output_position_frame(), "/").back();

    for (int i = 0; i < (int)_attach_geo_names.size(); ++i) {
        std::cout << "Writing .vtp files: " << file_path << "/" 
            << base_name << "_"<< _attach_geo_names[i] << std::endl;

        //Face Connectivity
        const SimTK::PolygonalMesh& mesh = _attach_geo_meshes[i];

        SimTK::Matrix mesh_faces(mesh.getNumFaces(), mesh.getNumVerticesForFace(0));

        for (int j = 0; j < mesh.getNumFaces(); ++j) {
            for (int k = 0; k < mesh.getNumVerticesForFace(0); ++k) {
                mesh_faces(j, k) = mesh.getFaceVertex(j, k);
            }
        }

        for (int frame_num = 0; frame_num < _n_frames; ++frame_num) {

            //Write file
            VTPFileAdapter* mesh_vtp = new VTPFileAdapter();
            mesh_vtp->setDataFormat(get_vtp_file_format());
            
            if (isDynamic) {
                mesh_vtp->setPointLocations(_attach_geo_vertex_locations[i][frame_num]);
                mesh_vtp->setPolygonConnectivity(mesh_faces);

                mesh_vtp->write(base_name + "_mesh_" + _attach_geo_names[i] + "_dynamic_" +
                    frame + "_" + origin, file_path + "/", frame_num);
            }
            else { //static
                mesh_vtp->setPolygonsFromMesh(mesh);
                mesh_vtp->write(base_name + "_mesh_" + _attach_geo_names[i] + "_static_" +
                    frame + "_" + origin, file_path + "/", frame_num);
            }
            delete mesh_vtp;
        }
    }
}

void JointMechanicsTool::writeLineVTPFiles(std::string line_name,
    const SimTK::Vector& nPoints, const SimTK::Matrix_<SimTK::Vec3>& path_points,
    const std::vector<std::string>& output_double_names, const SimTK::Matrix& output_double_values) 
{
    for (int i = 0; i < _n_frames; ++i) {
        int nPathPoints = (int)nPoints.get(i);
            
        VTPFileAdapter* mesh_vtp = new VTPFileAdapter();

        mesh_vtp->setDataFormat(get_vtp_file_format());
       

        //Collect points
        SimTK::RowVector_<SimTK::Vec3> points(nPathPoints);
        SimTK::Vector lines(nPathPoints);

        for (int k = 0; k < nPathPoints; k++) {
            points(k) = path_points.get(i, k);
            lines(k) = k;
        }
                
        mesh_vtp->setPointLocations(points);
        mesh_vtp->setLineConnectivity(lines);

        //Collect Data
        for (int k = 0; k < (int)output_double_names.size(); ++k) {
            SimTK::Vector point_data(nPathPoints, output_double_values[i][k]);
            mesh_vtp->appendPointData(output_double_names[k], point_data);
        }

        //Write File
        std::string frame = split_string(get_output_orientation_frame(), "/").back();
        std::string origin = split_string(get_output_position_frame(), "/").back();

        mesh_vtp->write(get_results_file_basename() + "_" + line_name + "_" + 
            frame + "_" + origin, get_results_directory() + "/", i);
        delete mesh_vtp;
    }
}

void JointMechanicsTool::writeH5File(
    const std::string &aBaseName, const std::string &aDir)
{
    H5FileAdapter h5;

    const std::string h5_file{ aDir + "/" + aBaseName + ".h5" };
    h5.open(h5_file);
    h5.writeTimeDataSet(_time);

    h5.createGroup(_model.getName());

    std::string force_group = _model.getName() + "/forceset";
    h5.createGroup(force_group);

    //Write COMAK Convergence
    if (!get_input_comak_convergence_file().empty()) {

        //Read COMAK convergence sto
        
        TimeSeriesTable convergence_table = TimeSeriesTable(get_input_comak_convergence_file());

        //h5.createGroup("comak");
        h5.writeDataSet(convergence_table, "comak",true);
    }

    //Write Inverse Dynamics Data
    if (!get_input_inverse_dynamics_file().empty() && 
        (get_coordinate_outputs(0) == "all" ||
                getProperty_coordinate_outputs().findIndex("id_generalized_force") > -1)
        ) {


        Storage id_sto = processInputStorage(get_input_inverse_dynamics_file());
        TimeSeriesTable id_table = id_sto.exportToTable();

        std::string coordinate_group = _model.getName() + "/coordinateset";
        h5.createGroup(coordinate_group);

        for (std::string& label : id_table.getColumnLabels()) {
            std::vector<std::string> split_label = split_string(label, "_");

            //force or moment
            std::string id_type = split_label.back();

            std::string coord_name;
            for (size_t i = 0; i < split_label.size() - 1; i++) {
                if (i > 0) {
                    coord_name.append("_");
                }
                coord_name.append(split_label[i]);
            }

            std::string coord_path = _model.getCoordinateSet().
                get(coord_name).getAbsolutePathString();




            if (get_coordinates(0) == "all" ||
                getProperty_coordinates().findIndex(coord_path) > -1) {


                std::string coord_group = coordinate_group + "/" + coord_name;
                h5.createGroup(coord_group);

                SimTK::VectorView data = id_table.getDependentColumn(label);
                h5.writeDataSetSimTKVector(data, coord_group + "/" + id_type);
            }
        }
    }

    //Write States Data
    if (get_h5_states_data()) {
        StatesReporter& states_analysis = dynamic_cast<StatesReporter&>(_model.updAnalysisSet().get("joint_mechanics_states_analysis"));
        const TimeSeriesTable& states_table = states_analysis.getStatesStorage().exportToTable();

        //h5.writeDataSet(states_table, _model.getName() + "/states");
        h5.writeDataSet2(states_table, _model.getName() + "/states");
        //h5.writeStatesDataSet(states_table);
    }

    //Write coordinate data
    if (get_h5_kinematics_data() &&
        !(get_coordinate_outputs(0) == "none")) {

        std::string coordinate_group = _model.getName() + "/coordinateset";
        h5.createGroup(coordinate_group);

        int i = 0;
        for (std::string coord_name : _coordinate_names) {
            std::string coord_group = coordinate_group + "/" + coord_name;
           

            std::string coord_path = _model.getCoordinateSet().
                get(coord_name).getAbsolutePathString();

            if (get_coordinates(0) == "all" ||
                getProperty_coordinates().findIndex(coord_path) > -1) {

                 h5.createGroup(coord_group);

                int j = 0;
                for (std::string data_label : _coordinate_output_double_names) {

                    if (get_coordinate_outputs(0) == "all" ||
                        getProperty_coordinate_outputs().findIndex(data_label) > -1) {

                        SimTK::Vector data = _coordinate_output_double_values[i](j);
                        h5.writeDataSetSimTKVector(data, coord_group + "/" + data_label);
                    }
                    j++;
                }
                
            }
            i++;
        }
    }

    //Write Muscle Data
    if (!_muscle_names.empty()) {
        std::string muscle_group = 
            force_group + "/Muscle";

        h5.createGroup(muscle_group);

        int i = 0;
        for (std::string msl_name : _muscle_names) {
            std::string msl_group = muscle_group + "/" + msl_name;
            h5.createGroup(msl_group);

            int j = 0;
            for (std::string data_label : _muscle_output_double_names) {

                SimTK::Vector data = _muscle_output_double_values[i](j);
                h5.writeDataSetSimTKVector(data, msl_group + "/" + data_label);
                j++;
            }
            i++;
        }
    }

    //Write Ligament Data
    if (!_ligament_names.empty()) {
        std::string ligament_group = 
            force_group + "/Blankevoort1991Ligament";

        h5.createGroup(ligament_group);

        int i = 0;
        for (std::string lig_name : _ligament_names) {
            std::string lig_group = ligament_group + "/" + lig_name;
            h5.createGroup(lig_group);

            int j = 0;
            for (std::string data_label : _ligament_output_double_names) {

                SimTK::Vector data = _ligament_output_double_values[i](j);
                h5.writeDataSetSimTKVector(data, lig_group + "/" + data_label);
                j++;
            }
            i++;
        }
    }

    //Write Contact Data
    if (!_contact_mesh_names.empty()) {
        
        std::string contact_group = 
            force_group + "/Smith2018ArticularContactForce";

        h5.createGroup(contact_group);

        int i = 0;
        for (std::string contact_force_name :  _contact_force_names) {
            std::string cnt_group = contact_group + "/" + contact_force_name;
            h5.createGroup(cnt_group);

            std::string casting_mesh_name = _model.getComponent
                <Smith2018ArticularContactForce>(_contact_force_paths[i]).
                getSocket<Smith2018ContactMesh>("casting_mesh").getConnectee().getName();

            std::string target_mesh_name = _model.getComponent
                <Smith2018ArticularContactForce>(_contact_force_paths[i]).
                getSocket<Smith2018ContactMesh>("target_mesh").getConnectee().getName();

            std::string casting_mesh_group = 
                cnt_group + "/" + casting_mesh_name;

            std::string target_mesh_group = 
                cnt_group + "/" + target_mesh_name;

            h5.createGroup(casting_mesh_group);
            h5.createGroup(target_mesh_group);

            // output double
            int j = 0;
            for (std::string output_name : _contact_output_double_names) {
                std::vector<std::string> split_name = 
                    split_string(output_name, "_");

                std::string mesh_name;
                if (split_name[0] == "casting") {
                    mesh_name = casting_mesh_name;
                }
                else {
                    mesh_name = target_mesh_name;
                }

                std::string data_label;
                for (int k = 1; k < (int)split_name.size(); ++k) {
                    if (k == ((int)split_name.size() - 1)) {
                        data_label.append(split_name[k]);
                    }
                    else {
                        data_label.append(split_name[k] + "_");
                    }
                }

                SimTK::Vector data = _contact_output_double_values[i](j);

                std::string data_path = 
                    cnt_group + "/" + mesh_name + "/" + data_label;

                h5.writeDataSetSimTKVector(data, data_path);

                j++;
            }

            // output vec3
            j = 0;
            for (std::string output_name : _contact_output_vec3_names) {
                std::vector<std::string> split_name = 
                    split_string(output_name, "_");

                std::string mesh_name;
                if (split_name[0] == "casting") {
                    mesh_name = casting_mesh_name;
                }
                else {
                    mesh_name = target_mesh_name;
                }

                std::string data_label;
                for (int k = 1; k < (int)split_name.size(); ++k) {
                    if (k == ((int)split_name.size() - 1)) {
                        data_label.append(split_name[k]);
                    }
                    else {
                        data_label.append(split_name[k] + "_");
                    }
                }

                SimTK::Vector_<SimTK::Vec3> data = _contact_output_vec3_values[i](j);

                std::string data_path = 
                    cnt_group + "/" + mesh_name + "/" + data_label;
                
                h5.writeDataSetSimTKVectorVec3(data, data_path);

                j++;
            }

            // output vector
            j = 0;
            for (std::string output_name : _contact_output_vector_double_names) {
                std::vector<std::string> split_name = 
                    split_string(output_name, "_");

                std::string mesh_name;
                if (split_name[0] == "casting") {
                    mesh_name = casting_mesh_name;
                }
                else {
                    mesh_name = target_mesh_name;
                }

                std::string data_label;
                for (int k = 1; k < (int)split_name.size(); ++k) {
                    if (k == ((int)split_name.size() - 1)) {
                        data_label.append(split_name[k]);
                    }
                    else {
                        data_label.append(split_name[k] + "_");
                    }
                }

                SimTK::Matrix data =  _contact_output_vector_double_values[i][j];

                std::string data_path = 
                    cnt_group + "/" + mesh_name + "/" + data_label;

                h5.writeDataSetSimTKMatrix(data, data_path);

                j++;
            }

             // output vector vec3
            j = 0;
            for (std::string output_name : _contact_output_vector_vec3_names) {
                std::vector<std::string> split_name = 
                    split_string(output_name, "_");

                std::string mesh_name;
                if (split_name[0] == "casting") {
                    mesh_name = casting_mesh_name;
                }
                else {
                    mesh_name = target_mesh_name;
                }

                std::string data_label;
                for (int k = 1; k < (int)split_name.size(); ++k) {
                    if (k == ((int)split_name.size() - 1)) {
                        data_label.append(split_name[k]);
                    }
                    else {
                        data_label.append(split_name[k] + "_");
                    }
                }

                SimTK::Matrix_<SimTK::Vec3> data = 
                    _contact_output_vector_vec3_values[i][j];

                std::string data_path = 
                    cnt_group + "/" + mesh_name + "/" + data_label;
                h5.createGroup(data_path);

                std::vector<std::string> reg_data_path;
                for (int r = 0; r < 6; ++r) {
                    reg_data_path.push_back(data_path + "/" + std::to_string(r));
                }

                h5.writeDataSetSimTKMatrixVec3Columns(data, reg_data_path);

                j++;
            }

            i++;
        }

        //Mesh Transforms
        /*std::string transforms_group = contact_group + "/transforms";
        h5.createGroup(transforms_group);

        for (int i = 0; i < (int)_contact_mesh_paths.size(); ++i) {
            std::string data_path =
                transforms_group + "/" + _contact_mesh_names[i];

            h5.writeDataSetSimTKMatrix(_mesh_transforms[i], data_path);
        }*/
    }

    //Write Transforms Data
    /*
    if (get_h5_write_transforms()) {
        std::string transforms_group = _model.getName() + "/transforms";
        h5.createGroup(transforms_group);
        //_model_frame_transforms.getColumnLabels();
        int nFrames = _model_frame_transforms.getNumColumns() / 16;

        int c = 0;
        for (int i = 0; i < nFrames; ++i) {
            for (int j = 0; j < 16; j++) {
                std::string mesh_group = transforms_group + "/" + _model_frame_transforms.getColumnLabel(c);
                std::cout << mesh_group << std::endl;
                //h5.createGroup(mesh_group);
                h5.writeDataSetSimTKMatrix(_model_frame_transforms.getDependentColumnAtIndex(c), mesh_group);
                c++;
            }
        }
    }*/

    h5.close();

}

void JointMechanicsTool::writeTransformsFile() {
    std::string file_type = get_output_transforms_file_type();
    if (file_type == "sto" || file_type == ".sto") {
        std::string full_file = get_results_directory() + "/" +
            get_results_file_basename() + "_frame_transforms_in_" + 
            get_output_orientation_frame() + ".sto";

        STOFileAdapter().write(_model_frame_transforms, full_file);
    }
    if (file_type == "csv" || file_type == ".csv") {
        std::string full_file = get_results_directory() + "/"+
            get_results_file_basename() + "_frame_transforms_in_" + 
            get_output_orientation_frame() + ".csv";

        CSVFileAdapter().write(_model_frame_transforms, full_file);
    }
}

/*Once OpenSim is upgraded to 4.2 this can be replaced with TableUtilities*/
int JointMechanicsTool::findStateLabelIndexInternal(const std::string* begin,
        const std::string* end, const std::string& desired) {

    auto found = std::find(begin, end, desired);
    if (found != end) return (int)std::distance(begin, found);

    // 4.0 and its beta versions differ slightly in the absolute path but
    // the <joint>/<coordinate>/value (or speed) will be common to both.
    // Likewise, for muscle states <muscle>/activation (or fiber_length)
    // must be common to the state variable (path) name and column label.
    std::string shortPath = desired;
    std::string::size_type front = shortPath.find('/');
    while (found == end && front < std::string::npos) {
        shortPath = shortPath.substr(front + 1, desired.length());
        found = std::find(begin, end, shortPath);
        front = shortPath.find('/');
    }
    if (found != end) return (int)std::distance(begin, found);

    // Assume column labels follow pre-v4.0 state variable labeling.
    // Redo search with what the pre-v4.0 label might have been.

    // First, try just the last element of the path.
    std::string::size_type back = desired.rfind('/');
    std::string prefix = desired.substr(0, back);
    std::string shortName = desired.substr(back + 1, desired.length() - back);
    found = std::find(begin, end, shortName);
    if (found != end) return (int)std::distance(begin, found);

    // If that didn't work, specifically check for coordinate state names
    // (<coord_name>/value and <coord_name>/speed) and muscle state names
    // (<muscle_name>/activation <muscle_name>/fiber_length).
    if (shortName == "value") {
        // pre-v4.0 did not have "/value" so remove it if here
        back = prefix.rfind('/');
        shortName = prefix.substr(back + 1, prefix.length());
        found = std::find(begin, end, shortName);
    } else if (shortName == "speed") {
        // replace "/speed" (the v4.0 labeling for speeds) with "_u"
        back = prefix.rfind('/');
        shortName = prefix.substr(back + 1, prefix.length() - back) + "_u";
        found = std::find(begin, end, shortName);
    } else if (back < desired.length()) {
        // try replacing the '/' with '.' in the last segment
        shortName = desired;
        shortName.replace(back, 1, ".");
        back = shortName.rfind('/');
        shortName = shortName.substr(back + 1, shortName.length() - back);
        found = std::find(begin, end, shortName);
    }
    if (found != end) return (int)std::distance(begin, found);

    // If all of the above checks failed, return -1.
    return -1;
}
/*
void JointMechanicsTool::loadModel(const std::string &aToolSetupFileName)
{
    
    OPENSIM_THROW_IF(get_model_file().empty(), Exception,
            "No model file was specified (<model_file> element is empty) in "
            "the Setup file. ");
    std::string saveWorkingDirectory = IO::getCwd();
    std::string directoryOfSetupFile = IO::getParentDirectory(aToolSetupFileName);
    IO::chDir(directoryOfSetupFile);

    std::cout<<"JointMechanicsTool "<< getName() <<" loading model '"<<get_model_file() <<"'"<< std::endl;

    Model model;

    try {
        model = Model(get_model_file());
        model.finalizeFromProperties();
        
    } catch(...) { // Properly restore current directory if an exception is thrown
        IO::chDir(saveWorkingDirectory);
        throw;
    }
    _model = model;
    IO::chDir(saveWorkingDirectory);
}*/