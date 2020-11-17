
#include "IMUInverseKinematicsTool.h"
#include <OpenSim/Simulation/OpenSense/OpenSenseUtilities.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Common/TableSource.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/TRCFileAdapter.h>
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Simulation/OrientationsReference.h>


using namespace OpenSim;
using namespace SimTK;
using namespace std;


IMUInverseKinematicsTool::IMUInverseKinematicsTool()
        : InverseKinematicsToolBase() {
    constructProperties();
}

IMUInverseKinematicsTool::IMUInverseKinematicsTool(const std::string& setupFile)
        : InverseKinematicsToolBase(setupFile, true) {
    constructProperties();
    updateFromXMLDocument();
}

IMUInverseKinematicsTool::~IMUInverseKinematicsTool()
{
}

void IMUInverseKinematicsTool::constructProperties()
{
    constructProperty_sensor_to_opensim_rotations(
            SimTK::Vec3(0));
    constructProperty_orientations_file("");
    OrientationWeightSet orientationWeights;
    constructProperty_orientation_weights(orientationWeights);
}
/**
void IMUInverseKinematicsTool::
    previewExperimentalData(const TimeSeriesTableVec3& markers,
                const TimeSeriesTable_<SimTK::Rotation>& orientations) const
{
    Model previewWorld;

    TimeSeriesTableVec3 trimmedMarkerData(markers);
    trimmedMarkerData.trim(getStartTime(), getEndTime());
    // Load the marker data into a TableSource that has markers
    // as its output which each markers occupying its own channel
    TableSourceVec3* markersSource = new TableSourceVec3(trimmedMarkerData);
    // Add the markersSource Component to the model
    previewWorld.addComponent(markersSource);

    // Get the underlying Table backing the the marker Source so we 
    // know how many markers we have and their names
    const auto& markerData = markersSource->getTable();
    auto& times = markerData.getIndependentColumn();

    // Create an ExperimentalMarker Component for every column in the markerData 
    for (int i = 0; i < int(markerData.getNumColumns()) ; ++i) {
        auto marker = new ExperimentalMarker();
        marker->setName(markerData.getColumnLabel(i));

        // markers are owned by the model
        previewWorld.addComponent(marker);
        // the time varying location of the marker comes from the markersSource
        // Component
        marker->updInput("location_in_ground").connect(
            markersSource->getOutput("column").getChannel(markerData.getColumnLabel(i)));
    }

    previewWorld.setUseVisualizer(true);
    SimTK::State& state = previewWorld.initSystem();
    state.updTime() = times[0];

    previewWorld.realizePosition(state);
    previewWorld.getVisualizer().show(state);


    log_info("Visualizing experimental marker data ...");
    for (size_t j =0; j < times.size(); j=j+10) {
        log_info("time: {} s", times[j]);
        state.setTime(times[j]);
        previewWorld.realizePosition(state);
        previewWorld.getVisualizer().show(state);
    }
}
*/
void IMUInverseKinematicsTool::runInverseKinematicsWithOrientationsFromFile(
        Model& model, const std::string& orientationsFileName,
        bool visualizeResults) {

    // Ideally if we add a Reporter, we also remove it at the end for good hygiene but 
    // at the moment there's no interface to remove Reporter so we'll reuse one if exists
    const auto reporterExists = model.findComponent<TableReporter>("ik_reporter");

    bool reuse_reporter = true;
    TableReporter* ikReporter = nullptr;
    if (reporterExists == nullptr) {
        // Add a reporter to get IK computed coordinate values out
        ikReporter = new TableReporter();
        ikReporter->setName("ik_reporter");
        reuse_reporter = false;
    } else
        ikReporter = &model.updComponent<TableReporter>("ik_reporter");

    auto coordinates = model.updComponentList<Coordinate>();

    // Hookup reporter inputs to the individual coordinate outputs
    // and lock coordinates that are translational since they cannot be
    for (auto& coord : coordinates) {
        ikReporter->updInput("inputs").connect(
                coord.getOutput("value"), coord.getName());
        if (coord.getMotionType() == Coordinate::Translational) {
            coord.setDefaultLocked(true);
        }
    }

    if (!reuse_reporter) {
        model.addComponent(ikReporter);
    }
    TimeSeriesTable_<SimTK::Quaternion> quatTable(orientationsFileName);
    log_info("Loading orientations as quaternions from '{}'...",
        orientationsFileName);
    // Will maintain only data in time range specified by the tool
    // If unspecified {-inf, inf} no trimming is done
    quatTable.trim(getStartTime(), getEndTime());
    // Convert to OpenSim Frame
    const SimTK::Vec3& rotations = get_sensor_to_opensim_rotations();
    SimTK::Rotation sensorToOpenSim = SimTK::Rotation(
            SimTK::BodyOrSpaceType::SpaceRotationSequence, 
            rotations[0], SimTK::XAxis, rotations[1], SimTK::YAxis, 
            rotations[2], SimTK::ZAxis);

    // Rotate data so Y-Axis is up
    OpenSenseUtilities::rotateOrientationTable(quatTable, sensorToOpenSim);
    //Trim to time window required by Tool
    quatTable.trim(getStartTime(), getEndTime());

    TimeSeriesTable_<SimTK::Rotation> orientationsData =
        OpenSenseUtilities::convertQuaternionsToRotations(quatTable);

    OrientationsReference oRefs(orientationsData, &get_orientation_weights());

    SimTK::Array_<CoordinateReference> coordinateReferences;

    // visualize for debugging
    if (visualizeResults)
        model.setUseVisualizer(true);
    SimTK::State& s0 = model.initSystem();

    AnalysisSet& analysisSet = model.updAnalysisSet();
    analysisSet.begin(s0);


    double t0 = s0.getTime();

    // create the solver given the input data
    const double accuracy = 1e-4;
    InverseKinematicsSolver ikSolver(model, nullptr,
            std::make_shared<OrientationsReference>(oRefs),
        coordinateReferences);
    ikSolver.setAccuracy(accuracy);

    auto& times = oRefs.getTimes();
    std::shared_ptr<TimeSeriesTable> modelOrientationErrors(
            get_report_errors() ? new TimeSeriesTable()
                                : nullptr);
    s0.updTime() = times[0];
    ikSolver.assemble(s0);
    // Create place holder for orientation errors, populate based on user pref.
    // according to report_errors property
    int nos = ikSolver.getNumOrientationSensorsInUse();
    SimTK::Array_<double> orientationErrors(nos, 0.0);

    if (get_report_errors()) { 
        SimTK::Array_<string> labels;
        for (int i = 0; i < nos; ++i) {
            labels.push_back(ikSolver.getOrientationSensorNameForIndex(i));
        }
        modelOrientationErrors->setColumnLabels(labels);
        modelOrientationErrors->updTableMetaData().setValueForKey<string>(
                "name", "OrientationErrors");
        ikSolver.computeCurrentOrientationErrors(orientationErrors);
    }
    if (visualizeResults) {
        model.getVisualizer().show(s0);
        model.getVisualizer().getSimbodyVisualizer().setShowSimTime(true);
    }
    int step = 0;
    for (auto time : times) {
        s0.updTime() = time;
        ikSolver.track(s0);
        if (get_report_errors()) {
            ikSolver.computeCurrentOrientationErrors(orientationErrors);
            modelOrientationErrors->appendRow(
                    s0.getTime(), orientationErrors);
        }
        if (visualizeResults)  
            model.getVisualizer().show(s0);
        else
            log_info("Solved at time: {} s", time);
        // realize to report to get reporter to pull values from model
        analysisSet.step(s0, step++);
        model.realizeReport(s0);
    }

    auto report = ikReporter->getTable();
    // form resultsDir either from results_directory or output_motion_file
    auto resultsDir = get_results_directory();
    if (resultsDir.empty() && !get_output_motion_file().empty())
        resultsDir = IO::getParentDirectory(get_output_motion_file());
    if (!resultsDir.empty()) {
        IO::makeDir(resultsDir);
        // directory will be restored on block exit
        // by changing dir all other files are created in resultsDir
        auto cwd = IO::CwdChanger::changeTo(resultsDir);
        std::string outName = get_output_motion_file();
        outName = IO::GetFileNameFromURI(outName);
        if (outName.empty()) {
            bool isAbsolutePath;
            string directory, fileName, extension;
            SimTK::Pathname::deconstructPathname(orientationsFileName,
                    isAbsolutePath, directory, fileName, extension);
            outName = "ik_" + fileName;
        }
        std::string outputFile = outName;

        // Convert to degrees to compare with marker-based IK
        // but only for rotational coordinates
        model.getSimbodyEngine().convertRadiansToDegrees(report);
        report.updTableMetaData().setValueForKey<string>("name", outName);

        auto fullOutputFilename = outputFile;
        std::string::size_type extSep = fullOutputFilename.rfind(".");
        if (extSep == std::string::npos) { fullOutputFilename.append(".mot"); }
        STOFileAdapter_<double>::write(report, fullOutputFilename);

        log_info("Wrote IK with IMU tracking results to: '{}'.",
                fullOutputFilename);
        if (get_report_errors()) {
            STOFileAdapter_<double>::write(*modelOrientationErrors,
                    outName + "_orientationErrors.sto");
        }
    } 
    else
        log_info("IMUInverseKinematicsTool: No output files were generated, "
            "set output_motion_file to generate output files.");
    // Results written to file, clear in case we run again
    ikReporter->clearTable();
}


// main driver
bool IMUInverseKinematicsTool::run(bool visualizeResults)
{
    if (_model.empty()) {
        _model.reset(new Model(get_model_file()));
    }

    runInverseKinematicsWithOrientationsFromFile(*_model,
                                                 get_orientations_file(),
                                                 visualizeResults);

    return true;
}

TimeSeriesTable_<SimTK::Vec3> 
    IMUInverseKinematicsTool::loadMarkersFile(const std::string& markerFile)
{
    TimeSeriesTable_<Vec3> markers(markerFile);
    log_info("'{}' loaded {} markers and {} rows of data.", markerFile,
        markers.getNumColumns(), markers.getNumRows());

    if (markers.hasTableMetaDataKey("Units")) {
        auto& value = markers.getTableMetaData().getValueForKey("Units");
        log_info("'{}' has Units meta data. Units are {}.", markerFile,
                value.getValue<std::string>());
        if (value.getValue<std::string>() == "mm") {
            log_info("Marker data in mm, converting to m.");
            for (size_t i = 0; i < markers.getNumRows(); ++i) {
                markers.updRowAtIndex(i) *= 0.001;
            }
            markers.updTableMetaData().removeValueForKey("Units");
            markers.updTableMetaData().setValueForKey<std::string>("Units", "m");
        }
    }
    auto& value = markers.getTableMetaData().getValueForKey("Units");
    log_info("'{}' Units are {}.", markerFile, value.getValue<std::string>());

    return markers;
}
