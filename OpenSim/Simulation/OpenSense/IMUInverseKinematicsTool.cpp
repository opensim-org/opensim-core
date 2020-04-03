
#include "IMUInverseKinematicsTool.h"
#include "OpenSenseUtilities.h"
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
#include "ExperimentalMarker.h"
#include "ExperimentalFrame.h"


using namespace OpenSim;
using namespace SimTK;
using namespace std;


IMUInverseKinematicsTool::IMUInverseKinematicsTool()
{
    constructProperties();
}

IMUInverseKinematicsTool::IMUInverseKinematicsTool(const std::string& setupFile)
    : Object(setupFile, true)
{
    constructProperties();
    updateFromXMLDocument();
}

IMUInverseKinematicsTool::~IMUInverseKinematicsTool()
{
}

void IMUInverseKinematicsTool::constructProperties()
{
    constructProperty_accuracy(1e-6);
    constructProperty_constraint_weight(Infinity);
    Array<double> range{ Infinity, 2};
    range[0] = -Infinity; // Make range -Infinity to Infinity unless limited by data
    constructProperty_time_range(range);

    constructProperty_sensor_to_opensim_rotations(
            SimTK::Vec3(0));

    constructProperty_model_file("");
    constructProperty_marker_file("");
    constructProperty_orientations_file("");

    constructProperty_results_directory("");
}

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

    char c;
    std::cout << "Press any key to visualize experimental marker data ..." << std::endl;
    std::cin >> c;

    for (size_t j =0; j < times.size(); j=j+10) {
        std::cout << "time: " << times[j] << "s" << std::endl;
        state.setTime(times[j]);
        previewWorld.realizePosition(state);
        previewWorld.getVisualizer().show(state);
    }
}

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
    std::cout << "Loading orientations as quaternions from "
        << orientationsFileName << std::endl;
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

    OrientationsReference oRefs(orientationsData);
    MarkersReference mRefs{};

    SimTK::Array_<CoordinateReference> coordinateReferences;

    // visualize for debugging
    if (visualizeResults)
        model.setUseVisualizer(true);
    SimTK::State& s0 = model.initSystem();

    double t0 = s0.getTime();

    // create the solver given the input data
    const double accuracy = 1e-4;
    InverseKinematicsSolver ikSolver(model, mRefs, oRefs,
        coordinateReferences);
    ikSolver.setAccuracy(accuracy);

    auto& times = oRefs.getTimes();

    s0.updTime() = times[0];
    ikSolver.assemble(s0);
    if (visualizeResults) {
        model.getVisualizer().show(s0);
        model.getVisualizer().getSimbodyVisualizer().setShowSimTime(true);
    }
    for (auto time : times) {
        s0.updTime() = time;
        ikSolver.track(s0);
        if (visualizeResults)  
            model.getVisualizer().show(s0);
        else
            cout << "Solved at time: " << time << endl;
        // realize to report to get reporter to pull values from model
        model.realizeReport(s0);
    }

    auto report = ikReporter->getTable();

    auto eix = orientationsFileName.rfind(".");
    auto stix = orientationsFileName.rfind("/") + 1;

    IO::makeDir(get_results_directory());
    std::string outName = "ik_" + orientationsFileName.substr(stix, eix-stix);
    std::string outputFile = get_results_directory() + "/" + outName;

    // Convert to degrees to compare with marker-based IK
    // but only for rotational coordinates
    model.getSimbodyEngine().convertRadiansToDegrees(report);
    report.updTableMetaData().setValueForKey<string>("name", outName);

    STOFileAdapter_<double>::write(report, outputFile + ".mot");

    std::cout << "Wrote IK with IMU tracking results to: '" <<
        outputFile << "'." << std::endl;

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
    std::cout << markerFile << " loaded " << markers.getNumColumns() << " markers "
        << " and " << markers.getNumRows() << " rows of data." << std::endl;

    if (markers.hasTableMetaDataKey("Units")) {
        std::cout << markerFile << " has Units meta data." << std::endl;
        auto& value = markers.getTableMetaData().getValueForKey("Units");
        std::cout << markerFile << " Units are " << value.getValue<std::string>() << std::endl;
        if (value.getValue<std::string>() == "mm") {
            std::cout << "Marker data in mm, converting to m." << std::endl;
            for (size_t i = 0; i < markers.getNumRows(); ++i) {
                markers.updRowAtIndex(i) *= 0.001;
            }
            markers.updTableMetaData().removeValueForKey("Units");
            markers.updTableMetaData().setValueForKey<std::string>("Units", "m");
        }
    }
    auto& value = markers.getTableMetaData().getValueForKey("Units");
    std::cout << markerFile << " Units are " << value.getValue<std::string>() << std::endl;
    return markers;
}
