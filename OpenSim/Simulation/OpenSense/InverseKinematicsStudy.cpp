
#include "InverseKinematicsStudy.h"
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


InverseKinematicsStudy::InverseKinematicsStudy()
{
    constructProperties();
}

InverseKinematicsStudy::InverseKinematicsStudy(const std::string& setupFile)
    : Object(setupFile, true)
{
    constructProperties();
    updateFromXMLDocument();
}

InverseKinematicsStudy::~InverseKinematicsStudy()
{
}

void InverseKinematicsStudy::constructProperties()
{
    constructProperty_accuracy(1e-6);
    constructProperty_constraint_weight(Infinity);
    Array<double> range{ Infinity, 2};
    constructProperty_time_range(range);

    constructProperty_base_imu_label("");
    constructProperty_base_heading_axis("z");

    constructProperty_model_file_name("");
    constructProperty_marker_file_name("");
    constructProperty_orientations_file_name("");

    constructProperty_results_directory("");
}

void InverseKinematicsStudy::
    previewExperimentalData(const TimeSeriesTableVec3& markers,
                const TimeSeriesTable_<SimTK::Rotation>& orientations) const
{
    Model previewWorld;

    // Load the marker data into a TableSource that has markers
    // as its output which each markers occupying its own channel
    TableSourceVec3* markersSource = new TableSourceVec3(markers);
    // Add the markersSource Component to the model
    previewWorld.addComponent(markersSource);

    // Get the underlying Table backing the the marker Source so we 
    // know how many markers we have and their names
    const auto& markerData = markersSource->getTable();
    auto& times = markerData.getIndependentColumn();

    auto startEnd = getTimeRangeInUse(times);

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
    std::cout << "press any key to visualize experimental marker data ..." << std::endl;
    std::cin >> c;

    for (int j =startEnd[0]; j <= startEnd[1]; j=j+10) {
        std::cout << "time: " << times[j] << "s" << std::endl;
        state.setTime(times[j]);
        previewWorld.realizePosition(state);
        previewWorld.getVisualizer().show(state);
    }
}

void InverseKinematicsStudy::
runInverseKinematicsWithOrientationsFromFile(Model& model,
    const std::string& orientationsFileName,
    bool visualizeResults)
{
    // Add a reporter to get IK computed coordinate values out
    TableReporter* ikReporter = new TableReporter();
    ikReporter->setName("ik_reporter");
    auto coordinates = model.updComponentList<Coordinate>();

    // Hookup reporter inputs to the individual coordinate outputs
    // and lock coordinates that are translational since they cannot be
    for (auto& coord : coordinates) {
        ikReporter->updInput("inputs").connect(
            coord.getOutput("value"), coord.getName());
        if(coord.getMotionType() == Coordinate::Translational) {
            coord.setDefaultLocked(true);
        }
    }

    model.addComponent(ikReporter);

    TimeSeriesTable_<SimTK::Quaternion> quatTable(orientationsFileName);
    std::cout << "Loading orientations as quaternions from "
        << orientationsFileName << std::endl;

    auto startEnd = getTimeRangeInUse(quatTable.getIndependentColumn());

    const auto axis_string = IO::Lowercase(get_base_heading_axis());

    int axis = (axis_string == "x" ? 0 : 
                                    ((axis_string == "y") ? 1 : 2) );
    SimTK::CoordinateAxis heading{ axis };


    cout << "Heading correction for base '" << get_base_imu_label()
        << "' along its '" << axis_string << "' axis." << endl;

    TimeSeriesTable_<SimTK::Rotation> orientationsData =
        OpenSenseUtilities::convertQuaternionsToRotations(quatTable,
             startEnd, get_base_imu_label(), heading);

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
            cout << "Solved frame at time: " << time << endl;
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
}


// main driver
bool InverseKinematicsStudy::run(bool visualizeResults)
{
    if (_model.empty()) {
        _model.reset(new Model(get_model_file_name()));
    }

    runInverseKinematicsWithOrientationsFromFile(*_model,
                                                 get_orientations_file_name(),
                                                 visualizeResults);

    return true;
}

SimTK::Array_<int> InverseKinematicsStudy::getTimeRangeInUse(
                                const std::vector<double>& times ) const
{
    int nt = static_cast<int>(times.size());
    int startIx = 0;
    int endIx = nt-1;

    for (int i = 0; i < nt; ++i) {
        if (times[i] <= get_time_range(0)) {
            startIx = i;
        }
        else {
            break;
        }
    }

    for (int i = nt - 1; i > 0; --i) {
        if (times[i] >= get_time_range(1)) {
            endIx= i;
        }
        else {
            break;
        }
    }
    SimTK::Array_<int> retArray;
    retArray.push_back(startIx);
    retArray.push_back(endIx);
    return retArray;
}

TimeSeriesTable_<SimTK::Vec3> 
    InverseKinematicsStudy::loadMarkersFile(const std::string& markerFile)
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