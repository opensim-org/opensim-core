
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

    for (size_t j = std::get<0>(startEnd); j <= std::get<1>(startEnd); j=j+10) {
        std::cout << "time: " << times[j] << "s" << std::endl;
        state.setTime(times[j]);
        previewWorld.realizePosition(state);
        previewWorld.getVisualizer().show(state);
    }
}

void InverseKinematicsStudy::
    runInverseKinematicsWithOrientationsFromFile(Model& model,
                                        const std::string& orientationsFileName)
{
    // Add a reporter to get IK computed coordinate values out
    TableReporter* ikReporter = new TableReporter();
    ikReporter->setName("ik_reporter");
    auto coordinates = model.updComponentList<Coordinate>();
    std::vector<std::string> transCoordNames;

    // Hookup reporter inputs to the individual coordinate outputs
    // and keep track of coordinates that are translations
    for (auto& coord : coordinates) {
        ikReporter->updInput("inputs").connect(
            coord.getOutput("value"), coord.getName());
        if (coord.getMotionType() == Coordinate::Translational) {
            transCoordNames.push_back(coord.getName());
            coord.set_locked(true);
        }
    }
    model.addComponent(ikReporter);

    TimeSeriesTable_<SimTK::Quaternion> quatTable =
        STOFileAdapter_<SimTK::Quaternion>::read(orientationsFileName);
    std::cout << "Loading orientations as quaternions from " 
        << orientationsFileName << std::endl;

    auto startEnd = getTimeRangeInUse(quatTable.getIndependentColumn());

    TimeSeriesTable_<SimTK::Rotation> orientationsData =
        OpenSenseUtilities::convertQuaternionsToRotations(quatTable, startEnd);

    OrientationsReference oRefs(orientationsData);
    MarkersReference mRefs{};

    SimTK::Array_<CoordinateReference> coordinateReferences;

    // visualize for debugging
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
    model.getVisualizer().show(s0);
    model.getVisualizer().getSimbodyVisualizer().setShowSimTime(true);

    for (auto time : times) {
        s0.updTime() = time;
        ikSolver.track(s0);
        model.getVisualizer().show(s0);
        // realize to report to get reporter to pull values from model
        model.realizeReport(s0);
    }

    auto report = ikReporter->getTable();

    auto eix = orientationsFileName.rfind("_");
    if (eix == std::string::npos) {
        eix = orientationsFileName.rfind(".");
    }
    auto stix = orientationsFileName.rfind("/") + 1;

    IO::makeDir(get_results_directory());
    std::string outName = "ik_" + orientationsFileName.substr(stix, eix-stix);
    std::string outputFile = get_results_directory() + "/" + outName;

    // Convert to degrees to compare with marker-based IK
    // but ignore translational coordinates
    for (size_t i = 0; i < report.getNumColumns(); ++i) {
        auto it = find( transCoordNames.begin(), transCoordNames.end(),
                        report.getColumnLabel(i) );
        if (it == transCoordNames.end()) { // not found = not translational
            auto repVec = report.updDependentColumnAtIndex(i);
            repVec *= SimTK::Real(SimTK_RTD);
        }
    }

    report.updTableMetaData().setValueForKey<string>("name", outName);
    report.updTableMetaData().setValueForKey<size_t>("nRows", report.getNumRows());
    // getNumColumns returns the number of dependent columns, but Storage expects time
    report.updTableMetaData().setValueForKey<size_t>("nColumns", report.getNumColumns()+1);
    report.updTableMetaData().setValueForKey<string>("inDegrees","yes");

    STOFileAdapter_<double>::write(report, outputFile + ".mot");

    std::cout << "Wrote IK with IMU tracking results to: '" <<
        outputFile << "'." << std::endl;
}


// main driver
bool InverseKinematicsStudy::run()
{
    bool correct_offsets = false;
    // Supply offset corrections due to incorrect placement, etc... as X, Y, Z body fixed rotation
    // Corrections are premultiplied. [C]*[R]
    std::map<std::string, SimTK::Vec3> imu_corections{
        {"back_imu", {0, 0, 90}},
        {"pelvis_imu", {0, 0, 90}} };

    Model modelWithIMUs(get_model_file_name());

    if (correct_offsets) {
        for (auto it : imu_corections) {
            auto* offset = modelWithIMUs.findComponent<PhysicalOffsetFrame>(it.first);
            SimTK::Transform transform = offset->getOffsetTransform();
            Vec3 seq = it.second;
            Rotation correction{
                SimTK::BodyOrSpaceType::BodyRotationSequence,
                double(SimTK_DEGREE_TO_RADIAN*seq[0]), SimTK::XAxis,
                double(SimTK_DEGREE_TO_RADIAN*seq[1]), SimTK::YAxis,
                double(SimTK_DEGREE_TO_RADIAN*seq[2]), SimTK::ZAxis };
            transform.updR() = correction*transform.R();
            modelWithIMUs.updComponent<PhysicalOffsetFrame>(offset->getAbsolutePath())
                .setOffsetTransform(transform);
        }
    }

    runInverseKinematicsWithOrientationsFromFile(modelWithIMUs,
                                                 get_orientations_file_name());

    return true;
}

std::tuple<size_t, size_t > InverseKinematicsStudy::getTimeRangeInUse(
                                const std::vector<double>& times ) const
{
    size_t nt = times.size();
    size_t startIx = 0;
    size_t endIx = nt-1;

    for (size_t i = 0; i < nt; ++i) {
        if (times[i] <= get_time_range(0)) {
            startIx = i;
        }
        else {
            break;
        }
    }

    for (size_t i = nt - 1; i > 0; --i) {
        if (times[i] >= get_time_range(1)) {
            endIx= i;
        }
        else {
            break;
        }
    }
    return std::make_tuple(startIx, endIx);
}

TimeSeriesTable_<SimTK::Vec3> 
    InverseKinematicsStudy::loadMarkersFile(const std::string& markerFile)
{
    auto markers = TRCFileAdapter::read(markerFile);
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