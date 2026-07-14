/* -------------------------------------------------------------------------- *
 *                         OpenSim:  MarkerPlacer.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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
#include "MarkerPlacer.h"

#include "IKCoordinateTask.h"
#include "OpenSim/Tools/IKMarkerTask.h"
#include "OpenSim/Tools/IKTaskSet.h"
#include <memory>

#include <OpenSim/Analyses/StatesReporter.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Simulation/CoordinateReference.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
MarkerPlacer::MarkerPlacer() { constructProperties(); }

//_____________________________________________________________________________
/**
 * Destructor.
 */
MarkerPlacer::~MarkerPlacer() {
    // delete _ikTrial;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MarkerPlacer::constructProperties() {
    constructProperty_apply(true);
    constructProperty_IKTaskSet(IKTaskSet());
    constructProperty_marker_file("");
    constructProperty_coordinate_file("");
    const Array<double> defaultTimeRange = {-1.0, -1.0};
    constructProperty_time_range(defaultTimeRange);
    constructProperty_output_motion_file("");
    constructProperty_output_model_file("");
    constructProperty_output_marker_file("");
    constructProperty_max_marker_movement(-1.0);
    constructProperty_print_result_files(true);
    constructProperty_move_model_markers(true);
}

void MarkerPlacer::registerTypes() {
    Object::registerType(IKCoordinateTask());
    Object::registerType(IKMarkerTask());
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * This method creates a SimmMotionTrial instance with the markerFile and
 * timeRange parameters. It also creates a Storage instance with the
 * coordinateFile parameter. Then it updates the coordinates and markers in
 * the model, if specified. Then it does IK to fit the model to the static
 * pose. Then it uses the current model pose to relocate all non-fixed markers
 * according to their locations in the SimmMotionTrial. Then it writes the
 * output files selected by the user.
 *
 * @param aModel the model to use for the marker placing process.
 * @return Whether the marker placing process was successful or not.
 */
bool MarkerPlacer::processModel(
        Model* aModel, const string& aPathToSubject) const {

    if (!getApply()) return false;

    log_info("Step 3: Placing markers on model");

    if (getProperty_time_range().size() < 2)
        throw Exception(
                "MarkerPlacer::processModel, time_range is unspecified.");

    /* Load the static pose marker file, and average all the
     * frames in the user-specified time range.
     */
    TimeSeriesTableVec3 staticPoseTable{aPathToSubject + get_marker_file()};
    const auto& timeCol = staticPoseTable.getIndependentColumn();

    // Users often set a time range that purposely exceeds the range of
    // their data with the mindset that all their data will be used.
    // To allow for that, we have to narrow the provided range to data
    // range, since the TimeSeriesTable will correctly throw that the
    // desired time exceeds the data range.
    const auto time_range = getTimeRange();
    if (time_range[0] < timeCol.front()) time_range[0] = timeCol.front();
    if (time_range[1] > timeCol.back()) time_range[1] = timeCol.back();

    const auto avgRow =
            staticPoseTable.averageRow(time_range[0], time_range[1]);
    for (size_t r = staticPoseTable.getNumRows(); r-- > 0;)
        staticPoseTable.removeRowAtIndex(r);
    staticPoseTable.appendRow(time_range[0], avgRow);

    OPENSIM_THROW_IF(!staticPoseTable.hasTableMetaDataKey("Units"), Exception,
            "MarkerPlacer::processModel -- Marker file does not have "
            "'Units'.");
    Units staticPoseUnits{
            staticPoseTable.getTableMetaData<std::string>("Units")};
    double scaleFactor = staticPoseUnits.convertTo(aModel->getLengthUnits());
    OPENSIM_THROW_IF(SimTK::isNaN(scaleFactor), Exception,
            "Model has unspecified units.");
    if (std::fabs(scaleFactor - 1) >= SimTK::Eps) {
        for (unsigned r = 0; r < staticPoseTable.getNumRows(); ++r)
            staticPoseTable.updRowAtIndex(r) *= scaleFactor;

        staticPoseUnits = aModel->getLengthUnits();
        staticPoseTable.removeTableMetaDataKey("Units");
        staticPoseTable.addTableMetaData(
                "Units", staticPoseUnits.getAbbreviation());
    }

    MarkerData staticPose = MarkerData(aPathToSubject + get_marker_file());
    staticPose.averageFrames(
            get_max_marker_movement(), time_range[0], time_range[1]);
    staticPose.convertToUnits(aModel->getLengthUnits());

    /* Delete any markers from the model that are not in the static
     * pose marker file.
     */
    aModel->deleteUnusedMarkers(staticPose.getMarkerNames());

    // Construct the system and get the working state when done changing the
    // model
    SimTK::State& s = aModel->initSystem();
    s.updTime() = time_range[0];

    // Create references and WeightSets needed to initialize
    // InverseKinemaicsSolver
    Set<MarkerWeight> markerWeightSet;
    get_IKTaskSet().createMarkerWeightSet(
            markerWeightSet); // order in tasks file
    // MarkersReference takes ownership of marker data (staticPose)
    auto markersReference = std::make_shared<MarkersReference>(
            staticPoseTable, markerWeightSet);
    SimTK::Array_<CoordinateReference> coordinateReferences;

    // Load the coordinate data
    // create CoordinateReferences for Coordinate Tasks
    std::unique_ptr<FunctionSet> coordFunctions;
    // bool haveCoordinateFile = false;
    if (get_coordinate_file() != "" && get_coordinate_file() != "Unassigned") {
        Storage coordinateValues(aPathToSubject + get_coordinate_file());
        aModel->getSimbodyEngine().convertDegreesToRadians(coordinateValues);
        // haveCoordinateFile = true;
        coordFunctions = std::make_unique<GCVSplineSet>(5, &coordinateValues);
    }

    int index = 0;
    for (int i = 0; i < get_IKTaskSet().getSize(); i++) {
        const IKCoordinateTask* coordTask =
                dynamic_cast<const IKCoordinateTask*>(&get_IKTaskSet()[i]);
        // auto& coordTask = get_ik_task_set(i);
        // auto& coordTaskProperty = getProperty_ik_task_set();
        if (coordTask && coordTask->getApply()) {
            std::unique_ptr<CoordinateReference> coordRef;
            if (coordTask->getValueType() == IKCoordinateTask::FromFile) {
                index = coordFunctions->getIndex(coordTask->getName(), index);
                if (index >= 0) {
                    coordRef = std::make_unique<CoordinateReference>(
                            coordTask->getName(), coordFunctions->get(index));
                }
            } else if ((coordTask->getValueType() ==
                               IKCoordinateTask::ManualValue)) {
                Constant reference(Constant(coordTask->getValue()));
                coordRef = std::make_unique<CoordinateReference>(
                        coordTask->getName(), reference);
            } else { // assume it should be held at its current/default value
                double value = aModel->getCoordinateSet()
                                       .get(coordTask->getName())
                                       .getValue(s);
                Constant reference = Constant(value);
                coordRef = std::make_unique<CoordinateReference>(
                        coordTask->getName(), reference);
            }

            if (coordRef == NULL)
                throw Exception("MarkerPlacer: value for coordinate " +
                                coordTask->getName() + " not found.");

            // We have a valid coordinate reference so now set its weight
            // according to the task
            coordRef->setWeight(coordTask->getWeight());
            coordinateReferences.push_back(*coordRef);
        }
    }
    double constraintWeight = std::numeric_limits<SimTK::Real>::infinity();

    InverseKinematicsSolver ikSol(
            *aModel, markersReference, coordinateReferences, constraintWeight);
    ikSol.assemble(s);

    // Call realize Position so that the transforms are updated and  markers can
    // be moved correctly
    aModel->getMultibodySystem().realize(s, SimTK::Stage::Position);
    // Report marker errors to assess the quality
    int nm = markerWeightSet.getSize();
    SimTK::Array_<double> squaredMarkerErrors(nm, 0.0);
    SimTK::Array_<Vec3> markerLocations(nm, Vec3(0));
    double totalSquaredMarkerError = 0.0;
    double maxSquaredMarkerError = 0.0;
    int worst = -1;
    // Report in the same order as the marker tasks/weights
    ikSol.computeCurrentSquaredMarkerErrors(squaredMarkerErrors);
    for (int j = 0; j < nm; ++j) {
        totalSquaredMarkerError += squaredMarkerErrors[j];
        if (squaredMarkerErrors[j] > maxSquaredMarkerError) {
            maxSquaredMarkerError = squaredMarkerErrors[j];
            worst = j;
        }
    }
    log_info("Frame at (t = {}):\t total squared error = {}, "
             "marker error: RMS = {}, max = {} ({})",
            s.getTime(), totalSquaredMarkerError,
            sqrt(totalSquaredMarkerError / nm), sqrt(maxSquaredMarkerError),
            ikSol.getMarkerNameForIndex(worst));
    /* Now move the non-fixed markers on the model so that they are coincident
     * with the measured markers in the static pose. The model is already in
     * the proper configuration so the coordinates do not need to be changed.
     */
    if (get_move_model_markers())
        moveModelMarkersToPose(s, *aModel, staticPose);

    _outputStorage.reset();
    // Make a storage file containing the solved states and markers for display
    // in GUI.
    Storage motionData;
    StatesReporter statesReporter(aModel);
    statesReporter.begin(s);

    _outputStorage =
            std::make_unique<Storage>(statesReporter.updStatesStorage());
    _outputStorage->setName("static pose");
    _outputStorage->getStateVector(0)->setTime(s.getTime());

    if (get_print_result_files()) {
        auto cwd = IO::CwdChanger::changeTo(aPathToSubject);

        if (getProperty_output_model_file().isValidFileName()) {
            aModel->print(aPathToSubject + get_output_model_file());
            log_info("Wrote model file '{}' from model {}.",
                    get_output_model_file(), aModel->getName());
        }

        if (getProperty_output_marker_file().isValidFileName()) {
            aModel->writeMarkerFile(aPathToSubject + get_output_marker_file());
            log_info("Wrote marker file '{}' from model {}.",
                    get_output_marker_file(), aModel->getName());
        }

        if (getProperty_output_motion_file().isValidFileName()) {
            _outputStorage->print(aPathToSubject + get_output_motion_file(),
                    "w",
                    "File generated from solving marker data for model " +
                            aModel->getName());
        }
    }

    return true;
}

//_____________________________________________________________________________
/**
 * Set the local offset of each non-fixed marker so that in the model's
 * current pose the marker coincides with the marker's global position
 * in the passed-in MarkerData.
 *
 * @param aModel the model to use
 * @param aPose the static-pose marker cloud to get the marker locations from
 */
void MarkerPlacer::moveModelMarkersToPose(
        SimTK::State& s, Model& aModel, MarkerData& aPose) const {
    aPose.averageFrames(0.01);
    const MarkerFrame& frame = aPose.getFrame(0);

    // const SimbodyEngine& engine = aModel.getSimbodyEngine();

    MarkerSet& markerSet = aModel.updMarkerSet();

    int i;
    for (i = 0; i < markerSet.getSize(); i++) {
        Marker& modelMarker = markerSet.get(i);

        if (!modelMarker.get_fixed()) {
            int index = aPose.getMarkerIndex(modelMarker.getName());
            if (index >= 0) {
                Vec3 globalMarker = frame.getMarker(index);
                if (!globalMarker.isNaN()) {
                    Vec3 pt, pt2;
                    Vec3 globalPt = globalMarker;
                    double conversionFactor =
                            aPose.getUnits().convertTo(aModel.getLengthUnits());
                    pt = conversionFactor * globalPt;
                    pt2 = aModel.getGround().findStationLocationInAnotherFrame(
                            s, pt, modelMarker.getParentFrame());
                    modelMarker.set_location(pt2);
                } else {
                    log_warn("Marker {} does not have valid coordinates in "
                             "'{}'. It will not be moved to match location in "
                             "marker file.",
                            modelMarker.getName(), aPose.getFileName());
                }
            }
        }
    }

    log_info("Moved markers in model {} to match locations in marker file "
             "'{}'.",
            aModel.getName(), aPose.getFileName());
}

Storage* MarkerPlacer::getOutputStorage() { return _outputStorage.get(); }
