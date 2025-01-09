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
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/CoordinateReference.h>
#include "IKCoordinateTask.h"
#include "IKTaskSet.h"
#include <OpenSim/Analyses/StatesReporter.h>
#include <OpenSim/Common/IO.h>
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
MarkerPlacer::MarkerPlacer() :
    _apply(_applyProp.getValueBool()),
    _markerFileName(_markerFileNameProp.getValueStr()),
    _timeRange(_timeRangeProp.getValueDblArray()),
    _ikTaskSetProp(PropertyObj("", IKTaskSet())),
    _ikTaskSet((IKTaskSet&)_ikTaskSetProp.getValueObj()),
    _coordinateFileName(_coordinateFileNameProp.getValueStr()),
    _outputModelFileName(_outputModelFileNameProp.getValueStr()),
    _outputMarkerFileName(_outputMarkerFileNameProp.getValueStr()),
    _outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
    _maxMarkerMovement(_maxMarkerMovementProp.getValueDbl())
{
    setNull();
    setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
MarkerPlacer::~MarkerPlacer()
{
    //delete _ikTrial;
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMarkerPlacer MarkerPlacer to be copied.
 */
MarkerPlacer::MarkerPlacer(const MarkerPlacer &aMarkerPlacer) :
   Object(aMarkerPlacer),
    _apply(_applyProp.getValueBool()),
   _markerFileName(_markerFileNameProp.getValueStr()),
    _timeRange(_timeRangeProp.getValueDblArray()),
    _ikTaskSetProp(PropertyObj("", IKTaskSet())),
    _ikTaskSet((IKTaskSet&)_ikTaskSetProp.getValueObj()),
    _coordinateFileName(_coordinateFileNameProp.getValueStr()),
    _outputModelFileName(_outputModelFileNameProp.getValueStr()),
    _outputMarkerFileName(_outputMarkerFileNameProp.getValueStr()),
    _outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
    _maxMarkerMovement(_maxMarkerMovementProp.getValueDbl())
{
    setNull();
    setupProperties();
    copyData(aMarkerPlacer);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one MarkerPlacer to another.
 *
 * @param aMarkerPlacer MarkerPlacer to be copied.
 */
void MarkerPlacer::copyData(const MarkerPlacer &aMarkerPlacer)
{
    _apply = aMarkerPlacer._apply;
    _markerFileName = aMarkerPlacer._markerFileName;
    _timeRange = aMarkerPlacer._timeRange;
    _ikTaskSet = aMarkerPlacer._ikTaskSet;
    _coordinateFileName = aMarkerPlacer._coordinateFileName;
    _outputModelFileName = aMarkerPlacer._outputModelFileName;
    _outputMarkerFileName = aMarkerPlacer._outputMarkerFileName;
    _outputMotionFileName = aMarkerPlacer._outputMotionFileName;
    _maxMarkerMovement = aMarkerPlacer._maxMarkerMovement;
    _printResultFiles = aMarkerPlacer._printResultFiles;
}

//_____________________________________________________________________________
/**
 * Set the data members of this MarkerPlacer to their null values.
 */
void MarkerPlacer::setNull()
{
    _apply = true;
    _coordinateFileName = "";

    _printResultFiles = true;
    _moveModelMarkers = true;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MarkerPlacer::setupProperties()
{
    _applyProp.setComment("Whether or not to use the marker placer during scale");
    _applyProp.setName("apply");
    _propertySet.append(&_applyProp);

    _ikTaskSetProp.setComment("Task set used to specify weights used in the IK computation of the static pose.");
    _ikTaskSetProp.setName("IKTaskSet");
    _propertySet.append(&_ikTaskSetProp);

    _markerFileNameProp.setComment("TRC file (.trc) containing the time history of experimental marker positions "
        "(usually a static trial).");
    _markerFileNameProp.setName("marker_file");
    _propertySet.append(&_markerFileNameProp);

    _coordinateFileNameProp.setComment("Name of file containing the joint angles "
        "used to set the initial configuration of the model for the purpose of placing the markers. "
        "These coordinate values can also be included in the optimization problem used to place the markers. "
        "Before the model markers are placed, a single frame of an inverse kinematics (IK) problem is solved. "
        "The IK problem can be solved simply by matching marker positions, but if the model markers are not "
        "in the correct locations, the IK solution will not be very good and neither will marker placement. "
        "Alternatively, coordinate values (specified in this file) can be specified and used to influence the IK solution. "
        "This is valuable particularly if you have high confidence in the coordinate values. "
        "For example, you know for the static trial the subject was standing will all joint angles close to zero. "
        "If the coordinate set (see the CoordinateSet property) contains non-zero weights for coordinates, "
        "the IK solution will try to match not only the marker positions, but also the coordinates in this file. "
        "Least-squared error is used to solve the IK problem. ");
    _coordinateFileNameProp.setName("coordinate_file");
    _propertySet.append(&_coordinateFileNameProp);

    _timeRangeProp.setComment("Time range over which the marker positions are averaged.");
    const double defaultTimeRange[] = {-1.0, -1.0};
    _timeRangeProp.setName("time_range");
    _timeRangeProp.setValue(2, defaultTimeRange);
    _timeRangeProp.setAllowableListSize(2);
    _propertySet.append(&_timeRangeProp);

    _outputMotionFileNameProp.setComment("Name of the motion file (.mot) written after marker relocation (optional).");
    _outputMotionFileNameProp.setName("output_motion_file");
    _propertySet.append(&_outputMotionFileNameProp);

    _outputModelFileNameProp.setComment("Output OpenSim model file (.osim) after scaling and maker placement.");
    _outputModelFileNameProp.setName("output_model_file");
    _propertySet.append(&_outputModelFileNameProp);

    _outputMarkerFileNameProp.setComment("Output marker set containing the new marker locations after markers have been placed.");
    _outputMarkerFileNameProp.setName("output_marker_file");
    _propertySet.append(&_outputMarkerFileNameProp);

    _maxMarkerMovementProp.setComment("Maximum amount of movement allowed in marker data when averaging frames of the static trial. "
        "A negative value means there is not limit.");
    _maxMarkerMovementProp.setName("max_marker_movement");
    _maxMarkerMovementProp.setValue(-1.0); // units of this value are the units of the marker data in the static pose (usually mm)
    _propertySet.append(&_maxMarkerMovementProp);
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
MarkerPlacer& MarkerPlacer::operator=(const MarkerPlacer &aMarkerPlacer)
{
    // BASE CLASS
    Object::operator=(aMarkerPlacer);

    copyData(aMarkerPlacer);

    return(*this);
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
bool MarkerPlacer::processModel(Model* aModel,
        const string& aPathToSubject) const {

    if(!getApply()) return false;

    log_info("Step 3: Placing markers on model");

    if (_timeRange.getSize()<2) 
        throw Exception("MarkerPlacer::processModel, time_range is unspecified.");

    /* Load the static pose marker file, and average all the
    * frames in the user-specified time range.
    */
    TimeSeriesTableVec3 staticPoseTable{aPathToSubject + _markerFileName};
    const auto& timeCol = staticPoseTable.getIndependentColumn();

    // Users often set a time range that purposely exceeds the range of
    // their data with the mindset that all their data will be used.
    // To allow for that, we have to narrow the provided range to data
    // range, since the TimeSeriesTable will correctly throw that the 
    // desired time exceeds the data range.
    if (_timeRange[0] < timeCol.front())
        _timeRange[0] = timeCol.front();
    if (_timeRange[1] > timeCol.back())
        _timeRange[1] = timeCol.back();

    const auto avgRow = staticPoseTable.averageRow(_timeRange[0],
                                                   _timeRange[1]);
    for(size_t r = staticPoseTable.getNumRows(); r-- > 0; )
        staticPoseTable.removeRowAtIndex(r);
    staticPoseTable.appendRow(_timeRange[0], avgRow);
    
    OPENSIM_THROW_IF(!staticPoseTable.hasTableMetaDataKey("Units"),
                     Exception,
                     "MarkerPlacer::processModel -- Marker file does not have "
                     "'Units'.");
    Units
    staticPoseUnits{staticPoseTable.getTableMetaData<std::string>("Units")};
    double scaleFactor = staticPoseUnits.convertTo(aModel->getLengthUnits());
    OPENSIM_THROW_IF(SimTK::isNaN(scaleFactor),
                     Exception,
                     "Model has unspecified units.");
    if(std::fabs(scaleFactor - 1) >= SimTK::Eps) {
        for(unsigned r = 0; r < staticPoseTable.getNumRows(); ++r)
            staticPoseTable.updRowAtIndex(r) *= scaleFactor;

        staticPoseUnits = aModel->getLengthUnits();
        staticPoseTable.removeTableMetaDataKey("Units");
        staticPoseTable.addTableMetaData("Units",
                                         staticPoseUnits.getAbbreviation());
    }
    
    MarkerData* staticPose = new MarkerData(aPathToSubject + _markerFileName);
    staticPose->averageFrames(_maxMarkerMovement, _timeRange[0], _timeRange[1]);
    staticPose->convertToUnits(aModel->getLengthUnits());

    /* Delete any markers from the model that are not in the static
     * pose marker file.
     */
    aModel->deleteUnusedMarkers(staticPose->getMarkerNames());

    // Construct the system and get the working state when done changing the model
    SimTK::State& s = aModel->initSystem();
    s.updTime() = _timeRange[0];
    
    // Create references and WeightSets needed to initialize InverseKinemaicsSolver
    Set<MarkerWeight> markerWeightSet;
    _ikTaskSet.createMarkerWeightSet(markerWeightSet); // order in tasks file
    // MarkersReference takes ownership of marker data (staticPose)
    std::shared_ptr<MarkersReference> markersReference(new MarkersReference(staticPoseTable, markerWeightSet));
    SimTK::Array_<CoordinateReference> coordinateReferences;

    // Load the coordinate data
    // create CoordinateReferences for Coordinate Tasks
    FunctionSet *coordFunctions = NULL;
    // bool haveCoordinateFile = false;
    if(_coordinateFileName != "" && _coordinateFileName != "Unassigned"){
        Storage coordinateValues(aPathToSubject + _coordinateFileName);
        aModel->getSimbodyEngine().convertDegreesToRadians(coordinateValues);
        // haveCoordinateFile = true;
        coordFunctions = new GCVSplineSet(5,&coordinateValues);
    }
    
    int index = 0;
    for(int i=0; i< _ikTaskSet.getSize(); i++){
        IKCoordinateTask *coordTask = dynamic_cast<IKCoordinateTask *>(&_ikTaskSet[i]);
        if (coordTask && coordTask->getApply()){
            std::unique_ptr<CoordinateReference> coordRef{};
            if(coordTask->getValueType() == IKCoordinateTask::FromFile){
                index = coordFunctions->getIndex(coordTask->getName(), index);
                if(index >= 0){
                    coordRef.reset(new CoordinateReference(coordTask->getName(),coordFunctions->get(index)));
                }
            }
            else if((coordTask->getValueType() == IKCoordinateTask::ManualValue)){
                Constant reference(Constant(coordTask->getValue()));
                coordRef.reset(new CoordinateReference(coordTask->getName(), reference));
            }
            else{ // assume it should be held at its current/default value
                double value = aModel->getCoordinateSet().get(coordTask->getName()).getValue(s);
                Constant reference = Constant(value);
                coordRef.reset(new CoordinateReference(coordTask->getName(), reference));
            }

            if(coordRef == NULL)
                throw Exception("MarkerPlacer: value for coordinate "+coordTask->getName()+" not found.");

            // We have a valid coordinate reference so now set its weight according to the task
            coordRef->setWeight(coordTask->getWeight());
            coordinateReferences.push_back(*coordRef);      
        }           
    }
    double constraintWeight = std::numeric_limits<SimTK::Real>::infinity();

    InverseKinematicsSolver ikSol(*aModel, markersReference,
                                  coordinateReferences, constraintWeight);
    ikSol.assemble(s);

    // Call realize Position so that the transforms are updated and  markers can be moved correctly
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
    for(int j=0; j<nm; ++j){
        totalSquaredMarkerError += squaredMarkerErrors[j];
        if(squaredMarkerErrors[j] > maxSquaredMarkerError){
            maxSquaredMarkerError = squaredMarkerErrors[j];
            worst = j;
        }
    }
    log_info("Frame at (t = {}):\t total squared error = {}, "
             "marker error: RMS = {}, max = {} ({})",
            s.getTime(), totalSquaredMarkerError,
            sqrt(totalSquaredMarkerError/nm),
            sqrt(maxSquaredMarkerError),
            ikSol.getMarkerNameForIndex(worst));
    /* Now move the non-fixed markers on the model so that they are coincident
     * with the measured markers in the static pose. The model is already in
     * the proper configuration so the coordinates do not need to be changed.
     */
    if(_moveModelMarkers) moveModelMarkersToPose(s, *aModel, *staticPose);

    _outputStorage.reset();
    // Make a storage file containing the solved states and markers for display in GUI.
    Storage motionData;
    StatesReporter statesReporter(aModel);
    statesReporter.begin(s);
    
    _outputStorage.reset(new Storage(statesReporter.updStatesStorage()));
    _outputStorage->setName("static pose");
    _outputStorage->getStateVector(0)->setTime(s.getTime());

    if(_printResultFiles) {
        auto cwd = IO::CwdChanger::changeTo(aPathToSubject);

        if (_outputModelFileNameProp.isValidFileName()) {
            aModel->print(aPathToSubject + _outputModelFileName);
            log_info("Wrote model file '{}' from model {}.",
                _outputModelFileName, aModel->getName());
        }

        if (_outputMarkerFileNameProp.isValidFileName()) {
            aModel->writeMarkerFile(aPathToSubject + _outputMarkerFileName);
            log_info("Wrote marker file '{}' from model {}.",
                _outputMarkerFileName, aModel->getName());
        }

        if (_outputMotionFileNameProp.isValidFileName()) {
            _outputStorage->print(aPathToSubject + _outputMotionFileName,
                "w", "File generated from solving marker data for model "
                + aModel->getName());
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
void MarkerPlacer::moveModelMarkersToPose(SimTK::State& s, Model& aModel,
        MarkerData& aPose) const
{
    aPose.averageFrames(0.01);
    const MarkerFrame &frame = aPose.getFrame(0);

    // const SimbodyEngine& engine = aModel.getSimbodyEngine();

    MarkerSet& markerSet = aModel.updMarkerSet();

    int i;
    for (i = 0; i < markerSet.getSize(); i++)
    {
        Marker& modelMarker = markerSet.get(i);

        if (!modelMarker.get_fixed())
        {
            int index = aPose.getMarkerIndex(modelMarker.getName());
            if (index >= 0)
            {
                Vec3 globalMarker = frame.getMarker(index);
                if (!globalMarker.isNaN())
                {
                    Vec3 pt, pt2;
                    Vec3 globalPt = globalMarker;
                    double conversionFactor = aPose.getUnits().convertTo(aModel.getLengthUnits());
                    pt = conversionFactor*globalPt;
                    pt2 = aModel.getGround().findStationLocationInAnotherFrame(s, pt, modelMarker.getParentFrame());
                    modelMarker.set_location(pt2);
                }
                else
                {
                    log_warn("Marker {} does not have valid coordinates in "
                             "'{}'. It will not be moved to match location in "
                             "marker file.", 
                        modelMarker.getName(), aPose.getFileName());
                }
            }
        }
    }

    log_info("Moved markers in model {} to match locations in marker file "
             "'{}'.", aModel.getName(), aPose.getFileName());
}

Storage* MarkerPlacer::getOutputStorage() 
{
    return _outputStorage.get();
}
