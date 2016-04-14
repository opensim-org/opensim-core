/* -------------------------------------------------------------------------- *
 *                         OpenSim:  MarkerPlacer.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/Model/Marker.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/CoordinateReference.h>
#include "IKCoordinateTask.h"
#include <OpenSim/Analyses/StatesReporter.h>
#include <OpenSim/Common/Adapters.h>
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
    _outputStorage = NULL;
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
    _outputStorage = NULL;
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
bool MarkerPlacer::processModel(Model* aModel, const string& aPathToSubject)
{
    if(!getApply()) return false;

    cout << endl << "Step 3: Placing markers on model" << endl;

    /* Load the static pose marker file, and average all the
     * frames in the user-specified time range.
     */
    auto&& markers = FileAdapter::readFile(aPathToSubject + _markerFileName);
    auto staticPose = dynamic_cast<MarkerTable&>(*(markers.at("markers")));


    if (_timeRange.getSize()<2) 
        throw Exception("MarkerPlacer::processModel, time_range is unspecified.");

    averageFrames(staticPose, _maxMarkerMovement, _timeRange[0], _timeRange[1]);

    convertUnits(aModel, staticPose);

    /* Delete any markers from the model that are not in the static
     * pose marker file.
     */
    auto& labels = staticPose.
                   getDependentsMetaData().
                   getValueArrayForKey("labels");
    Array<std::string> markerNames{};
    for(size_t i = 0; i < labels.size(); ++i)
        markerNames.append(labels[i].getValue<std::string>());
    aModel->deleteUnusedMarkers(markerNames);

    // Construct the system and get the working state when done changing the model
    SimTK::State& s = aModel->initSystem();
    
    // Create references and WeightSets needed to initialize InverseKinemaicsSolver
    Set<MarkerWeight> markerWeightSet;
    _ikTaskSet.createMarkerWeightSet(markerWeightSet); // order in tasks file
    MarkersReference markersReference(staticPose, &markerWeightSet);
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
            CoordinateReference *coordRef = NULL;
            if(coordTask->getValueType() == IKCoordinateTask::FromFile){
                index = coordFunctions->getIndex(coordTask->getName(), index);
                if(index >= 0){
                    coordRef = new CoordinateReference(coordTask->getName(),coordFunctions->get(index));
                }
            }
            else if((coordTask->getValueType() == IKCoordinateTask::ManualValue)){
                Constant reference(Constant(coordTask->getValue()));
                coordRef = new CoordinateReference(coordTask->getName(), reference);
            }
            else{ // assume it should be held at its current/default value
                double value = aModel->getCoordinateSet().get(coordTask->getName()).getValue(s);
                Constant reference = Constant(value);
                coordRef = new CoordinateReference(coordTask->getName(), reference);
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
    cout << "Frame at (t=" << s.getTime() << "):\t";
    cout << "total squared error = " << totalSquaredMarkerError;
    cout << ", marker error: RMS=" << sqrt(totalSquaredMarkerError/nm);
    cout << ", max=" << sqrt(maxSquaredMarkerError) << " (" << ikSol.getMarkerNameForIndex(worst) << ")" << endl;
    /* Now move the non-fixed markers on the model so that they are coincident
     * with the measured markers in the static pose. The model is already in
     * the proper configuration so the coordinates do not need to be changed.
     */
    if(_moveModelMarkers) moveModelMarkersToPose(s, *aModel, staticPose);

    if (_outputStorage!= NULL){
        delete _outputStorage;
    }
    // Make a storage file containing the solved states and markers for display in GUI.
    Storage motionData;
    StatesReporter statesReporter(aModel);
    statesReporter.begin(s);
    
    _outputStorage = new Storage(statesReporter.updStatesStorage());
    _outputStorage->setName("static pose");
    //_outputStorage->print("statesReporterOutput.sto");
    Storage markerStorage;

    //********************* Fill up storage with marker data - Begin
    auto& rStorage = *_outputStorage;
    // Clear existing frames.
    rStorage.reset(0);
    // Make Column labels.
    Array<std::string> columnLabels{};
    columnLabels.append("time");
    for(size_t i = 0; i < staticPose.getNumColumns(); ++i) {
        columnLabels.append(labels[i].getValue<std::string>() + "_tx");
        columnLabels.append(labels[i].getValue<std::string>() + "_ty");
        columnLabels.append(labels[i].getValue<std::string>() + "_tz");
    }
    rStorage.setColumnLabels(columnLabels);

    // Store the marker coordinates in an array of doubles and add it to the
    // Storage.
    for(size_t i = 0; i < staticPose.getNumRows(); ++i) {
        const auto& frame = staticPose.getRowAtIndex(i);
        std::vector<double> row{};
        for(size_t j = 0; j < staticPose.getNumColumns(); ++j) {
            const auto& marker = frame[j];
            row.push_back(marker[0]);
            row.push_back(marker[1]);
            row.push_back(marker[2]);
        }
        rStorage.append(staticPose.getIndependentColumn()[i],
                        row.size(),
                        row.data());
    }
    //********************* Fill up storage with marker data - End

    _outputStorage->getStateVector(0)->setTime(s.getTime());
    statesReporter.updStatesStorage().addToRdStorage(*_outputStorage, s.getTime(), s.getTime());
    //_outputStorage->print("statesReporterOutputWithMarkers.sto");

    if(_printResultFiles) {
        if (!_outputModelFileNameProp.getValueIsDefault())
        {
            aModel->print(aPathToSubject + _outputModelFileName);
            cout << "Wrote model file " << _outputModelFileName << " from model " << aModel->getName() << endl;
        }

        if (!_outputMarkerFileNameProp.getValueIsDefault())
        {
            aModel->writeMarkerFile(aPathToSubject + _outputMarkerFileName);
            cout << "Wrote marker file " << _outputMarkerFileName << " from model " << aModel->getName() << endl;
        }
        
        if (!_outputMotionFileNameProp.getValueIsDefault())
        {
            _outputStorage->print(aPathToSubject + _outputMotionFileName, 
                "w", "File generated from solving marker data for model "+aModel->getName());
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
void MarkerPlacer::moveModelMarkersToPose(SimTK::State& s, 
                                          Model& aModel, 
                                          MarkerTable& aPose)
{
    averageFrames(aPose, 0.01);

    const auto& frame = aPose.getRowAtIndex(0);

    // const SimbodyEngine& engine = aModel.getSimbodyEngine();

    MarkerSet& markerSet = aModel.updMarkerSet();

    int i;
    for (i = 0; i < markerSet.getSize(); i++)
    {
        Marker& modelMarker = markerSet.get(i);

        if (true/*!modelMarker.getFixed()*/)
        {
            auto& labels = aPose.
                           getDependentsMetaData().
                           getValueArrayForKey("labels");
            std::vector<std::string> markernames{};
            for(size_t i = 0; i < labels.size(); ++i) 
                markernames.push_back(labels[i].getValue<std::string>());
            auto iter = std::find(markernames.cbegin(),
                                  markernames.cend(),
                                  modelMarker.getName());
            if (iter != markernames.cend())
            {
                auto index = iter - markernames.cbegin();
                Vec3 globalMarker = frame[index];
                if (!globalMarker.isNaN())
                {
                    Vec3 pt, pt2;
                    Vec3 globalPt = globalMarker;
                    auto units = Units{aPose.
                                       getTableMetaData().
                                       getValueForKey("Units").
                                       getValue<std::string>()};
                    auto modelUnits = aModel.getLengthUnits();
                    double conversionFactor = units.convertTo(modelUnits);
                    pt = conversionFactor*globalPt;
                    pt2 = modelMarker.getReferenceFrame().findLocationInAnotherFrame(s, pt, aModel.getGround());
                    modelMarker.set_location(pt2);
                }
                else
                {
                    cout << "___WARNING___: marker " << modelMarker.getName() << " does not have valid coordinates." << endl;
                    cout << "               It will not be moved to match location in marker file." << endl;
                }
            }
        }
    }

    cout << "Moved markers in model " << aModel.getName() << " to match locations in marker file." << endl;
}

Storage *MarkerPlacer::getOutputStorage() 
{
    return _outputStorage; ;
}

void MarkerPlacer::averageFrames(MarkerTable& markerTable,
                                 double aThreshold,
                                 double aStartTime,
                                 double aEndTime) {
    const auto& timecol = markerTable.getIndependentColumn();

    if(aStartTime > aEndTime)
        throw Exception{"Start Time > End Time."};
    if(aStartTime > timecol.back())
        throw Exception{"Start Time lies after last frame."};
    if(aEndTime < timecol.front())
        throw Exception{"End Time lies before first frame."};

    const auto begind = std::lower_bound(timecol.cbegin(), 
                                         timecol.cend(), 
                                         aStartTime) - timecol.cbegin();
    const auto endind = std::upper_bound(timecol.cbegin(), 
                                         timecol.cend(), 
                                         aEndTime) - 1 - timecol.cbegin();

    if(begind > endind)
        throw Exception{"Zero frames between start and end time."};

    MarkerTable::RowVector minrow{static_cast<int>(markerTable.getNumColumns()),
                                  SimTK::Vec3{SimTK::Infinity}};
    MarkerTable::RowVector maxrow{static_cast<int>(markerTable.getNumColumns()),
                                  SimTK::Vec3{-SimTK::Infinity}};
    MarkerTable::RowVector avgrow{static_cast<int>(markerTable.getNumColumns()),
                                  SimTK::Vec3{SimTK::NaN}};
    double avgtime{};
    for(size_t i = begind; i < endind; ++i) {
        avgtime += markerTable.getIndependentColumn()[i];
        const auto row = markerTable.getRowAtIndex(i);
        for(size_t m = 0; m < markerTable.getNumColumns(); ++m) {
            const auto pt = row[m];
            if(!pt.isNaN()) {
                if(avgrow[m].isNaN())
                    avgrow[m] = pt;
                else
                    avgrow[m] += pt;
            }
            if(aThreshold > 0) {
                for(size_t j = 0; j < 3; ++j) {
                    minrow[m][j] = std::min(minrow[m][j], pt[j]);
                    maxrow[m][j] = std::max(maxrow[m][j], pt[j]);
                }
            }
        }
    }
    avgtime /= endind - begind;
    avgrow /= endind - begind;
    
    MarkerTable newMarkerTable{};
    newMarkerTable.updTableMetaData() = markerTable.getTableMetaData();
    newMarkerTable.setDependentsMetaData(markerTable.getDependentsMetaData());
    for(size_t i = 0; i < begind; ++i) {
        newMarkerTable.appendRow(markerTable.getIndependentColumn()[i],
                                 markerTable.getRowAtIndex(i));
    }
    newMarkerTable.appendRow(avgtime, avgrow);
    for(size_t i = endind + 1; i < markerTable.getNumRows(); ++i) {
        newMarkerTable.appendRow(markerTable.getIndependentColumn()[i],
                                 markerTable.getRowAtIndex(i));
    }

    auto& labels = newMarkerTable.
                   getDependentsMetaData().
                   getValueArrayForKey("labels");

    for(size_t m = 0; m < newMarkerTable.getNumColumns(); ++m) {
        if(avgrow[m].isNaN())
            std::cout << "__WARNING__: marker " 
                      << labels[m].getValue<std::string>() 
                      << " is missing in frames " << begind << " to " << endind 
                      << ". Coordinates will be set to NaN." << std::endl;
        else if(maxrow[m][0] - minrow[m][0] > aThreshold ||
                maxrow[m][1] - minrow[m][1] > aThreshold ||
                maxrow[m][2] - minrow[m][2] > aThreshold) {
            double maxdim = maxrow[m][0] - minrow[m][0];
            maxdim = std::max(maxdim, maxrow[m][1] - minrow[m][1]);
            maxdim = std::max(maxdim, maxrow[m][2] - minrow[m][2]);
            std::cout << "__WARNING__: movement of marker " 
                      << labels[m].getValue<std::string>() << " is " << maxdim 
                      << " (threshold = " << aThreshold << ")" 
                      << std::endl;
        }
    }

    std::cout << "Averaged frames from time " << _timeRange[0] << " to "
              << _timeRange[1] << " (frames " << begind << " to " << endind 
              << ")" << std::endl;

    markerTable = newMarkerTable;
}


void MarkerPlacer::convertUnits(const Model * const aModel,
                                MarkerTable& markerTable) {
    auto units = markerTable.
                 getTableMetaData().
                 getValueForKey("Units").
                 getValue<std::string>();
    double scaleFactor = Units{units}.convertTo(aModel->getLengthUnits());
    if(fabs(scaleFactor - 1.0) > SimTK::Eps) {
        if(!SimTK::isNaN(scaleFactor)) {
            for(size_t i = 0; i < markerTable.getNumRows(); ++i)
                markerTable.updRowAtIndex(i) *= scaleFactor;
        }
    }
}
