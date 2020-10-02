/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ModelScaler.cpp                          *
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
#include "ModelScaler.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/MarkerData.h>
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
ModelScaler::ModelScaler() :
    _apply(_applyProp.getValueBool()),
    _scalingOrder(_scalingOrderProp.getValueStrArray()),
    _measurementSetProp(PropertyObj("", MeasurementSet())),
    _measurementSet((MeasurementSet&)_measurementSetProp.getValueObj()),
    _scaleSetProp(PropertyObj("", ScaleSet())),
    _scaleSet((ScaleSet&)_scaleSetProp.getValueObj()),
    _markerFileName(_markerFileNameProp.getValueStr()),
    _timeRange(_timeRangeProp.getValueDblArray()),
    _preserveMassDist(_preserveMassDistProp.getValueBool()),
    _outputModelFileName(_outputModelFileNameProp.getValueStr()),
    _outputScaleFileName(_outputScaleFileNameProp.getValueStr())
{
    setNull();
    setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
ModelScaler::~ModelScaler()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aModelScaler ModelScaler to be copied.
 */
ModelScaler::ModelScaler(const ModelScaler &aModelScaler) :
   Object(aModelScaler),
    _apply(_applyProp.getValueBool()),
    _scalingOrder(_scalingOrderProp.getValueStrArray()),
    _measurementSetProp(PropertyObj("", MeasurementSet())),
    _measurementSet((MeasurementSet&)_measurementSetProp.getValueObj()),
    _scaleSetProp(PropertyObj("", ScaleSet())),
    _scaleSet((ScaleSet&)_scaleSetProp.getValueObj()),
   _markerFileName(_markerFileNameProp.getValueStr()),
    _timeRange(_timeRangeProp.getValueDblArray()),
    _preserveMassDist(_preserveMassDistProp.getValueBool()),
    _outputModelFileName(_outputModelFileNameProp.getValueStr()),
    _outputScaleFileName(_outputScaleFileNameProp.getValueStr())
{
    setNull();
    setupProperties();
    copyData(aModelScaler);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one ModelScaler to another.
 *
 * @param aModelScaler ModelScaler to be copied.
 */
void ModelScaler::copyData(const ModelScaler &aModelScaler)
{
    _apply = aModelScaler._apply;
    _scalingOrder = aModelScaler._scalingOrder;
    _measurementSet = aModelScaler._measurementSet;
    _scaleSet = aModelScaler._scaleSet;
    _markerFileName = aModelScaler._markerFileName;
    _timeRange = aModelScaler._timeRange;
    _preserveMassDist = aModelScaler._preserveMassDist;
    _outputModelFileName = aModelScaler._outputModelFileName;
    _outputScaleFileName = aModelScaler._outputScaleFileName;
    _printResultFiles = aModelScaler._printResultFiles;
}

//_____________________________________________________________________________
/**
 * Set the data members of this ModelScaler to their null values.
 */
void ModelScaler::setNull()
{
    _apply = true;

    _printResultFiles = true;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ModelScaler::setupProperties()
{
    _applyProp.setComment("Whether or not to use the model scaler during scale");
    _applyProp.setName("apply");
    _propertySet.append(&_applyProp);

    _scalingOrderProp.setComment("Specifies the scaling method and order. "
        "Valid options are 'measurements', 'manualScale', singly or both in any sequence.");
    _scalingOrderProp.setName("scaling_order");
    Array<string> sorder("");
    _scalingOrderProp.setValue(sorder);
    _propertySet.append(&_scalingOrderProp);

    _measurementSetProp.setComment("Specifies the measurements by which body segments are to be scaled.");
    _measurementSetProp.setName("MeasurementSet");
    _propertySet.append(&_measurementSetProp);

    _scaleSetProp.setComment("Scale factors to be used for manual scaling.");
    _scaleSetProp.setName("ScaleSet");
    _propertySet.append(&_scaleSetProp);

    _markerFileNameProp.setComment("TRC file (.trc) containing the marker positions used for measurement-based scaling. "
        "This is usually a static trial, but doesn't need to be.  The marker-pair distances are computed for each "
        "time step in the TRC file and averaged across the time range.");
    _markerFileNameProp.setName("marker_file");
    _propertySet.append(&_markerFileNameProp);

    _timeRangeProp.setComment("Time range over which to average marker-pair distances in the marker file (.trc) for "
        "measurement-based scaling.");
    const double defaultTimeRange[] = {-1.0, -1.0};
    _timeRangeProp.setName("time_range");
    _timeRangeProp.setValue(2, defaultTimeRange);
    _timeRangeProp.setAllowableListSize(2);
    _propertySet.append(&_timeRangeProp);

    _preserveMassDistProp.setComment("Flag (true or false) indicating whether or not to preserve relative mass between segments.");
    _preserveMassDistProp.setName("preserve_mass_distribution");
    _propertySet.append(&_preserveMassDistProp);

    _outputModelFileNameProp.setComment("Name of OpenSim model file (.osim) to write when done scaling.");
    _outputModelFileNameProp.setName("output_model_file");
    _propertySet.append(&_outputModelFileNameProp);

    _outputScaleFileNameProp.setComment("Name of file to write containing the scale factors that were applied to the unscaled model (optional).");
    _outputScaleFileNameProp.setName("output_scale_file");
    _propertySet.append(&_outputScaleFileNameProp);
}

//_____________________________________________________________________________
/**
 * Register the types used by this class.
 */
void ModelScaler::registerTypes()
{
    Object::registerType(Measurement());
    //Object::registerType(Scale());
    Measurement::registerTypes();
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
ModelScaler& ModelScaler::operator=(const ModelScaler &aModelScaler)
{
    // BASE CLASS
    Object::operator=(aModelScaler);

    copyData(aModelScaler);

    return(*this);
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * This method scales a model based on user-specified XYZ body scale factors
 * and/or a set of marker-to-marker distance measurements.
 *
 * @param aModel the model to scale.
 * @param aSubjectMass the final mass of the model after scaling.
 * @return Whether the scaling process was successful or not.
 */
bool ModelScaler::processModel(Model* aModel, const string& aPathToSubject,
        double aSubjectMass) const
{
    if (!getApply()) return false;

    int i;
    ScaleSet theScaleSet;
    Vec3 unity(1.0);

    log_info("Step 2: Scaling generic model");

    /* Make a scale set with a Scale for each physical frame.
     * Initialize all factors to 1.0.
     */
    for (const auto& segment : aModel->getComponentList<PhysicalFrame>()) {
        Scale* segmentScale = new Scale();
        segmentScale->setSegmentName(segment.getName());
        segmentScale->setScaleFactors(unity);
        segmentScale->setApply(true);
        theScaleSet.adoptAndAppend(segmentScale);
    }

    SimTK::State& s = aModel->initSystem();
    aModel->getMultibodySystem().realize(s, SimTK::Stage::Position);

    try
    {
        /* Make adjustments to theScaleSet, in the user-specified order. */
        for (i = 0; i < _scalingOrder.getSize(); i++)
        {
            /* For measurements, measure the distance between a pair of markers
             * in the model, and in the static pose. The latter divided by the
             * former is the scale factor. Put that scale factor in theScaleSet,
             * using the body/axis names specified in the measurement to
             * determine in what place[s] to put the factor.
             */
            if (_scalingOrder[i] == "measurements")
            {
                /* Load the static pose marker file, and convert units.
                */
                std::unique_ptr<MarkerData> markerData{};
                if(!_markerFileName.empty() && _markerFileName!=PropertyStr::getDefaultStr()) {
                    markerData.reset(new MarkerData(aPathToSubject + _markerFileName));
                    markerData->convertToUnits(aModel->getLengthUnits());
                }

                /* Now take and apply the measurements. */
                for (int j = 0; j < _measurementSet.getSize(); j++)
                {
                    if (_measurementSet.get(j).getApply())
                    {
                        if(!markerData)
                            throw Exception("ModelScaler.processModel: ERROR- "+_markerFileNameProp.getName()+
                                                " not set but measurements are used",__FILE__,__LINE__);
                        double scaleFactor = computeMeasurementScaleFactor(s,*aModel, *markerData, _measurementSet.get(j));
                        if (!SimTK::isNaN(scaleFactor))
                            _measurementSet.get(j).applyScaleFactor(scaleFactor, theScaleSet);
                        else
                            log_warn("'{}' measurement not used to scale {}", 
                                _measurementSet.get(j).getName(),
                                aModel->getName());
                    }
                }
            }
            /* For manual scales, just copy the XYZ scale factors from
             * the manual scale into theScaleSet.
             */
            else if (_scalingOrder[i] == "manualScale")
            {
                for (int j = 0; j < _scaleSet.getSize(); j++)
                {
                    if (_scaleSet[j].getApply())
                    {
                        const string& bodyName = _scaleSet[j].getSegmentName();
                        Vec3 factors(1.0);
                        _scaleSet[j].getScaleFactors(factors);
                        for (int k = 0; k < theScaleSet.getSize(); k++)
                        {
                            if (theScaleSet[k].getSegmentName() == bodyName)
                                theScaleSet[k].setScaleFactors(factors);
                        }
                    }
                }
            }
            else
            {
                throw Exception("ModelScaler: ERR- Unrecognized string '"+_scalingOrder[i]+"' in "+_scalingOrderProp.getName()+" property (expecting 'measurements' or 'manualScale').",__FILE__,__LINE__);
            }
        }

        /* Now scale the model. */
        aModel->scale(s, theScaleSet, _preserveMassDist, aSubjectMass);

        if(_printResultFiles) {
            auto cwd = IO::CwdChanger::changeTo(aPathToSubject);

            if (_outputModelFileNameProp.isValidFileName()) {
                if (aModel->print(_outputModelFileName))
                    log_info("Wrote model file '{}' from model.",
                        _outputModelFileName, aModel->getName());
            }

            if (_outputScaleFileNameProp.isValidFileName()) {
                if (theScaleSet.print(_outputScaleFileName))
                    log_info("Wrote scale file '{}' for model {}.",
                        _outputScaleFileName, aModel->getName());
            }
        }
    }
    catch (const Exception& x) {
        log_error(x.what());
        return false;
    }

    return true;
}

//_____________________________________________________________________________
/**
 * For measurement based scaling, we average the scale factors across the different marker pairs used.
 * For each marker pair, the scale factor is computed by dividing the average distance between the pair 
 * in the experimental marker data by the distance between the pair on the model.
 */
double ModelScaler::computeMeasurementScaleFactor(const SimTK::State& s, const Model& aModel, const MarkerData& aMarkerData, const Measurement& aMeasurement) const
{
    double scaleFactor = 0;
    log_info("Measurement '{}'", aMeasurement.getName());
    if(aMeasurement.getNumMarkerPairs()==0) return SimTK::NaN;
    for(int i=0; i<aMeasurement.getNumMarkerPairs(); i++) {
        const MarkerPair& pair = aMeasurement.getMarkerPair(i);
        string name1, name2;
        pair.getMarkerNames(name1, name2);
        double modelLength = takeModelMeasurement(s, aModel, name1, name2, aMeasurement.getName());
        double experimentalLength = takeExperimentalMarkerMeasurement(aMarkerData, name1, name2, aMeasurement.getName());
        if(SimTK::isNaN(modelLength) || SimTK::isNaN(experimentalLength)) return SimTK::NaN;
        log_info("\tpair {} ({}, {}): model = {}, experimental = {}",
            i, name1, name2, modelLength, experimentalLength);
        scaleFactor += experimentalLength / modelLength;
    }
    scaleFactor /= aMeasurement.getNumMarkerPairs();
    log_info("\toverall scale factor = {}", scaleFactor);
    return scaleFactor;
}

//_____________________________________________________________________________
/**
 * Measure the distance between two model markers.
 *
 * @return The measured distance.
 */
double ModelScaler::takeModelMeasurement(const SimTK::State& s, const Model& aModel, const string& aName1, const string& aName2, const string& aMeasurementName) const
{
    for (const auto& aName : {aName1, aName2}) {
        if (!aModel.getMarkerSet().contains(aName)) {
            log_warn("Marker {} in {} measurement not found in {}.", aName, 
                aMeasurementName, aModel.getName());
            return SimTK::NaN;
        }
    }
    const Marker& marker1 = aModel.getMarkerSet().get(aName1);
    const Marker& marker2 = aModel.getMarkerSet().get(aName2);
    Vec3 difference = marker1.get_location() - marker2.findLocationInFrame(s, marker1.getParentFrame());
    return difference.norm();
}

//_____________________________________________________________________________
/**
 * Measure the average distance between a marker pair in an experimental marker data.
 */
double ModelScaler::takeExperimentalMarkerMeasurement(const MarkerData& aMarkerData, const string& aName1, const string& aName2, const string& aMeasurementName) const
{
    const Array<string>& experimentalMarkerNames = aMarkerData.getMarkerNames();
    int marker1 = experimentalMarkerNames.findIndex(aName1);
    int marker2 = experimentalMarkerNames.findIndex(aName2);
    if (marker1 >= 0 && marker2 >= 0) {
        int startIndex, endIndex;
        if (_timeRange.getSize()<2) 
            throw Exception("ModelScaler::takeExperimentalMarkerMeasurement, time_range is unspecified.");

        aMarkerData.findFrameRange(_timeRange[0], _timeRange[1], startIndex, endIndex);
        double length = 0;
        for(int i=startIndex; i<=endIndex; i++) {
            Vec3 p1 = aMarkerData.getFrame(i).getMarker(marker1);
            Vec3 p2 = aMarkerData.getFrame(i).getMarker(marker2);
            length += (p2 - p1).norm();
        }
        return length/(endIndex-startIndex+1);
    } else {
        if (marker1 < 0)
            log_warn("Marker {} in {} measurement not found in {}.", aName1, 
                    aMeasurementName, aMarkerData.getFileName());
        if (marker2 < 0)
            log_warn("Marker {} in {} measurement not found in {}.", aName2,
                    aMeasurementName, aMarkerData.getFileName());
        return SimTK::NaN;
    }
}
