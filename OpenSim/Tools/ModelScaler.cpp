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

#include "OpenSim/Common/CommonUtilities.h"
#include "OpenSim/Common/IO.h"
#include "OpenSim/Common/MarkerData.h"
#include "OpenSim/Common/Scale.h"
#include "OpenSim/Common/ScaleSet.h"
#include "OpenSim/Common/XMLDocument.h"
#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Tools/MarkerPair.h"
#include "OpenSim/Tools/Measurement.h"
#include "OpenSim/Tools/MeasurementSet.h"

#include <memory>

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
ModelScaler::ModelScaler() { constructProperties(); }

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ModelScaler::constructProperties() {
    constructProperty_apply(true);
    constructProperty_scaling_order(Array<std::string>(""));
    constructProperty_MeasurementSet(MeasurementSet());
    constructProperty_ScaleSet(ScaleSet());
    constructProperty_marker_file("");
    constructProperty_time_range(Array<double>{-1.0, -1.0});
    constructProperty_preserve_mass_distribution(true);
    constructProperty_output_model_file("");
    constructProperty_output_scale_file("");
    constructProperty_print_result_files(true);
}

//_____________________________________________________________________________
/**
 * Register the types used by this class.
 */
void ModelScaler::registerTypes()
{
    Object::registerType(Measurement());
    Object::registerType(Scale());
    Measurement::registerTypes();
}

void ModelScaler::addMeasurement(Measurement* aMeasurement) {
    upd_MeasurementSet().adoptAndAppend(aMeasurement);
}
void ModelScaler::addScale(Scale* aScale) {
    upd_ScaleSet().adoptAndAppend(aScale);
}

void ModelScaler::setScaleSetFile(const std::string& aScaleSetFilename) {
    updProperty_ScaleSet() = ScaleSet(aScaleSetFilename);
}
void ModelScaler::setMeasurementSet(MeasurementSet& measurementSet) {
    upd_MeasurementSet() = measurementSet;
}
Array<std::string> ModelScaler::getScalingOrder() const {
    Array<std::string> array;
    for (int i = 0; i < getProperty_scaling_order().size(); ++i) {
        array.append(get_scaling_order(i));
    }
    return array;
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
        double aSubjectMass) const {
    if (!getApply()) return false;

    int i;
    ScaleSet theScaleSet;
    Vec3 unity(1.0);

    log_info("Step 2: Scaling generic model");

    /* Make a scale set with a Scale for each physical frame.
     * Initialize all factors to 1.0.
     */
    for (const auto& segment : aModel->getComponentList<PhysicalFrame>()) {
        auto segmentScale = Scale();
        segmentScale.setSegmentName(segment.getName());
        segmentScale.setScaleFactors(unity);
        segmentScale.setApply(true);
        theScaleSet.cloneAndAppend(segmentScale);
    }

    SimTK::State& s = aModel->initSystem();
    aModel->getMultibodySystem().realize(s, SimTK::Stage::Position);

    try
    {
        /* Make adjustments to theScaleSet, in the user-specified order. */
        for (i = 0; i < getProperty_scaling_order().size(); i++) {
            /* For measurements, measure the distance between a pair of markers
             * in the model, and in the static pose. The latter divided by the
             * former is the scale factor. Put that scale factor in theScaleSet,
             * using the body/axis names specified in the measurement to
             * determine in what place[s] to put the factor.
             */
            if (get_scaling_order(i) == "measurements") {
                /* Load the static pose marker file, and convert units.
                */
                std::unique_ptr<MarkerData> markerData;
                if (!get_marker_file().empty() &&
                        !getProperty_marker_file().getValueIsDefault()) {
                    markerData = std::make_unique<MarkerData>(
                            aPathToSubject + get_marker_file());
                    markerData->convertToUnits(aModel->getLengthUnits());
                }

                /* Now take and apply the measurements. */
                for (int j = 0; j < get_MeasurementSet().getSize(); j++) {
                    auto& measurement = get_MeasurementSet()[j];
                    if (measurement.getApply()) {
                        if(!markerData)
                            throw Exception(
                                    "ModelScaler.processModel: ERROR- " +
                                            getProperty_marker_file()
                                                    .getName() +
                                            " not set but measurements are "
                                            "used",
                                    __FILE__, __LINE__);
                        double scaleFactor = computeMeasurementScaleFactor(
                                s, *aModel, *markerData, measurement);
                        if (!SimTK::isNaN(scaleFactor))
                            measurement.applyScaleFactor(
                                    scaleFactor, theScaleSet);
                        else
                            log_warn("'{}' measurement not used to scale {}",
                                    measurement.getName(), aModel->getName());
                    }
                }
            }
            /* For manual scales, just copy the XYZ scale factors from
             * the manual scale into theScaleSet.
             */
            else if (get_scaling_order(i) == "manualScale") {
                for (int j = 0; j < get_ScaleSet().getSize(); j++) {
                    const auto& scale = get_ScaleSet()[j];
                    if (scale.getApply()) {
                        const string& bodyName = scale.getSegmentName();
                        const auto& factors = scale.get_scales();
                        for (int k = 0; k < theScaleSet.getSize(); k++) {
                            if (theScaleSet[k].getSegmentName() == bodyName)
                                theScaleSet[k].setScaleFactors(factors);
                        }
                    }
                }
            } else {
                throw Exception("ModelScaler: ERR- Unrecognized string '" +
                                        get_scaling_order(i) + "' in " +
                                        getProperty_scaling_order().getName() +
                                        " property (expecting 'measurements' "
                                        "or 'manualScale').",
                        __FILE__, __LINE__);
            }
        }

        /* Now scale the model. */
        aModel->scale(
                s, theScaleSet, get_preserve_mass_distribution(), aSubjectMass);

        if (get_print_result_files()) {
            auto cwd = IO::CwdChanger::changeTo(aPathToSubject);

            if (isValidFileName(getProperty_output_model_file())) {
                if (aModel->print(get_output_model_file()))
                    log_info("Wrote model file '{}' from model.",
                            get_output_model_file(), aModel->getName());
            }

            if (isValidFileName(getProperty_output_scale_file())) {
                if (theScaleSet.print(get_output_scale_file()))
                    log_info("Wrote scale file '{}' for model {}.",
                            get_output_scale_file(), aModel->getName());
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
        if (getProperty_time_range().size() < 2)
            throw Exception("ModelScaler::takeExperimentalMarkerMeasurement, time_range is unspecified.");

        aMarkerData.findFrameRange(
                get_time_range(0), get_time_range(1), startIndex, endIndex);
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
