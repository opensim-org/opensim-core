/* -------------------------------------------------------------------------- *
 *                       OpenSim:  MarkersReference.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

#include "MarkersReference.h"
#include <OpenSim/Common/Units.h>
#include <SimTKcommon/internal/State.h>

using namespace std;
using namespace SimTK;

namespace OpenSim {

MarkersReference::MarkersReference() : Reference_<SimTK::Vec3>()
{
    constructProperties();
    setAuthors("Ajay Seth");
}

MarkersReference::MarkersReference(const std::string& markerFile,
    Units modelUnits) : MarkersReference()
{
    loadMarkersFile(markerFile, modelUnits);
}


MarkersReference::MarkersReference(MarkerData *markerData,
    const Set<MarkerWeight>* markerWeightSet) : MarkersReference()
{
    _markerData.reset(markerData);
    if (markerWeightSet!=nullptr) upd_marker_weights()= *markerWeightSet;
    populateFromMarkerData(*markerData);
}

/* load the marker data for this MarkersReference from markerFile  */
void MarkersReference::loadMarkersFile(const std::string markerFile, Units modelUnits)
{
    upd_marker_file() = markerFile;
    _markerData.reset(new MarkerData(get_marker_file()));

    // Convert the marker data into the model's units
    _markerData->convertToUnits(modelUnits);

    populateFromMarkerData(*_markerData);
}


/* A convenience method to populate MarkersReference from MarkerData **/
void MarkersReference::populateFromMarkerData(const MarkerData& aMarkerData)
{
    const Array<std::string> &tempNames = aMarkerData.getMarkerNames();
    int nm = tempNames.getSize();

    // empty any lingering names and weights
    _markerNames.clear();
    _weights.clear();
    // pre-allocate arrays to the number of markers in the file with default weightings
    _markerNames.assign(nm, "");
    _weights.assign(nm, get_default_weight());

    for (int i = 0; i < nm; i++) {
        const std::string &name = tempNames[i];
        _markerNames[i] = name;
    }

    // Names must be assigned before weights can be updated
    updateInternalWeights();
}

SimTK::Vec2 MarkersReference::getValidTimeRange() const
{
    return Vec2(_markerData->getStartFrameTime(), _markerData->getLastFrameTime());
}

// utility to define object properties including their tags, comments and 
// default values.
void MarkersReference::constructProperties()
{
    constructProperty_marker_file("");
    Set<MarkerWeight> markerWeights;
    constructProperty_marker_weights(markerWeights);
    constructProperty_default_weight(1.0);
}

/** get the names of the markers serving as references */
const SimTK::Array_<std::string>& MarkersReference::getNames() const
{
    return _markerNames;
}

/** get the values of the MarkersReference */
void  MarkersReference::getValues(const SimTK::State &s, SimTK::Array_<Vec3> &values) const
{
    double time =  s.getTime();

    int before=0, after=0;
    // get index for time 
    _markerData->findFrameRange(time, time, before, after);
    if(before > after || after < 0)
        throw Exception("MarkersReference: No index corresponding to time of frame.");
    else if(after-before > 0){
        before = abs(_markerData->getFrame(before).getFrameTime()-time) < abs(_markerData->getFrame(after).getFrameTime()-time) ? before : after;
    }

    values = _markerData->getFrame(before).getMarkers();
}

/** get the speed value of the MarkersReference */
void MarkersReference::getSpeedValues(const SimTK::State &s, SimTK::Array_<Vec3> &speedValues) const
{
    throw Exception("MarkersReference: getSpeedValues not implemented.");
}

/** get the acceleration value of the MarkersReference */
void MarkersReference::getAccelerationValues(const SimTK::State &s, SimTK::Array_<Vec3> &accValues) const
{
    throw Exception("MarkersReference: getAccelerationValues not implemented.");
}

/** get the weights of the Markers */
void  MarkersReference::getWeights(const SimTK::State &s, SimTK::Array_<double> &weights) const
{
    updateInternalWeights();
    weights = _weights;
}

void MarkersReference::setMarkerWeightSet(const Set<MarkerWeight>& markerWeights)
{
    upd_marker_weights() = markerWeights;
}

void MarkersReference::setDefaultWeight(double weight)
{
    set_default_weight(weight); 
}

void MarkersReference::updateInternalWeights() const
{
    // if weights are not being changed, do not rebuild list of weights.
    if (isObjectUpToDateWithProperties())
        return;

    // Begin by assigning default weight to each. Markers that do not have a
    // weight specified in the marker_weights property use the default weight.
    _weights.assign(getNumRefs(), get_default_weight());

    // Next fill in the marker weights that were specified in the 
    // marker_weights property 
    int wix = -1;
    int ix = 0;
    // Build flat lists of marker weights in the same order as the marker names
    for (const std::string &name : _markerNames) {
        wix = get_marker_weights().getIndex(name, wix);
        // Associate user weights (as specified in the marker_weights property)
        // with the corresponding marker by order of marker names
        if (wix >= 0)
            _weights[ix++] = get_marker_weights()[wix].getWeight();
    }
}

} // end of namespace OpenSim