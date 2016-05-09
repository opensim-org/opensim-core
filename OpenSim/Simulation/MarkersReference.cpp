/* -------------------------------------------------------------------------- *
 *                       OpenSim:  MarkersReference.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

/** load the marker data for this MarkersReference from markerFile  */
void MarkersReference::loadMarkersFile(const std::string markerFile, Units modelUnits)
{
    upd_marker_file() = markerFile;
    _markerData.reset(new MarkerData(get_marker_file()));

    // Convert the marker data into the model's units
    _markerData->convertToUnits(modelUnits);

    populateFromMarkerData(*_markerData);
}


/** A convenience method yo populate MarkersReference from MarkerData **/
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

    int index = 0;
    // Build flat lists (arrays) of marker names and weights in the same order as the marker data
    for(int i=0; i<tempNames.getSize(); i++){
        const std::string &name = tempNames[i];
        _markerNames[i] = name;
        index = get_marker_weights().getIndex(name, index);
        //Assign user weighting for markers that are user listed in the input set
        if(index >= 0)
            _weights[i] = get_marker_weights()[index].getWeight();
    }

    if(_markerNames.size() != _weights.size())
        throw Exception("MarkersReference: Mismatch between the number of marker names and weights. Verify that marker names are unique.");
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
    weights = _weights;
}

void MarkersReference::setMarkerWeightSet(const Set<MarkerWeight>& markerWeights)
{
    upd_marker_weights() = markerWeights;
}

} // end of namespace OpenSim