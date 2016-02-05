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
#include <OpenSim/Common/Adapters.h>

using namespace std;
using namespace SimTK;

namespace OpenSim {

//______________________________________________________________________________
/**
 * An implementation of the MarkersReference 
 *
 * @param model to assemble
 */
MarkersReference::MarkersReference() : Reference_<SimTK::Vec3>(), 
        _markersFile(_markersFileProp.getValueStr()),
        _markerWeightSetProp(PropertyObj("", Set<MarkerWeight>())),
        _markerWeightSet((Set<MarkerWeight>&)_markerWeightSetProp.getValueObj()),
        _defaultWeight(_defaultWeightProp.getValueDbl()),
        _markerData{}
{
    setAuthors("Ajay Seth");
}

MarkersReference::MarkersReference(const std::string markerFile, Units modelUnits) : Reference_<SimTK::Vec3>(), 
        _markersFile(_markersFileProp.getValueStr()),
        _markerWeightSetProp(PropertyObj("", Set<MarkerWeight>())),
        _markerWeightSet((Set<MarkerWeight>&)_markerWeightSetProp.getValueObj()),
        _defaultWeight(_defaultWeightProp.getValueDbl()),
        _markerData{}
{
    setAuthors("Ajay Seth");
    loadMarkersFile(markerFile, modelUnits);
}

    template<typename>
    class shrik;

/**
 * Convenience constructor to be used for Marker placement. 
 *
 * @param aMarkerData: MarkerData, assumed to be in the correct units already
 */
MarkersReference::MarkersReference(MarkerTable& aMarkerData, const Set<MarkerWeight>* aMarkerWeightSet) : Reference_<SimTK::Vec3>(), 
        _markersFile(_markersFileProp.getValueStr()),
        _markerWeightSetProp(PropertyObj("", Set<MarkerWeight>())),
        _markerWeightSet((Set<MarkerWeight>&)_markerWeightSetProp.getValueObj()),
        _defaultWeight(_defaultWeightProp.getValueDbl()),
        _markerData{}
{
    if (aMarkerWeightSet!=nullptr) _markerWeightSet= *aMarkerWeightSet;
    populateFromMarkerData(aMarkerData);
}

/** load the marker data for this MarkersReference from markerFile  */
void MarkersReference::loadMarkersFile(const std::string markerFile, Units modelUnits)
{
    _markersFile = markerFile;

    _markerData = *TRCFileAdapter{}.read(markerFile);

    // Convert the marker data into the model's units
    auto units = _markerData.
                 getTableMetaData().
                 getValueForKey("Units").
                 getValue<std::string>();

    double scaleFactor = Units{units}.convertTo(modelUnits);

    if(fabs(scaleFactor - 1.0) > SimTK::Eps &&
       !SimTK::isNaN(scaleFactor)) {
        for(size_t i = 0; i < _markerData.getNumRows(); ++i)
            _markerData.updRowAtIndex(i) *= scaleFactor;
    }
    populateFromMarkerData(_markerData);

}

int MarkersReference::getNumRefs() const {
    return _markerData.getNumColumns();
}

/** A convenience method yo populate MarkersReference from MarkerData **/
void MarkersReference::populateFromMarkerData(MarkerTable& aMarkerData)
{
    _markerData = aMarkerData;
    auto& labels = aMarkerData.
                   getDependentsMetaData().
                   getValueArrayForKey("labels");
    Array<std::string> tempNames{};
    for(size_t i = 0; i < labels.size(); ++i)
        tempNames.append(labels[i].getValue<std::string>());
    
    int nm = tempNames.getSize();

    // empty any lingering names and weights
    _markerNames.clear();
    _weights.clear();
    // pre-allocate arrays to the number of markers in the file with default weightings
    _markerNames.assign(nm, "");
    _weights.assign(nm, _defaultWeight);

    int index = 0;
    // Build flat lists (arrays) of marker names and weights in the same order as the marker data
    for(int i=0; i<tempNames.getSize(); i++){
        const std::string &name = tempNames[i];
        _markerNames[i] = name;
        index = _markerWeightSet.getIndex(name, index);
        //Assign user weighting for markers that are user listed in the input set
        if(index >= 0)
            _weights[i] = _markerWeightSet[index].getWeight();
    }

    if(_markerNames.size() != _weights.size())
        throw Exception("MarkersReference: Mismatch between the number of marker names and weights. Verify that marker names are unique.");
}

SimTK::Vec2 MarkersReference::getValidTimeRange() const
{
    const auto& indCol = _markerData.getIndependentColumn();
    return Vec2(indCol.front(), indCol.back());
}

// utility to define object properties including their tags, comments and 
// default values.
void MarkersReference::setupProperties()
{
    _markersFileProp.setComment("TRC file (.trc) containing the time history of observations of marker."
                                "positions.");
    _markersFileProp.setName("marker_file");
    _propertySet.append( &_markersFileProp );

    _markerWeightSetProp.setComment("Set of marker weights identified by marker name with weight being a positive scalar.");
    _markerWeightSetProp.setName("marker_weights");
    _propertySet.append( &_markerWeightSetProp );
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

    auto cbeg = _markerData.getIndependentColumn().cbegin();
    auto cend = _markerData.getIndependentColumn().cend();

    auto after = std::lower_bound(cbeg, cend, time);
    if(after == cend)
        throw Exception{"No index corresponding to time of frame."};

    auto ind = after - cbeg;
    if(*after != time && after != cbeg) {
        auto before = after - 1;
        if((time - *before) < (*after - time))
            --ind;
    }

    const auto row = _markerData.getRowAtIndex(ind);
    values.clear();
    for(size_t c = 0; c < _markerData.getNumColumns(); ++c)
        values.push_back(row[c]);
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

double MarkersReference::getSamplingFrequency() {
    return std::stod(_markerData.
                     getTableMetaData().
                     getValueForKey("DataRate").
                     getValue<std::string>());
}

void MarkersReference::setMarkerWeightSet(Set<MarkerWeight> &markerWeights)
{
    _markerWeightSet.setSize(0);
    for(int i=0; i<markerWeights.getSize(); i++)
        _markerWeightSet.adoptAndAppend(&markerWeights[i]);

    //Make sure the input set no longer owns the weightings
    _markerWeightSet.setMemoryOwner(false);
}


} // end of namespace OpenSim
