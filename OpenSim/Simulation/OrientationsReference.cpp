/* -------------------------------------------------------------------------- *
 *                   OpenSim:  OrientationsReference.cpp                      *
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

#include "OrientationsReference.h"
#include <OpenSim/Common/Units.h>
#include <OpenSim/Common/TRCFileAdapter.h>

using namespace std;
using namespace SimTK;

namespace OpenSim {

OrientationsReference::OrientationsReference() : Reference_<SimTK::Vec3>()
{
    constructProperties();
    setAuthors("Ajay Seth");
}

OrientationsReference::OrientationsReference(const std::string& orientationFile,
    Units modelUnits) : OrientationsReference()
{
    loadOrientationsFile(orientationFile, modelUnits);
}


OrientationsReference::OrientationsReference(TimeSeriesTableVec3* orientationData,
    const Set<OrientationWeight>* orientationWeightSet) : OrientationsReference()
{
    _orientationData = *orientationData;
    if (orientationWeightSet!=nullptr) upd_orientation_weights()= *orientationWeightSet;
    populateFromOrientationData(*orientationData);
}

/** load the orientation data for this OrientationsReference from orientationFile  */
void OrientationsReference::loadOrientationsFile(const std::string orientationFile, 
                                                 Units modelUnits)
{
    upd_orientation_file() = orientationFile;

    _orientationData = TRCFileAdapter::read(get_orientation_file());

    // Convert the orientation data into the model's units
    //_orientationData.convertToUnits(modelUnits);

    populateFromOrientationData(_orientationData);
}


/** A convenience method yo populate OrientationsReference from OrientationData **/
void OrientationsReference::populateFromOrientationData(
    const TimeSeriesTableVec3& orientationData)
{
    const std::vector<std::string> &tempNames = orientationData.getColumnLabels();
    size_t no = tempNames.size();

    // empty any lingering names and weights
    _orientationNames.clear();
    _weights.clear();
    // pre-allocate arrays to the number of Orientations in the file with default weightings
    _orientationNames.assign(no, "");
    _weights.assign(no, get_default_weight());

    int index = 0;
    // Build flat lists (arrays) of orientation names and weights in the same order as the orientation data
    for(size_t i=0; i<no; ++i){
        const std::string &name = tempNames[i];
        _orientationNames[i] = name;
        index = get_orientation_weights().getIndex(name, index);
        //Assign user weighting for Orientations that are user listed in the input set
        if(index >= 0)
            _weights[i] = get_orientation_weights()[index].getWeight();
    }

    if(_orientationNames.size() != _weights.size())
        throw Exception("OrientationsReference: Mismatch between the number of orientation names and weights. Verify that orientation names are unique.");
}

int OrientationsReference::getNumRefs() const
{
    return _orientationData.getNumColumns();
}

double OrientationsReference::getSamplingFrequency() const
{
    return _orientationData.getTableMetaData().getValueForKey("DataRate")
        .getValue<double>();
}

SimTK::Vec2 OrientationsReference::getValidTimeRange() const
{
    auto& times = _orientationData.getIndependentColumn();
    return Vec2(times[0], times[1]);
}

// utility to define object properties including their tags, comments and 
// default values.
void OrientationsReference::constructProperties()
{
    constructProperty_orientation_file("");
    Set<OrientationWeight> orientationWeights;
    constructProperty_orientation_weights(orientationWeights);
    constructProperty_default_weight(1.0);
}

/** get the names of the Orientations serving as references */
const SimTK::Array_<std::string>& OrientationsReference::getNames() const
{
    return _orientationNames;
}

/** get the values of the OrientationsReference */
void  OrientationsReference::getValues(const SimTK::State &s, SimTK::Array_<Vec3> &values) const
{
    double time =  s.getTime();

    // get values for time
    SimTK::RowVector_<Vec3> row = _orientationData.getRow(time);

    int n = row.size();
    values.resize(n);

    for (int i = 0; i < n; ++i) {
        values[i] = row[i];
    }
}

/** get the speed value of the OrientationsReference */
void OrientationsReference::getSpeedValues(const SimTK::State &s, SimTK::Array_<Vec3> &speedValues) const
{
    throw Exception("OrientationsReference: getSpeedValues not implemented.");
}

/** get the acceleration value of the OrientationsReference */
void OrientationsReference::getAccelerationValues(const SimTK::State &s, SimTK::Array_<Vec3> &accValues) const
{
    throw Exception("OrientationsReference: getAccelerationValues not implemented.");
}

/** get the weights of the Orientations */
void  OrientationsReference::getWeights(const SimTK::State &s, SimTK::Array_<double> &weights) const
{
    weights = _weights;
}

void OrientationsReference::setOrientationWeightSet(const Set<OrientationWeight>& orientationWeights)
{
    upd_orientation_weights() = orientationWeights;
}

} // end of namespace OpenSim