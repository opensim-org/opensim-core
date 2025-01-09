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
#include <SimTKcommon/internal/State.h>

using namespace std;
using namespace SimTK;

namespace OpenSim {

OrientationsReference::OrientationsReference()
        : StreamableReference_<SimTK::Rotation>() {
    constructProperties();
    setAuthors("Ajay Seth");
}

OrientationsReference::OrientationsReference(const std::string& orientationFile,
    Units modelUnits) : OrientationsReference()
{
    loadOrientationsEulerAnglesFile(orientationFile, modelUnits);
}


OrientationsReference::OrientationsReference(
    const TimeSeriesTable_<Rotation>& orientationData,
    const Set<OrientationWeight>* orientationWeightSet) 
        : OrientationsReference()
{
    _orientationData = orientationData;
    if (orientationWeightSet!=nullptr)
        upd_orientation_weights()= *orientationWeightSet;
    populateFromOrientationData();
}

void OrientationsReference::loadOrientationsEulerAnglesFile(
                                    const std::string orientationFile,
                                    Units modelUnits)
{
    upd_orientation_file() = orientationFile;

    auto xyzEulerData = TimeSeriesTable_<Vec3>(orientationFile);

    _orientationData.updTableMetaData() = xyzEulerData.getTableMetaData();
    _orientationData.setDependentsMetaData(xyzEulerData.getDependentsMetaData());

    const auto& times = xyzEulerData.getIndependentColumn();

    size_t nt = xyzEulerData.getNumRows();
    int nc = int(xyzEulerData.getNumColumns());

    RowVector_<Rotation> row(nc);

    for (size_t i = 0; i < nt; ++i) {
        const auto& xyzRow = xyzEulerData.getRowAtIndex(i);
        for (int j = 0; j < nc; ++j) {
            const Vec3& xyzO = xyzRow[j];
            row[j] = Rotation(BodyOrSpaceType::BodyRotationSequence,
                xyzO[0], XAxis, xyzO[1], YAxis, xyzO[2], ZAxis);
        }
        _orientationData.appendRow(times[i], row);
    }

    populateFromOrientationData();
}

void OrientationsReference::populateFromOrientationData()
{
    const std::vector<std::string>& tempNames = 
        _orientationData.getColumnLabels();
    unsigned int no = unsigned(tempNames.size());

    // empty any lingering names and weights
    _orientationNames.clear();
    _weights.clear();
    // pre-allocate arrays to the number of Orientations in the file with default weightings
    _orientationNames.assign(no, "");
    _weights.assign(no, get_default_weight());

    int index = 0;
    // Build flat lists (arrays) of orientation names and weights in the same order as the orientation data
    for(unsigned int i=0; i<no; ++i){
        const std::string &name = tempNames[i];
        _orientationNames[i] = name;
        index = get_orientation_weights().getIndex(name, index);
        //Assign user weighting for Orientations that are user listed in the input set
        if(index >= 0)
            _weights[i] = get_orientation_weights()[index].getWeight();
    }

    if(_orientationNames.size() != _weights.size())
        throw Exception("OrientationsReference: Mismatch between the number "
            "of orientation names and weights. Verify that orientation names "
            "are unique.");
}

int OrientationsReference::getNumRefs() const
{
    return int(_orientationData.getNumColumns());
}

double OrientationsReference::getSamplingFrequency() const
{
    return std::atoi(_orientationData.getTableMetaData().getValueForKey("DataRate")
        .getValue<std::string>().c_str());
}

SimTK::Vec2 OrientationsReference::getValidTimeRange() const
{
    auto& times = _orientationData.getIndependentColumn();
    return Vec2(*times.begin(), *(--times.end()));
}

const std::vector<double>& OrientationsReference::getTimes() const
{
    return _orientationData.getIndependentColumn();
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
void OrientationsReference::getValuesAtTime(
        double time, SimTK::Array_<Rotation> &values) const
{

    // get values for time
    SimTK::RowVector_<Rotation> row = _orientationData.getRow(time);

    int n = row.size();
    values.resize(n);

    for (int i = 0; i < n; ++i) {
        values[i] = row[i];
    }
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