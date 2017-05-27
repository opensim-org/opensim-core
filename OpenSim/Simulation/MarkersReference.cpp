/* -------------------------------------------------------------------------- *
 *                       OpenSim:  MarkersReference.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <SimTKcommon/internal/State.h>
#include <cmath>

using namespace std;
using namespace SimTK;

namespace OpenSim {

MarkersReference::MarkersReference() :
    Reference_<SimTK::Vec3>() {
    constructProperties();
    setAuthors("Ajay Seth");
}

MarkersReference::MarkersReference(const std::string& markerFile,
                                   Units modelUnits) :
    MarkersReference() {
    loadMarkersFile(markerFile, modelUnits);
}

MarkersReference::
MarkersReference(const TimeSeriesTable_<SimTK::Vec3>& markerTable,
                 const Set<MarkerWeight>* markerWeightSet,
                 Units units) :
    MarkersReference() {
    _markerTable = markerTable;
    if(markerWeightSet != nullptr)
        upd_marker_weights() = *markerWeightSet;
    populateFromMarkerData(markerTable, units.getAbbreviation());
}

void MarkersReference::loadMarkersFile(const std::string markerFile,
                                       Units modelUnits) {
    auto fileExt = FileAdapter::findExtension(markerFile);
    OPENSIM_THROW_IF(!(fileExt == "sto" || fileExt == "trc"),
                     UnsupportedFileType,
                     markerFile,
                     "Supported file types are -- STO, TRC.");

    if(fileExt == "trc") {
        _markerTable = TimeSeriesTableVec3{markerFile};
    } else {
        try {
            _markerTable = (TimeSeriesTable{markerFile}).pack<SimTK::Vec3>();
        } catch(const IncorrectTableType&) {
            _markerTable = TimeSeriesTable_<SimTK::Vec3>{markerFile};
        }
    }

    upd_marker_file() = markerFile;

    populateFromMarkerData(_markerTable, modelUnits.getAbbreviation());
}

void
MarkersReference::
populateFromMarkerData(const TimeSeriesTable_<SimTK::Vec3>& markerTable,
                       const std::string& units) {
    Units thisUnits{};
    if(_markerTable.hasTableMetaDataKey("Units"))
        thisUnits = Units{_markerTable.getTableMetaData<std::string>("Units")};
    else
        thisUnits = Units{Units::Meters};

    double scaleFactor = thisUnits.convertTo(Units{units});

    OPENSIM_THROW_IF(SimTK::isNaN(scaleFactor),
                     Exception,
                     "TimeSeriesTable has unspecified units.");

    if(std::fabs(scaleFactor - 1) >= SimTK::Eps) {
        for(unsigned r = 0; r < _markerTable.getNumRows(); ++r)
            _markerTable.updRowAtIndex(r) *= scaleFactor;

        _markerTable.removeTableMetaDataKey("Units");
        _markerTable.addTableMetaData("Units", units);
    }
    
    const auto& markerNames = markerTable.getColumnLabels();
    _markerNames.clear();
    _weights.clear();
    _markerNames.assign(static_cast<unsigned>(markerNames.size()), "");
    _weights.assign(static_cast<unsigned>(markerNames.size()),
                    get_default_weight());

    for(unsigned i = 0; i < markerNames.size(); ++i) {
        _markerNames[i] = markerNames[i];
    }

    // Names must be assigned before weights can be updated
    updateInternalWeights();
}

SimTK::Vec2 MarkersReference::getValidTimeRange() const {
    OPENSIM_THROW_IF(_markerTable.getNumRows() == 0,
                     Exception,
                     "Marker-table is empty.");

    return {_markerTable.getIndependentColumn().front(),
            _markerTable.getIndependentColumn().back()};
}

void MarkersReference::constructProperties() {
    constructProperty_marker_file("");
    Set<MarkerWeight> markerWeights;
    constructProperty_marker_weights(markerWeights);
    constructProperty_default_weight(1.0);
}

const SimTK::Array_<std::string>& MarkersReference::getNames() const {
    return _markerNames;
}

void  MarkersReference::getValues(const SimTK::State& s,
                                  SimTK::Array_<Vec3>& values) const {
    double time = s.getTime();
    const auto rowView = _markerTable.getNearestRow(time);
    values.clear();
    for(int i = 0; i < rowView.ncol(); ++i)
        values.push_back(rowView[i]);
}

// void
// MarkersReference::getSpeedValues(const SimTK::State &s,
//                                  SimTK::Array_<Vec3> &speedValues) const {
//     throw Exception("MarkersReference: getSpeedValues not implemented.");
// }
// 
// void
// MarkersReference::getAccelerationValues(const SimTK::State &s,
//                                         SimTK::Array_<Vec3> &accValues) const {
//     throw Exception("MarkersReference: getAccelerationValues not implemented.");
// }

void MarkersReference::getWeights(const SimTK::State &s,
                                  SimTK::Array_<double> &weights) const {
    updateInternalWeights();
    weights = _weights;
}

void
MarkersReference::setMarkerWeightSet(const Set<MarkerWeight>& markerWeights) {
    upd_marker_weights() = markerWeights;
}

void
MarkersReference::setDefaultWeight(double weight) {
    set_default_weight(weight);
}

void
MarkersReference::updateInternalWeights() const {
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
            _weights[ix] = get_marker_weights()[wix].getWeight();
        ++ix;
    }
}

int
MarkersReference::getNumRefs() const {
    return static_cast<int>(_markerTable.getNumColumns());
}

double
MarkersReference::getSamplingFrequency() const {
    if(_markerTable.hasTableMetaDataKey("DataRate")) {
        auto datarate = _markerTable.getTableMetaData<std::string>("DataRate");
        return std::stod(datarate);
    } else
        return SimTK::NaN;
}

size_t
MarkersReference::getNumFrames() const {
    return _markerTable.getNumRows();
}

} // end of namespace OpenSim
