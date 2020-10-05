#ifndef OPENSIM_MARKERS_REFERENCE_H_
#define OPENSIM_MARKERS_REFERENCE_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  MarkersReference.h                        *
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

#include "Reference.h"
#include <OpenSim/Common/Set.h>
#include "OpenSim/Common/Units.h"
#include "OpenSim/Common/TimeSeriesTable.h"

namespace OpenSim {

class UnsupportedFileType : public Exception {
public:
    UnsupportedFileType(const std::string& file,
                        size_t line,
                        const std::string& func,
                        const std::string& filename,
                        const std::string& msg) :
        Exception{file, line, func} {
        std::string message = msg + "\nFilename : '" + filename + "'";

        addMessage(message);
    }
};

class Units;

class OSIMSIMULATION_API MarkerWeight : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MarkerWeight, Object);
private:
    OpenSim_DECLARE_PROPERTY(weight, double, "Marker weight.");

public:
    MarkerWeight() : Object() { constructProperties(); }

    MarkerWeight(std::string name, double weight) : MarkerWeight() {
        setName(name);
        upd_weight() = weight;
    }

    void setWeight(double weight) { upd_weight() = weight; }
    double getWeight() const {return get_weight(); }

private:
    void constructProperties() {
        constructProperty_weight(1.0);
    }

}; // end of MarkerWeight class


//=============================================================================
//=============================================================================
/**
 * Reference values to be achieved for specified Markers that will be used
 * via optimization and/or tracking. Also contains a weighting that identifies
 * the relative importance of achieving one marker's reference relative to
 * another.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API MarkersReference
        : public Reference_<SimTK::Vec3> {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MarkersReference, Reference_<SimTK::Vec3>);
    //=============================================================================
// Properties
//=============================================================================
public:
    OpenSim_DECLARE_PROPERTY(marker_file, std::string, 
        "Marker file(e.g. .trc) containing the time history of observations of "
        "marker positions.");

    OpenSim_DECLARE_PROPERTY(marker_weights, Set<MarkerWeight>,
        "Set of marker weights identified by marker name with weight being a "
        "positive scalar.");

    OpenSim_DECLARE_PROPERTY(default_weight, double,
        "Default weight for a marker.");
//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    MarkersReference();

    /** Convenience load markers from a file. See below. */
    MarkersReference(const std::string& markerFileName,
                     const Set<MarkerWeight>& markerWeightSet,
                     Units modelUnits = Units(Units::Meters));
    /** Form a Reference from TimeSeriesTable and corresponding marker weights. 
    The marker weights are used to initialize the weightings of the markers
    provided by the Reference. Marker weights are associated to markers by
    name. If a markerWeightSet is provided, then only those markers listed in
    the set are tracked, otherwise all the markerData table that correspond
    with model markers are tracked with the default weighting.
    The TimeSeriesTable should contain the key 'Units', representing 
    units of the columns, as table metadata. In absence of 'Units' metadata,
    columns are assumed to be of units 'meters'.                              */
    MarkersReference(const TimeSeriesTable_<SimTK::Vec3>& markerData,
        const Set<MarkerWeight>& markerWeightSet,
                     Units units = Units(Units::Meters));

    virtual ~MarkersReference() {}

    /** Initialize this MarkersReference from data in a markerFile such that it
        corresponds to the markers that have weights. If weights is empty Set,
        all corresponding markers are tracked at default reference weight.
        See setDefaultWeight()*/
    void initializeFromMarkersFile(const std::string& markerFile,
                                   const Set<MarkerWeight>& markerWeightSet,
                                   Units modelUnits = Units(Units::Meters));

    //--------------------------------------------------------------------------
    // Reference Interface
    //--------------------------------------------------------------------------
    int getNumRefs() const override;
    /** get the time range for which the MarkersReference values are valid,
        based on the loaded marker data.*/
    SimTK::Vec2 getValidTimeRange() const override;
    /** get the names of the markers serving as references */
    const SimTK::Array_<std::string>& getNames() const override;
    /** get the value of the MarkersReference  */
    void getValuesAtTime(
            double time, SimTK::Array_<SimTK::Vec3> &values) const override;
    // The following two methods are commented out as they are not implemented
    // and we don't want users to think it *is* implemented when viewing
    // doxygen.
    // /** get the speed value of the MarkersReference */
    // virtual void getSpeedValues(const SimTK::State &s,
    //     SimTK::Array_<SimTK::Vec3> &speedValues) const;
    // /** get the acceleration value of the MarkersReference */
    // virtual void getAccelerationValues(const SimTK::State &s,
    //     SimTK::Array_<SimTK::Vec3> &accValues) const;
    /** get the weighting (importance) of meeting this MarkersReference in the
        same order as names*/
    void getWeights(const SimTK::State &s,
                    SimTK::Array_<double> &weights) const override;
    /** get the marker trajectories in a table*/
    const TimeSeriesTable_<SimTK::Vec3>& getMarkerTable() const;

    //--------------------------------------------------------------------------
    // Convenience Access
    //--------------------------------------------------------------------------
    double getSamplingFrequency() const;
    const Set<MarkerWeight>& getMarkerWeightSet() const
        {   return get_marker_weights(); }
    Set<MarkerWeight>& updMarkerWeightSet() {return upd_marker_weights(); }
    /** %Set the marker weights from a set of MarkerWeights. As of OpenSim 4.0
        the input set is const and a copy of the Set is used internally. 
        Therefore, subsequent changes to the Set of MarkerWeights will have
        no effect on the marker weights associated with this Reference. */
    void setMarkerWeightSet(const Set<MarkerWeight>& markerWeights);
    void setDefaultWeight(double weight);
    size_t getNumFrames() const;

private:
    void constructProperties();
    void
    populateFromMarkerData(const TimeSeriesTable_<SimTK::Vec3>& markerData,
                           const Set<MarkerWeight>& markerWeightSet,
                           const std::string& units = "Meters");
    void updateInternalWeights() const;

    TimeSeriesTable_<SimTK::Vec3> _markerTable;
    // marker names inside the marker data
    SimTK::Array_<std::string> _markerNames;
    // List of weights guaranteed to be in the same order as marker names.
    mutable SimTK::Array_<double> _weights;
//=============================================================================
};  // END of class MarkersReference
//=============================================================================
} // namespace

#endif // OPENSIM_MARKERS_REFERENCE_H_
