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

#include "Reference.h"
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/MarkerData.h>

namespace OpenSim {

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
class OSIMSIMULATION_API MarkersReference : public Reference_<SimTK::Vec3> {
    OpenSim_DECLARE_CONCRETE_OBJECT(MarkersReference, Reference_<SimTK::Vec3>);
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

    /** Convenience load markers from a file */
    MarkersReference(const std::string& markerFileName,
                     Units modelUnits=Units(Units::Meters));
    /** Form a Reference from MarkerData and corresponding marker weights. Note
        The MarkersReference take ownership of the pointer to the MarkerData. 
        The marker weights are used to initialize the weightings of the markers
        provided by the Reference. Marker weights are associated to markers by
        name.*/
    MarkersReference(MarkerData* markerData,
        const Set<MarkerWeight>* markerWeightSet=nullptr);

    virtual ~MarkersReference() {}

    /** load the marker data for this MarkersReference from markerFile  */
    void loadMarkersFile(const std::string markerFile,
        Units modelUnits=Units(Units::Meters));

    //--------------------------------------------------------------------------
    // Reference Interface
    //--------------------------------------------------------------------------
    int getNumRefs() const override {return _markerData->getNumMarkers(); }
    /** get the time range for which the MarkersReference values are valid,
        based on the loaded marker data.*/
    SimTK::Vec2 getValidTimeRange() const override;
    /** get the names of the markers serving as references */
    const SimTK::Array_<std::string>& getNames() const override;
    /** get the value of the MarkersReference */
    void getValues(const SimTK::State &s,
        SimTK::Array_<SimTK::Vec3> &values) const override;
    /** get the speed value of the MarkersReference */
    virtual void getSpeedValues(const SimTK::State &s,
        SimTK::Array_<SimTK::Vec3> &speedValues) const;
    /** get the acceleration value of the MarkersReference */
    virtual void getAccelerationValues(const SimTK::State &s,
        SimTK::Array_<SimTK::Vec3> &accValues) const;
    /** get the weighting (importance) of meeting this MarkersReference in the
        same order as names*/
    void getWeights(const SimTK::State &s,
                    SimTK::Array_<double> &weights) const override;

    //--------------------------------------------------------------------------
    // Convenience Access
    //--------------------------------------------------------------------------
    double getSamplingFrequency() {return _markerData->getDataRate(); }
    Set<MarkerWeight> &updMarkerWeightSet() {return upd_marker_weights(); }
    /** %Set the marker weights from a set of MarkerWeights. As of OpenSim 4.0
        the input set is const and a copy of the Set is used internally. 
        Therefore, subsequent changes to the Set of MarkerWeights will have
        no effect on the marker weights associated with this Reference. */
    void setMarkerWeightSet(const Set<MarkerWeight>& markerWeights);
    void setDefaultWeight(double weight) { set_default_weight(weight); }

private:
    void constructProperties();
    void populateFromMarkerData(const MarkerData& markerData);

private:
    // Use a specialized data structure for holding the marker data
    SimTK::ResetOnCopy< std::unique_ptr<MarkerData> > _markerData;
    // marker names inside the marker data
    SimTK::Array_<std::string> _markerNames;
    // corresponding list of weights guaranteed to be in the same order as names above
    SimTK::Array_<double> _weights;

//=============================================================================
};  // END of class MarkersReference
//=============================================================================
} // namespace

#endif // OPENSIM_MARKERS_REFERENCE_H_
