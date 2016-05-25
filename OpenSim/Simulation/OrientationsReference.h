#ifndef OPENSIM_ORIENTATIONS_REFERENCE_H_
#define OPENSIM_ORIENTATIONS_REFERENCE_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  OrientationsReference.h                      *
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

#include "Reference.h"
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Common/Units.h>

namespace OpenSim {

class OSIMSIMULATION_API OrientationWeight : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(OrientationWeight, Object);
private:
    OpenSim_DECLARE_PROPERTY(weight, double, "Orientation reference weight.");

public:
    OrientationWeight() : Object() { constructProperties(); }

    OrientationWeight(std::string name, double weight) : OrientationWeight() {
        setName(name);
        upd_weight() = weight;
    }

    void setWeight(double weight) { upd_weight() = weight; }
    double getWeight() const {return get_weight(); }

private:
    void constructProperties() {
        constructProperty_weight(1.0);
    }

}; // end of OrientationWeight class


//=============================================================================
//=============================================================================
/**
 * Reference values to be achieved for specified Orientations that will be used
 * via optimization and/or tracking. Also contains a weighting that identifies
 * the relative importance of achieving one orientation's reference relative to
 * another.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API OrientationsReference : public Reference_<SimTK::Vec3> {
    OpenSim_DECLARE_CONCRETE_OBJECT(OrientationsReference, Reference_<SimTK::Vec3>);
//=============================================================================
// Properties
//=============================================================================
public:
    OpenSim_DECLARE_PROPERTY(orientation_file, std::string, 
        "Orientation file (.sto, .csv) containing the time history of observations "
        "of segment (sensor) orientations.");

    OpenSim_DECLARE_PROPERTY(orientation_weights, Set<OrientationWeight>,
        "Set of orientation weights identified by orientation name with weight "
        " being a positive scalar.");

    OpenSim_DECLARE_PROPERTY(default_weight, double,
        "Default weight for an orientation.");
//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    OrientationsReference();

    /** Convenience load Orientations from a file */
    OrientationsReference(const std::string& orientationFileName,
                     Units modelUnits=Units(Units::Radians));
    /** Form a Reference from TimeSeriesData of Euler angles (Vec3) and corres-
    ponding orientation weights. Note, the OrientationsReference takes owner-
    ship of the pointer to the orientationData. The orientation weights are used
    to initialize the weightings of the Orientations provided by the Reference.
    Orientation weights are associated to Orientations by name.*/
    OrientationsReference(TimeSeriesTableVec3* orientationData,
        const Set<OrientationWeight>* orientationWeightSet=nullptr);

    virtual ~OrientationsReference() {}

    /** load the orientation data for this OrientationsReference from orientationFile  */
    void loadOrientationsFile(const std::string orientationFile,
        Units modelUnits=Units(Units::Meters));

    //--------------------------------------------------------------------------
    // Reference Interface
    //--------------------------------------------------------------------------
    int getNumRefs() const override;
    /** get the time range for which the OrientationsReference values are valid,
        based on the loaded orientation data.*/
    SimTK::Vec2 getValidTimeRange() const override;
    /** get the names of the Orientations serving as references */
    const SimTK::Array_<std::string>& getNames() const override;
    /** get the value of the OrientationsReference */
    void getValues(const SimTK::State &s,
        SimTK::Array_<SimTK::Vec3> &values) const override;
    /** get the speed value of the OrientationsReference */
    virtual void getSpeedValues(const SimTK::State &s,
        SimTK::Array_<SimTK::Vec3> &speedValues) const;
    /** get the acceleration value of the OrientationsReference */
    virtual void getAccelerationValues(const SimTK::State &s,
        SimTK::Array_<SimTK::Vec3> &accValues) const;
    /** get the weighting (importance) of meeting this OrientationsReference in the
        same order as names*/
    void getWeights(const SimTK::State &s,
                    SimTK::Array_<double> &weights) const override;

    //--------------------------------------------------------------------------
    // Convenience Access
    //--------------------------------------------------------------------------
    double getSamplingFrequency() const;
    Set<OrientationWeight> &updOrientationWeightSet() {return upd_orientation_weights(); }
    /** %Set the orientation weights from a set of OrientationWeights, which is
    const and a copy of the Set is used internally. Therefore, subsequent changes
    to the Set of OrientationWeights will have no effect on the orientation weights
    associated with this Reference. You can, however, change the weigtings on the
    InverseKinematicsSolver prior to solving at any instant in time. */
    void setOrientationWeightSet(const Set<OrientationWeight>& orientationWeights);
    void setDefaultWeight(double weight) { set_default_weight(weight); }

private:
    void constructProperties();
    void populateFromOrientationData(const TimeSeriesTableVec3& orientationData);

private:
    // Use a specialized data structure for holding the orientation data
    TimeSeriesTableVec3 _orientationData;
    // orientation names inside the orientation data
    SimTK::Array_<std::string> _orientationNames;
    // corresponding list of weights guaranteed to be in the same order as names above
    SimTK::Array_<double> _weights;

//=============================================================================
};  // END of class OrientationsReference
//=============================================================================
} // namespace

#endif // OPENSIM_ORIENTATIONS_REFERENCE_H_
