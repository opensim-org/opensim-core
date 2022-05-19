#ifndef OPENSIM_EXPONENTIAL_SPRING_FORCE_H_
#define OPENSIM_EXPONENTIAL_SPRING_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  HuntCrossleyForce.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Eastman                                                   *
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
// INCLUDE
#include "Force.h"
#include "OpenSim/Common/Set.h"

namespace OpenSim {

//==============================================================================
//                       EXPONENTIAL SPRING FORCE
//==============================================================================
/** This force subclass implements an ExponentialSpringForce to model contact
of a specified point on a body (i.e., a "station" in Simbody vocabulary) with
a contact plane that is anchored to Ground.

@author F. C. Anderson **/
class OSIMSIMULATION_API ExponentialSpringForce : public Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(ExponentialSpringForce, Force);

public:
    //==============================================================================
    // PROPERTIES
    //==============================================================================
    OpenSim_DECLARE_PROPERTY(contact_plane_transform, SimTK::Transform,
            "Rotation and translation of the contact plane wrt Ground.");
    OpenSim_DECLARE_PROPERTY(
            body, PhysicalFrame, "Body to which the force is applied.");
    OpenSim_DECLARE_PROPERTY(body_station, SimTK::Vec3,
            "Point on the body at which the force is applied.");

    //______________________________________________________________________________
    /** Construct an ExponentialSpringForce.
    @param XContactPlane Transform specifying the location and orientation of
    the contact plane in Ground.
    @param body Body to which the force is applied.
    @param station Point on the body at which the force is applied. */
    explicit ExponentialSpringForce(const SimTK::Transform& XContactPlane,
            const PhysicalFrame& body, const SimTK::Vec3& station);

    //-----------------------------------------------------------------------------
    // Accessors
    //-----------------------------------------------------------------------------
    /** Get the tranform that specifies the location and orientation of the
    contact plane in the Ground frame. */
    const SimTK::Transform& getContactPlaneTransform() const;
    /** Set the tranform that specifies the location and orientation of the
    contact plane in the Ground frame. */
    void setContactPlaneTransform(const SimTK::Transform& XContactPlane);

    //-----------------------------------------------------------------------------
    // Reporting
    //-----------------------------------------------------------------------------
    /** Provide name(s) of the quantities (column labels) of the value(s) to be
    reported. */
    OpenSim::Array<std::string> getRecordLabels() const override;
    /** Provide the value(s) to be reported that correspond to the labels. */
    OpenSim::Array<double> getRecordValues(
            const SimTK::State& state) const override;

protected:
    /** Create a SimTK::Force which implements this Force. */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

private:
    void constructProperties(const SimTK::Transform& XContactPlane,
            const PhysicalFrame& body, const SimTK::Vec3& station);

}; // END of class ExponentialSpringForce

} // end of namespace OpenSim

#endif // OPENSIM_EXPONENTIAL_SPRING_FORCE_H_


