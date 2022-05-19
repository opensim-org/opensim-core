/* -------------------------------------------------------------------------- *
 *                      OpenSim:  ExponentialSpringForce.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s):  F. C. Anderson                                                  *
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

#include "ExponentialSpringForce.h"
#include "Model.h"

#include "simbody/internal/ExponentialSpringForce.h"

namespace OpenSim {

//=============================================================================
// Class ExponentialSpringForce
//=============================================================================

//-----------------------------------------------------------------------------
// Construction
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
ExponentialSpringForce::
ExponentialSpringForce(const SimTK::Transform& XContactPlane,
    const PhysicalFrame& body, const SimTK::Vec3& station)
{
    constructProperties(XContactPlane, body, station);
}


//-----------------------------------------------------------------------------
// Properties
//-----------------------------------------------------------------------------
// Questions:
// 1. Is it ok not to have a default constructor? There really isn't a
// situation in which a default constructor makes sense. There needs to
// be a body to which the force is applied. I suppose the default constructor
// could apply a force between Ground and Ground.
//_____________________________________________________________________________
void
ExponentialSpringForce::
constructProperties(const SimTK::Transform& XContactPlane,
    const PhysicalFrame& body, const SimTK::Vec3& station)
{
    constructProperty_contact_plane_transform(XContactPlane);
    constructProperty_body(body);
    constructProperty_body_station(station);
}


//-----------------------------------------------------------------------------
// ADD TO THE MULTIBODY SYSTEM
//-----------------------------------------------------------------------------
// Questions:
// 1. Does "spr" not need to be allocated from the heap?
// 2. What happens to the force subsystem when spr goes out of scope?
// 
// Guesses:
// 1. The GeneralForceSubsystem makes a clone of spr in a way that
// keeps references to it still valid.
// 
//_____________________________________________________________________________
// Connect to SimTK::ExponentialSpringForce
void
ExponentialSpringForce::
extendAddToSystem(SimTK::MultibodySystem& system) const {
    // Extend the OpenSim::Force parent
    Super::extendAddToSystem(system);

    // Construct the SimTK::ExponentialSpringForce object
    SimTK::GeneralForceSubsystem& forces = _model->updForceSubsystem();
    const SimTK::Transform& XContactPlane = get_contact_plane_transform();
    const OpenSim::PhysicalFrame& body = get_body();
    const SimTK::Vec3& station = get_body_station();
    SimTK::ExponentialSpringForce
        spr(forces, XContactPlane, body.getMobilizedBody(), station);

    // Get the subsystem index so we can access the SimTK::Force later.
    ExponentialSpringForce* mutableThis =
        const_cast<ExponentialSpringForce *>(this);
    mutableThis->_index = spr.getForceIndex();
}


//-----------------------------------------------------------------------------
// ACCESSORS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
const SimTK::Transform&
ExponentialSpringForce::getContactPlaneTransform() const {
    return get_contact_plane_transform();
}
//_____________________________________________________________________________
void ExponentialSpringForce::setContactPlaneTransform(
        const SimTK::Transform& XContactPlane) {
    set_contact_plane_transform(XContactPlane);
}


//-----------------------------------------------------------------------------
// Reporting
//-----------------------------------------------------------------------------
// Questions:
// 1. Do I need to conform to a certain reporting format since this object
// will be treated as an OpenSim::Force ?
//_____________________________________________________________________________
OpenSim::Array<std::string>
ExponentialSpringForce::
getRecordLabels() const {
    OpenSim::Array<std::string> labels("");
    std::string frameName = get_body().getName();
    labels.append(getName()+"."+frameName+".force.X");
    labels.append(getName()+"."+frameName+".force.Y");
    labels.append(getName()+"."+frameName+".force.Z");

    return labels;
}
//_____________________________________________________________________________
OpenSim::Array<double>
ExponentialSpringForce::
getRecordValues(const SimTK::State& state) const 
{
    OpenSim::Array<double> values(1);

    const auto& forceSubsys = getModel().getForceSubsystem();
    const SimTK::Force& abstractForce = forceSubsys.getForce(_index);
    const auto& spr = (SimTK::ExponentialSpringForce&)(abstractForce);

    SimTK::Vec3 force = spr.getForce(state);
    values.append(3, &force[0]);

    return values;
}

}// end of namespace OpenSim
