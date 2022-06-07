/* -------------------------------------------------------------------------- *
 *                      OpenSim:  ExponentialSpringForce.cpp                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s):  F. C. Anderson                                                 *
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

#include "Model.h"
#include "ExponentialSpringForce.h"

using namespace OpenSim;
using namespace std;


//=============================================================================
// ExponentialContact::Parameters
//=============================================================================
//_____________________________________________________________________________
ExponentialContact::Parameters::
Parameters()
{
    setNull();
    constructProperties();
}
//_____________________________________________________________________________
ExponentialContact::Parameters::
Parameters(const SimTK::ExponentialSpringParameters& params)
{
    setNull();
    _stkparams = params;
    constructProperties();
}
//_____________________________________________________________________________
void ExponentialContact::Parameters::setNull()
{
    setAuthors("F. C. Anderson");
}
//_____________________________________________________________________________
void
ExponentialContact::Parameters::
constructProperties()
{
    SimTK::Vec3 shape;
    _stkparams.getShapeParameters(shape[0], shape[1], shape[2]);
    constructProperty_exponential_shape_parameters(shape);
    constructProperty_normal_viscosity(_stkparams.getNormalViscosity());
    constructProperty_max_normal_force(_stkparams.getMaxNormalForce());
    constructProperty_friction_elasticity(_stkparams.getFrictionElasticity());
    constructProperty_friction_viscocity(_stkparams.getFrictionViscosity());
    constructProperty_sliding_time_constant(_stkparams.getSlidingTimeConstant());
    constructProperty_settle_velocity(_stkparams.getSettleVelocity());
    constructProperty_settle_acceleration(_stkparams.getSettleAcceleration());
    constructProperty_initial_mu_static(_stkparams.getInitialMuStatic());
    constructProperty_initial_mu_kinetic(_stkparams.getInitialMuKinetic());
}
//_____________________________________________________________________________
// Update the Properties based on the SimTK::ExponentialSpringParameters.
void
ExponentialContact::Parameters::
updateProperties()
{
    SimTK::Vec3 shape;
    _stkparams.getShapeParameters(shape[0], shape[1], shape[2]);
    set_exponential_shape_parameters(shape);
    set_normal_viscosity(_stkparams.getNormalViscosity());
    set_max_normal_force(_stkparams.getMaxNormalForce());
    set_friction_elasticity(_stkparams.getFrictionElasticity());
    set_friction_viscocity(_stkparams.getFrictionViscosity());
    set_sliding_time_constant(_stkparams.getSlidingTimeConstant());
    set_settle_velocity(_stkparams.getSettleVelocity());
    set_settle_acceleration(_stkparams.getSettleAcceleration());
    set_initial_mu_static(_stkparams.getInitialMuStatic());
    set_initial_mu_kinetic(_stkparams.getInitialMuKinetic());
}
//_____________________________________________________________________________
// Update the SimTK::ExponentialSpringParamters based on the Properties.
void
ExponentialContact::Parameters::
updateParameters()
{
    const SimTK::Vec3 shape = get_exponential_shape_parameters();
    _stkparams.setShapeParameters(shape[0], shape[1], shape[2]);
    _stkparams.setNormalViscosity(get_normal_viscosity());
    _stkparams.setMaxNormalForce(get_max_normal_force());
    _stkparams.setFrictionElasticity(get_friction_elasticity());
    _stkparams.setFrictionViscosity(get_friction_viscocity());
    _stkparams.setSlidingTimeConstant(get_sliding_time_constant());
    _stkparams.setSettleVelocity(get_settle_velocity());
    _stkparams.setSettleAcceleration(get_settle_acceleration());
    _stkparams.setInitialMuStatic(get_initial_mu_static());
    _stkparams.setInitialMuKinetic(get_initial_mu_kinetic());
}
//_____________________________________________________________________________
// This method is needed to ensure that the SimTK::ExponentialSpringParameters
// member variable (_stkparam) is kept consistent with the properties.
// It is necessary to have the _stkparams member variable because there needs
// to be a place to store non-default parameters when the underlying
// SimTK::ExponentialSpringForce hasn't been instantiated.
// Having to do a little extra bookkeeping (i.e., storing properties values
// twice [once in the Properties and once in _stkparams]) is justified
// by not having to rewrite a whole bunch of additional accessor methods.
// All parameter set/gets go through the SimTK::ExponentialSpringParameters
// interface (i.e., through _stkparams).
void
ExponentialContact::Parameters::
updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber)
{
    Super::updateFromXMLNode(node, versionNumber);
    updateParameters(); // catching any invalid property values in the process
    updateProperties(); // pushes valid parameters back to the properties.
}
//_____________________________________________________________________________
// Note that the OpenSim Properties are updated as well.
void
ExponentialContact::Parameters::
setSimTKParameters(const SimTK::ExponentialSpringParameters& params)
{
    _stkparams = params;
    updateProperties();
}
//_____________________________________________________________________________
// Get a read-only reference to the  SimTK::ExponentialSpringParamters held
// by this instance.
const SimTK::ExponentialSpringParameters&
ExponentialContact::Parameters::
getSimTKParameters() const
{
    return _stkparams;
}


//=============================================================================
// ExponentialContact
//=============================================================================
//_____________________________________________________________________________
ExponentialContact::ExponentialContact()
{
    setNull();
    constructProperties();
}
//_____________________________________________________________________________
ExponentialContact::
ExponentialContact(const SimTK::Transform& contactPlaneXform,
    const std::string& bodyName, const SimTK::Vec3& station,
    SimTK::ExponentialSpringParameters params)
{
    setNull();
    constructProperties();
    setContactPlaneTransform(contactPlaneXform);
    setBodyName(bodyName);
    setBodyStation(station);
    setParameters(params);
}
//_____________________________________________________________________________
/*
ExponentialContact::
ExponentialContact(const ExponentialContact& source)
{
    setNull();
    constructProperties();
    setContactPlaneTransform(source.getContactPlaneTransform());
    setBodyName(source.getBodyName());
    setBodyStation(source.getBodyStation());
}
*/
//_____________________________________________________________________________
void
ExponentialContact::
setNull()
{
    setAuthors("F. C. Anderson");
    _spr = NULL;
}
//_____________________________________________________________________________
void
ExponentialContact::
constructProperties()
{
    SimTK::Transform contactXForm;
    SimTK::Vec3 origin(0.0);
    Parameters params;
    constructProperty_contact_plane_transform(contactXForm);
    constructProperty_body_name("");
    constructProperty_body_station(origin);
    constructProperty_contact_parameters(params);
}
//_____________________________________________________________________________
void
ExponentialContact
::updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber) {
    Super::updateFromXMLNode(node, versionNumber);
}
//_____________________________________________________________________________
void
ExponentialContact::
extendConnectToModel(OpenSim::Model& model)
{
    // Allow based class to connect first
    Super::extendConnectToModel(model);

    // Find the OpenSim::Body
    const string& bodyName = getBodyName();
    if (getModel().hasComponent(bodyName))
        _body = &(getModel().getComponent<PhysicalFrame>(bodyName));
    else
        _body = &(getModel().getComponent<PhysicalFrame>(
                "./bodyset/" + bodyName));
}
//_____________________________________________________________________________
void
ExponentialContact::
extendAddToSystem(SimTK::MultibodySystem& system) const
{
    // Extend the OpenSim::Force parent
    Super::extendAddToSystem(system);

    // Construct the SimTK::ExponentialContact object
    SimTK::GeneralForceSubsystem& forces = _model->updForceSubsystem();
    const SimTK::Transform& XContactPlane = get_contact_plane_transform();
    const SimTK::Vec3& station = get_body_station();
    SimTK::ExponentialSpringForce* spr =
        new SimTK::ExponentialSpringForce(forces, XContactPlane,
            _body->getMobilizedBody(), station, getParameters());

    // Get the subsystem index so we can access the SimTK::Force later.
    ExponentialContact* mutableThis =
        const_cast<ExponentialContact *>(this);
    mutableThis->_spr = spr;
    mutableThis->_index = spr->getForceIndex();
}

//-----------------------------------------------------------------------------
// ACCESSORS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ExponentialContact::
setContactPlaneTransform(const SimTK::Transform& XContactPlane)
{
    set_contact_plane_transform(XContactPlane);
}
//_____________________________________________________________________________
const SimTK::Transform&
ExponentialContact::getContactPlaneTransform() const
{
    return get_contact_plane_transform();
}

//_____________________________________________________________________________
void
ExponentialContact::
setParameters(const SimTK::ExponentialSpringParameters& params)
{
    ExponentialContact::Parameters& p = upd_contact_parameters();
    p.setSimTKParameters(params);
    // The following call will invalidate the System at Stage::Topology
    if (_spr != NULL) _spr->setParameters(params);
}
//_____________________________________________________________________________
const SimTK::ExponentialSpringParameters&
ExponentialContact::
getParameters() const
{
    return get_contact_parameters().getSimTKParameters();
}

//-----------------------------------------------------------------------------
// Reporting
//-----------------------------------------------------------------------------
// Questions:
// 1. Do I need to conform to a certain reporting format because this object
// will be treated as an OpenSim::Force?
//_____________________________________________________________________________
OpenSim::Array<std::string>
ExponentialContact::
getRecordLabels() const
{
    OpenSim::Array<std::string> labels("");
    std::string frameName = getBodyName();
    labels.append(getName()+"."+frameName+".force.X");
    labels.append(getName()+"."+frameName+".force.Y");
    labels.append(getName()+"."+frameName+".force.Z");

    return labels;
}
//_____________________________________________________________________________
OpenSim::Array<double>
ExponentialContact::
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

