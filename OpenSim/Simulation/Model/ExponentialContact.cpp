/* -------------------------------------------------------------------------- *
 *                   OpenSim:  ExponentialContact.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2022-20232 Stanford University and the Authors               *
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
#include "simbody/internal/SimbodyMatterSubsystem.h"
#include "ExponentialContact.h"

using namespace OpenSim;
using namespace std;
using SimTK::Real;
using SimTK::Vec3;
using SimTK::State;


//=============================================================================
// ExponentialContact::Parameters
//=============================================================================
//_____________________________________________________________________________
ExponentialContact::Parameters::
Parameters() {
    setNull();
    constructProperties();
}
//_____________________________________________________________________________
ExponentialContact::Parameters::
Parameters(const SimTK::ExponentialSpringParameters& params) {
    setNull();
    _stkparams = params;
    constructProperties();
}
//_____________________________________________________________________________
void ExponentialContact::Parameters::setNull() {
    setAuthors("F. C. Anderson");
}
//_____________________________________________________________________________
void
ExponentialContact::Parameters::
constructProperties() {
    SimTK::Vec3 shape;
    _stkparams.getShapeParameters(shape[0], shape[1], shape[2]);
    constructProperty_exponential_shape_parameters(shape);
    constructProperty_normal_viscosity(_stkparams.getNormalViscosity());
    constructProperty_max_normal_force(_stkparams.getMaxNormalForce());
    constructProperty_friction_elasticity(_stkparams.getFrictionElasticity());
    constructProperty_friction_viscosity(_stkparams.getFrictionViscosity());
    constructProperty_settle_velocity(_stkparams.getSettleVelocity());
    constructProperty_initial_mu_static(_stkparams.getInitialMuStatic());
    constructProperty_initial_mu_kinetic(_stkparams.getInitialMuKinetic());
}
//_____________________________________________________________________________
// Update the Properties based on the SimTK::ExponentialSpringParameters.
void
ExponentialContact::Parameters::
updateProperties() {
    SimTK::Vec3 shape;
    _stkparams.getShapeParameters(shape[0], shape[1], shape[2]);
    set_exponential_shape_parameters(shape);
    set_normal_viscosity(_stkparams.getNormalViscosity());
    set_max_normal_force(_stkparams.getMaxNormalForce());
    set_friction_elasticity(_stkparams.getFrictionElasticity());
    set_friction_viscosity(_stkparams.getFrictionViscosity());
    set_settle_velocity(_stkparams.getSettleVelocity());
    set_initial_mu_static(_stkparams.getInitialMuStatic());
    set_initial_mu_kinetic(_stkparams.getInitialMuKinetic());
}
//_____________________________________________________________________________
// Update the SimTK::ExponentialSpringParamters based on the Properties.
void
ExponentialContact::Parameters::
updateParameters() {
    const SimTK::Vec3 shape = get_exponential_shape_parameters();
    _stkparams.setShapeParameters(shape[0], shape[1], shape[2]);
    _stkparams.setNormalViscosity(get_normal_viscosity());
    _stkparams.setMaxNormalForce(get_max_normal_force());
    _stkparams.setFrictionElasticity(get_friction_elasticity());
    _stkparams.setFrictionViscosity(get_friction_viscosity());
    _stkparams.setSettleVelocity(get_settle_velocity());
    _stkparams.setInitialMuStatic(get_initial_mu_static());
    _stkparams.setInitialMuKinetic(get_initial_mu_kinetic());
}
//_____________________________________________________________________________
// This method is needed to ensure that the SimTK::ExponentialSpringParameters
// member variable (_stkparam) is kept consistent with the properties.
// It is necessary to have the _stkparams member variable because there needs
// to be a place to store non-default parameters when the underlying
// SimTK::ExponentialSpringForce hasn't yet been instantiated.
// Having to do a little extra bookkeeping (i.e., storing properties values
// twice [once in the Properties and once in _stkparams]) is justified
// by not having to rewrite a whole bunch of additional accessor methods.
// All parameter set/gets go through the SimTK::ExponentialSpringParameters
// interface (i.e., through _stkparams).
void
ExponentialContact::Parameters::
updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber) {
    Super::updateFromXMLNode(node, versionNumber);
    updateParameters(); // catching any invalid property values in the process
    updateProperties(); // pushes valid parameters back to the properties.
}
//_____________________________________________________________________________
// Note that the OpenSim Properties are updated as well.
void
ExponentialContact::Parameters::
setSimTKParameters(const SimTK::ExponentialSpringParameters& params) {
    _stkparams = params;
    updateProperties();
}
//_____________________________________________________________________________
// Get a read-only reference to the SimTK::ExponentialSpringParameters held
// by this instance.
const SimTK::ExponentialSpringParameters&
ExponentialContact::Parameters::
getSimTKParameters() const {
    return _stkparams;
}


//=============================================================================
// ExponentialContact
//=============================================================================
//_____________________________________________________________________________
ExponentialContact::ExponentialContact() {
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
void
ExponentialContact::
setNull() {
    setAuthors("F. C. Anderson");
    _spr = NULL;
}
//_____________________________________________________________________________
void
ExponentialContact::
constructProperties() {
    SimTK::Transform X_GP;
    SimTK::Vec3 origin(0.0);
    Parameters params;
    constructProperty_contact_plane_transform(X_GP);
    constructProperty_body_name("");
    constructProperty_body_station(origin);
    constructProperty_contact_parameters(params);
}
//_____________________________________________________________________________
void
ExponentialContact::
updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber) {
    Super::updateFromXMLNode(node, versionNumber);
}
//_____________________________________________________________________________
void
ExponentialContact::
extendConnectToModel(OpenSim::Model& model) {
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
extendAddToSystem(SimTK::MultibodySystem& system) const {
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

    // Expose the discrete states of ExponentialSpringForce in OpenSim
    bool allocate = false;
    std::string name = getMuStaticDiscreteStateName();
    addDiscreteVariable(name, SimTK::Stage::Dynamics, allocate);
    name = getMuKineticDiscreteStateName();
    addDiscreteVariable(name, SimTK::Stage::Dynamics, allocate);
    name = getSlidingDiscreteStateName();
    addDiscreteVariable(name, SimTK::Stage::Dynamics, allocate);
    name = getAnchorPointDiscreteStateName();
    addDiscreteVariable(name, SimTK::Stage::Dynamics, allocate);
}


//_____________________________________________________________________________
// F. C. Anderson (Jan 2023)
// This method is needed because class OpenSim::ExponentialContact has 4
// discrete states that are allocated in
// SimTK::ExponentialSpringForce::realizeTopology(), not in
// OpenSim::Component::extendRealizeTopology().
// These states are added to the OpenSim map of discrete states
// (i.e., Component::_namedDiscreteVariableInfo) so that they are accessible
// in OpenSim. See ExponentialContact::extendAddToSystem(). However, because
// these discrete states are not allocated by OpenSim::Component, OpenSim has
// no knowledge of their indices into the SimTK::State. The purpose of this
// method is to properly initialize those indices.
void
ExponentialContact::
extendRealizeTopology(SimTK::State& state) const {

    Super::extendRealizeTopology(state);

    // A critical question...
    // Can we guarrantee that ExponentialSpringForce::realizeTopology() will
    // always be called before this method !?!?
    //
    // Maybe. Perhaps the ExponentialSpringForce object will always
    // be listed in the SimTK::System before the ExponentialContact component?
    // If not, then it is possible that getMuStaticStateIndex() will
    // return a bad index.
    //
    // What I can confirm is that, so far, the indices in multiple tests have
    // been assigned correctly.
    //
    // If there are mysterious failures because of a bad State index, the
    // source of the issue may be that ExponentialSpringForce::realizeTopology
    // was not called prior to ExponentialContact::extendRealizeTopology.

    const SimTK::Subsystem* subsys = getSubsystem();
    SimTK::DiscreteVariableIndex index;
    std::string name;

    name = getMuStaticDiscreteStateName();
    index = _spr->getMuStaticStateIndex();
    updDiscreteVariableIndex(name, index, subsys);

    name = getMuKineticDiscreteStateName();
    index = _spr->getMuKineticStateIndex();
    updDiscreteVariableIndex(name, index, subsys);

    name = getSlidingDiscreteStateName();
    index = _spr->getSlidingStateIndex();
    updDiscreteVariableIndex(name, index, subsys);

    name = getAnchorPointDiscreteStateName();
    index = _spr->getAnchorPointStateIndex();
    updDiscreteVariableIndex(name, index, subsys);
}



//-----------------------------------------------------------------------------
// Utility
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void
ExponentialContact::
resetAnchorPoint(SimTK::State& state) const {
    _spr->resetAnchorPoint(state);
}
//_____________________________________________________________________________
void
ExponentialContact::
resetAnchorPoints(OpenSim::ForceSet& fSet, SimTK::State& state) {
    int i;
    int n = fSet.getSize();
    for (i = 0; i < n; ++i) {
        try {
            ExponentialContact& ec =
                    dynamic_cast<ExponentialContact&>(fSet.get(i));
            ec.resetAnchorPoint(state);
        } catch (const std::bad_cast&) {
            // Nothing should happen here. Execution is just skipping any
            // OpenSim::Force that is not an ExponentialContact.
        }
    }
}

//-----------------------------------------------------------------------------
// ACCESSORS for properties
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ExponentialContact::
setContactPlaneTransform(const SimTK::Transform& XContactPlane) {
    set_contact_plane_transform(XContactPlane);
}
//_____________________________________________________________________________
const SimTK::Transform&
ExponentialContact::getContactPlaneTransform() const {
    return get_contact_plane_transform();
}

//_____________________________________________________________________________
void
ExponentialContact::
setParameters(const SimTK::ExponentialSpringParameters& params) {
    ExponentialContact::Parameters& p = upd_contact_parameters();
    p.setSimTKParameters(params);
    // The following call will invalidate the System at Stage::Topology
    if (_spr != NULL) _spr->setParameters(params);
}
//_____________________________________________________________________________
const SimTK::ExponentialSpringParameters&
ExponentialContact::
getParameters() const {
    return get_contact_parameters().getSimTKParameters();
}

//-----------------------------------------------------------------------------
// ACCESSORS for states
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void
ExponentialContact::
setMuStatic(SimTK::State& state, SimTK::Real mus) {
    _spr->setMuStatic(state, mus);
}
//_____________________________________________________________________________
SimTK::Real
ExponentialContact::
getMuStatic(const SimTK::State& state) const {
    return _spr->getMuStatic(state);
}

//_____________________________________________________________________________
void
ExponentialContact::
setMuKinetic(SimTK::State& state, SimTK::Real mus) {
    _spr->setMuKinetic(state, mus);
}
//_____________________________________________________________________________
SimTK::Real
ExponentialContact::
getMuKinetic(const SimTK::State& state) const {
    return _spr->getMuKinetic(state);
}

//_____________________________________________________________________________
SimTK::Real
ExponentialContact::
getSliding(const SimTK::State& state) const {
    return _spr->getSliding(state);
}

//-----------------------------------------------------------------------------
// ACCESSORS for data cache entries
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
Vec3
ExponentialContact::
getNormalForceElasticPart(const State& state, bool inGround) const {
    return _spr->getNormalForceElasticPart(state, inGround);
}
//_____________________________________________________________________________
Vec3
ExponentialContact::
getNormalForceDampingPart(const State& state, bool inGround) const {
    return _spr->getNormalForceDampingPart(state, inGround);
}
//_____________________________________________________________________________
Vec3
ExponentialContact::
getNormalForce(const State& state, bool inGround) const {
    return _spr->getNormalForce(state, inGround);
}
//_____________________________________________________________________________
Real
ExponentialContact::
getMu(const State& state) const {
    return _spr->getMu(state);
}
//_____________________________________________________________________________
Real
ExponentialContact::
getFrictionForceLimit(const SimTK::State& state) const {
    return _spr->getFrictionForceLimit(state);
}
//_____________________________________________________________________________
Vec3
ExponentialContact::
getFrictionForceElasticPart(const State& state, bool inGround) const {
    return _spr->getFrictionForceElasticPart(state, inGround);
}
//_____________________________________________________________________________
Vec3
ExponentialContact::
getFrictionForceDampingPart(const State& state, bool inGround) const {
    return _spr->getFrictionForceDampingPart(state, inGround);
}
//_____________________________________________________________________________
Vec3
ExponentialContact::
getFrictionForce(const State& state, bool inGround) const {
    return _spr->getFrictionForce(state, inGround);
}
//_____________________________________________________________________________
Vec3
ExponentialContact::
getForce(const State& state, bool inGround) const {
    return _spr->getForce(state, inGround);
}
//_____________________________________________________________________________
Vec3
ExponentialContact::
getStationPosition(const State& state, bool inGround) const {
    return _spr->getStationPosition(state, inGround);
}
//_____________________________________________________________________________
Vec3
ExponentialContact::
getStationVelocity(const State& state, bool inGround) const {
    return _spr->getStationVelocity(state, inGround);
}
//_____________________________________________________________________________
Vec3
ExponentialContact::
getAnchorPointPosition(const State& state, bool inGround) const {
    return _spr->getAnchorPointPosition(state, inGround);
}

//-----------------------------------------------------------------------------
// Reporting
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
OpenSim::Array<std::string>
ExponentialContact::
getRecordLabels() const {
    OpenSim::Array<std::string> labels("");
    string name = getName();  // Name of this contact instance.
    std::string frameName = getBodyName();
    std::string groundName = getModel().getGround().getName();

    // Record format consistent with HuntCrossleyForce.
    // Body
    labels.append(name + "." + frameName + ".force.X");
    labels.append(name + "." + frameName + ".force.Y");
    labels.append(name + "." + frameName + ".force.Z");
    labels.append(name + "." + frameName + ".torque.X");
    labels.append(name + "." + frameName + ".torque.Y");
    labels.append(name + "." + frameName + ".torque.Z");
    // Ground
    labels.append(name + "." + groundName + ".force.X");
    labels.append(name + "." + groundName + ".force.Y");
    labels.append(name + "." + groundName + ".force.Z");
    labels.append(name + "." + groundName + ".torque.X");
    labels.append(name + "." + groundName + ".torque.Y");
    labels.append(name + "." + groundName + ".torque.Z");

    return labels;
}
//_____________________________________________________________________________
OpenSim::Array<double>
ExponentialContact::
getRecordValues(const SimTK::State& state) const  {
    OpenSim::Array<double> values(0.0);

    const auto& forceSubsys = getModel().getForceSubsystem();
    const SimTK::Force& abstractForce = forceSubsys.getForce(_index);
    const auto& spr = (SimTK::ExponentialSpringForce&)(abstractForce);

    // Get the loads
    SimTK::Vector_<SimTK::SpatialVec> bForces(0);  // body
    SimTK::Vector_<SimTK::Vec3> pForces(0);  // particle
    SimTK::Vector mForces(0);  // mobility
    spr.calcForceContribution(state, bForces, pForces, mForces);

    // Body
    SimTK::Vec3 force;
    SimTK::Vec3 torque;
    const auto& bodyIndex = _body->getMobilizedBodyIndex();
    SimTK::SpatialVec& bodyForce = bForces(bodyIndex);
    force = bodyForce[1];
    double fy = force[1];
    torque = bodyForce[0];
    values.append(3, &force[0]);
    values.append(3, &torque[0]);

    // Ground
    const SimTK::MultibodySystem& system = _model->getSystem();
    const SimTK::SimbodyMatterSubsystem& matter = system.getMatterSubsystem();
    const SimTK::MobilizedBody& ground = matter.getGround();
    const auto& groundIndex = ground.getMobilizedBodyIndex();
    SimTK::SpatialVec& groundForce = bForces(groundIndex);
    force = groundForce[1];
    torque = groundForce[0];
    values.append(3, &force[0]);
    values.append(3, &torque[0]);

    return values;
}

//-----------------------------------------------------------------------------
// Internal Testing
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void
ExponentialContact::
assertPropertiesAndParametersEqual() const {
    const ExponentialContact::Parameters& a = get_contact_parameters();
    const SimTK::ExponentialSpringParameters& b = getParameters();

    const SimTK::Vec3& vecA = a.get_exponential_shape_parameters();
    SimTK::Vec3 vecB;
    b.getShapeParameters(vecB[0], vecB[1], vecB[2]);
    SimTK_TEST_EQ(vecA[0], vecB[0]);
    SimTK_TEST_EQ(vecA[1], vecB[1]);
    SimTK_TEST_EQ(vecA[2], vecB[2]);

    double valA, valB;
    valA = a.get_normal_viscosity();
    valB = b.getNormalViscosity();
    SimTK_TEST_EQ(valA, valB);

    valA = a.get_friction_elasticity();
    valB = b.getFrictionElasticity();
    SimTK_TEST_EQ(valA, valB);

    valA = a.get_friction_viscosity();
    valB = b.getFrictionViscosity();
    SimTK_TEST_EQ(valA, valB);

    valA = a.get_settle_velocity();
    valB = b.getSettleVelocity();
    SimTK_TEST_EQ(valA, valB);

    valA = a.get_initial_mu_static();
    valB = b.getInitialMuStatic();
    SimTK_TEST_EQ(valA, valB);

    valA = a.get_initial_mu_kinetic();
    valB = b.getInitialMuKinetic();
    SimTK_TEST_EQ(valA, valB);
}
