/* -------------------------------------------------------------------------- *
 *                   OpenSim:  ExponentialContactForce.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2024-2025 Stanford University and the Authors                *
 * Author(s):  F. C. Anderson                                                 *
 * Contributor(s): Nicholas Bianco                                            *
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
#include "ExponentialContactForce.h"

using namespace OpenSim;
using namespace std;
using SimTK::Real;
using SimTK::Vec3;
using SimTK::State;


//=============================================================================
// ExponentialContactForce::Parameters
//=============================================================================
ExponentialContactForce::Parameters::
Parameters() {
    setNull();
    constructProperties();
}

ExponentialContactForce::Parameters::
Parameters(const SimTK::ExponentialSpringParameters& params) {
    setNull();
    _stkparams = params;
    constructProperties();
}

void ExponentialContactForce::Parameters::setNull() {
    setAuthors("F. C. Anderson");
}

void
ExponentialContactForce::Parameters::
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

void
ExponentialContactForce::Parameters::
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

void
ExponentialContactForce::Parameters::
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

// This method is needed to ensure that the SimTK::ExponentialSpringParameters
// member variable (_stkparam) is kept consistent with the properties.
// It is necessary to have the _stkparams member variable because there needs
// to be a place to store non-default parameters when the underlying
// SimTK::ExponentialSpringForce hasn't yet been instantiated.
// Having to do a little extra bookkeeping (i.e., storing properties values
// twice [once in the Properties and once in _stkparams]) is justified
// by not having to rewrite a whole bunch of additional accessor methods.
// All parameter sets/gets go through the SimTK::ExponentialSpringParameters
// interface (i.e., through _stkparams).
void
ExponentialContactForce::Parameters::
updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber) {
    Super::updateFromXMLNode(node, versionNumber);
    updateParameters(); // catching any invalid property values in the process
    updateProperties(); // pushes valid parameters back to the properties.
}

void
ExponentialContactForce::Parameters::
setSimTKParameters(const SimTK::ExponentialSpringParameters& params) {
    _stkparams = params;
    updateProperties();
}

const SimTK::ExponentialSpringParameters&
ExponentialContactForce::Parameters::
getSimTKParameters() const {
    return _stkparams;
}


//=============================================================================
// ExponentialContactForce
//=============================================================================
// Default
ExponentialContactForce::
ExponentialContactForce() {
    setNull();
    constructProperties();
}

// Full Argument
ExponentialContactForce::
ExponentialContactForce(const SimTK::Transform& contactPlaneXform,
    const PhysicalFrame& frame, const SimTK::Vec3& location,
    SimTK::ExponentialSpringParameters params)
{
    setNull();
    constructProperties();
    set_contact_plane_transform(contactPlaneXform);

    // Create a Station object and set the station property.
    set_station(Station());
    upd_station().setParentFrame(frame);
    upd_station().set_location(location);

    setParameters(params);
}

void
ExponentialContactForce::
setNull() {
    setAuthors("F. C. Anderson");
}

const SimTK::ExponentialSpringForce&
ExponentialContactForce::
getExponentialSpringForce() const {
    SimTK_ASSERT(_index.isValid(), "Invalid Force index.");
    const auto& forceSubsys = getModel().getForceSubsystem();
    const SimTK::Force& abstractForce = forceSubsys.getForce(_index);
    return static_cast<const SimTK::ExponentialSpringForce&>(abstractForce);
}

SimTK::ExponentialSpringForce&
ExponentialContactForce::
updExponentialSpringForce() {
    SimTK_ASSERT(_index.isValid(), "Invalid Force index.");
    auto& forceSubsys = updModel().updForceSubsystem();
    SimTK::Force& abstractForce = forceSubsys.updForce(_index);
    return static_cast<SimTK::ExponentialSpringForce&>(abstractForce);
}

void
ExponentialContactForce::
constructProperties() {
    constructProperty_contact_plane_transform(SimTK::Transform());
    constructProperty_contact_parameters(Parameters());
    constructProperty_station(Station());
}

void
ExponentialContactForce::
updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber) {
    Super::updateFromXMLNode(node, versionNumber);
}

void
ExponentialContactForce::
extendConnectToModel(OpenSim::Model& model) {
    // Allow base class to connect first
    Super::extendConnectToModel(model);

    // The station should not be connected to Ground.
    const PhysicalFrame& frame = get_station().getParentFrame();
    OPENSIM_THROW_IF(&frame == &model.getGround(), Exception,
        "The station must be connected to a PhysicalFrame that is not Ground.")
}

// This method is where the actual underlying contact force subsystem
// (i.e., SimTK::ExponentialSpringForce) is constructed and added to the
// OpenSim::Model (wrapped by OpenSim::ExponentialContactForce).
void
ExponentialContactForce::
extendAddToSystem(SimTK::MultibodySystem& system) const {
    // Extend the OpenSim::Force parent
    Super::extendAddToSystem(system);

    // Construct the SimTK::ExponentialSpringForce object
    SimTK::GeneralForceSubsystem& forces = _model->updForceSubsystem();
    const SimTK::Transform& XContactPlane = get_contact_plane_transform();
    const PhysicalFrame& frame = get_station().getParentFrame();
    const Vec3& location = get_station().get_location();
    SimTK::ExponentialSpringForce spr(forces, XContactPlane,
            frame.getMobilizedBody(), location, getParameters());

    // Get the subsystem index so we can access the SimTK::Force later.
    ExponentialContactForce* mutableThis =
        const_cast<ExponentialContactForce*>(this);
    mutableThis->_index = spr.getForceIndex();

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

// This method is needed because class OpenSim::ExponentialContactForce has 4
// discrete states that are allocated in
// SimTK::ExponentialSpringForce::realizeTopology(), not in
// OpenSim::Component::extendRealizeTopology().
// These states are added to the OpenSim map of discrete states
// (i.e., Component::_namedDiscreteVariableInfo) so that they are accessible
// in OpenSim. See ExponentialContactForce::extendAddToSystem(). However,
// because these discrete states are not allocated by OpenSim::Component,
// OpenSim has no knowledge of their indices into the SimTK::State. The purpose
// of this method is to properly initialize those indices.
void
ExponentialContactForce::
extendRealizeTopology(SimTK::State& state) const {

    Super::extendRealizeTopology(state);

    // A critical question...
    // Can we guarrantee that ExponentialSpringForce::realizeTopology() will
    // always be called before this method !?!?
    //
    // Maybe. Perhaps the ExponentialSpringForce object will always
    // be listed in the SimTK::System before the ExponentialContactForce
    // component? If not, then it is possible that getMuStaticStateIndex()
    // will return a bad index.
    //
    // Or, perhaps the call above to Super::extendRealizeTopology(state)
    // ensures that the SimTK::ExponentialSpringForce::realizeTopology()
    // has been called before this method is executed.
    //
    // What I can confirm is that, so far, the indices in multiple tests have
    // been assigned correctly.
    //
    // If there are mysterious failures because of a bad State index, the
    // source of the issue may be that ExponentialSpringForce::realizeTopology
    // was not called prior to ExponentialContactForce::extendRealizeTopology.

    const SimTK::Subsystem* subsys = getSubsystem();
    const SimTK::SubsystemIndex ssIndex = subsys->getMySubsystemIndex();
    subsys->getMySubsystemIndex();
    SimTK::DiscreteVariableIndex dvIndex;
    std::string name;

    name = getMuStaticDiscreteStateName();
    dvIndex = getExponentialSpringForce().getMuStaticStateIndex();
    initializeDiscreteVariableIndexes(name, ssIndex, dvIndex);

    name = getMuKineticDiscreteStateName();
    dvIndex = getExponentialSpringForce().getMuKineticStateIndex();
    initializeDiscreteVariableIndexes(name, ssIndex, dvIndex);

    name = getSlidingDiscreteStateName();
    dvIndex = getExponentialSpringForce().getSlidingStateIndex();
    initializeDiscreteVariableIndexes(name, ssIndex, dvIndex);

    name = getAnchorPointDiscreteStateName();
    dvIndex = getExponentialSpringForce().getAnchorPointStateIndex();
    initializeDiscreteVariableIndexes(name, ssIndex, dvIndex);
}

//-----------------------------------------------------------------------------
// Utility
//-----------------------------------------------------------------------------
void
ExponentialContactForce::
resetAnchorPoint(SimTK::State& state) const {
    getExponentialSpringForce().resetAnchorPoint(state);
}

void
ExponentialContactForce::
resetAnchorPoints(OpenSim::Model& model, SimTK::State& state) {
    for (auto& ec : model.updComponentList<ExponentialContactForce>()) {
        ec.resetAnchorPoint(state);
    }
}

//-----------------------------------------------------------------------------
// ACCESSORS for properties
//-----------------------------------------------------------------------------
void
ExponentialContactForce::
setParameters(const SimTK::ExponentialSpringParameters& params) {
    ExponentialContactForce::Parameters& p = upd_contact_parameters();
    p.setSimTKParameters(params);
    // Push the new parameters to the SimTK::ExponentialSpringForce instance.
    // The following call will invalidate the System at Stage::Topology.
    if (_index.isValid()) updExponentialSpringForce().setParameters(params);
}

const SimTK::ExponentialSpringParameters&
ExponentialContactForce::
getParameters() const {
    return get_contact_parameters().getSimTKParameters();
}

const Station&
ExponentialContactForce::
getStation() const {
    return get_station();
}

//-----------------------------------------------------------------------------
// ACCESSORS for states
//-----------------------------------------------------------------------------
void
ExponentialContactForce::
setMuStatic(SimTK::State& state, SimTK::Real mus) {
   updExponentialSpringForce().setMuStatic(state, mus);
}

SimTK::Real
ExponentialContactForce::
getMuStatic(const SimTK::State& state) const {
    return getExponentialSpringForce().getMuStatic(state);
}

void
ExponentialContactForce::
setMuKinetic(SimTK::State& state, SimTK::Real muk) {
    updExponentialSpringForce().setMuKinetic(state, muk);
}

SimTK::Real
ExponentialContactForce::
getMuKinetic(const SimTK::State& state) const {
    return getExponentialSpringForce().getMuKinetic(state);
}

SimTK::Real
ExponentialContactForce::
getSliding(const SimTK::State& state) const {
    return getExponentialSpringForce().getSliding(state);
}

//-----------------------------------------------------------------------------
// ACCESSORS for data cache entries
//-----------------------------------------------------------------------------
Vec3
ExponentialContactForce::
getNormalForceElasticPart(const State& state, bool inGround) const {
    return
        getExponentialSpringForce().getNormalForceElasticPart(state, inGround);
}

Vec3
ExponentialContactForce::
getNormalForceDampingPart(const State& state, bool inGround) const {
    return
        getExponentialSpringForce().getNormalForceDampingPart(state, inGround);
}

Vec3
ExponentialContactForce::
getNormalForce(const State& state, bool inGround) const {
    return getExponentialSpringForce().getNormalForce(state, inGround);
}

Real
ExponentialContactForce::
getMu(const State& state) const {
    return getExponentialSpringForce().getMu(state);
}

Real
ExponentialContactForce::
getFrictionForceLimit(const SimTK::State& state) const {
    return getExponentialSpringForce().
                getFrictionForceLimit(state);
}

Vec3
ExponentialContactForce::
getFrictionForceElasticPart(const State& state, bool inGround) const {
    return getExponentialSpringForce().
                getFrictionForceElasticPart(state, inGround);
}

Vec3
ExponentialContactForce::
getFrictionForceDampingPart(const State& state, bool inGround) const {
    return getExponentialSpringForce().
                getFrictionForceDampingPart(state, inGround);
}

Vec3
ExponentialContactForce::
getFrictionForce(const State& state, bool inGround) const {
    return getExponentialSpringForce().getFrictionForce(state, inGround);
}

Vec3
ExponentialContactForce::
getForce(const State& state, bool inGround) const {
    return getExponentialSpringForce().getForce(state, inGround);
}

Vec3
ExponentialContactForce::
getStationPosition(const State& state, bool inGround) const {
    return getExponentialSpringForce().getStationPosition(state, inGround);
}

Vec3
ExponentialContactForce::
getStationVelocity(const State& state, bool inGround) const {
    return getExponentialSpringForce().getStationVelocity(state, inGround);
}

Vec3
ExponentialContactForce::
getAnchorPointPosition(const State& state, bool inGround) const {
    return getExponentialSpringForce().getAnchorPointPosition(state, inGround);
}

//-----------------------------------------------------------------------------
// Reporting
//-----------------------------------------------------------------------------
OpenSim::Array<std::string>
ExponentialContactForce::
getRecordLabels() const {
    OpenSim::Array<std::string> labels("");
    string name = getName();  // Name of this contact instance.
    std::string frameName = get_station().getParentFrame().getName();
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

OpenSim::Array<double>
ExponentialContactForce::
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
    const auto& bodyIndex = get_station().getParentFrame()
            .getMobilizedBodyIndex();
    SimTK::SpatialVec& bodyForce = bForces(bodyIndex);
    force = bodyForce[1];
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
void
ExponentialContactForce::
assertPropertiesAndParametersEqual() const {
    const ExponentialContactForce::Parameters& a = get_contact_parameters();
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
