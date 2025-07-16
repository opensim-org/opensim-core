/* -------------------------------------------------------------------------- *
 *                   OpenSim:  Scholz2015GeometryPath.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
 * Contributor(s): Pepijn van den Bos, Andreas Scholz                         *
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

#include "Scholz2015GeometryPath.h"

#include <OpenSim/Simulation/Model/ForceConsumer.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

//=============================================================================
// SCHOLZ 2015 GEOMETRY PATH OBSTACLE
//=============================================================================
Scholz2015GeometryPathObstacle::Scholz2015GeometryPathObstacle() : Component() {
    constructProperties();
}

void Scholz2015GeometryPathObstacle::constructProperties() {
    constructProperty_contact_hint(SimTK::Vec3(SimTK::NaN));
}

const ContactGeometry& Scholz2015GeometryPathObstacle::getObstacle() const {
    return getSocket<ContactGeometry>("obstacle").getConnectee();
}

//=============================================================================
// SCHOLZ 2015 GEOMETRY PATH SEGMENT
//=============================================================================
Scholz2015GeometryPathSegment::Scholz2015GeometryPathSegment() : Component() {
    constructProperties();
}

void Scholz2015GeometryPathSegment::constructProperties() {
    constructProperty_obstacles();
}

int Scholz2015GeometryPathSegment::getNumObstacles() const {
    return getProperty_obstacles().size();
}

const Station& Scholz2015GeometryPathSegment::getOrigin() const {
    return getSocket<Station>("origin").getConnectee();
}

const Station& Scholz2015GeometryPathSegment::getInsertion() const {
    return getSocket<Station>("insertion").getConnectee();
}

//=============================================================================
// CONSTRUCTOR
//=============================================================================
Scholz2015GeometryPath::Scholz2015GeometryPath() : AbstractGeometryPath() {
    constructProperties();
    append_segments(Scholz2015GeometryPathSegment());
}

Scholz2015GeometryPath::Scholz2015GeometryPath(const Station& origin,
        const Station& insertion) : Scholz2015GeometryPath() {
    setOrigin(origin);
    setInsertion(insertion);
}

void Scholz2015GeometryPath::setOrigin(const Station& origin) {
    connectSocket_origin(origin);
    auto& segment = upd_segments(0);
    segment.connectSocket_origin(origin);
}

void Scholz2015GeometryPath::setInsertion(const Station& insertion) {
    connectSocket_insertion(insertion);
    auto& segment = upd_segments(getProperty_segments().size() - 1);
    segment.connectSocket_insertion(insertion);
}

void Scholz2015GeometryPath::addObstacle(const ContactGeometry& obstacle,
        const SimTK::Vec3& contactHint) {
    // Add the obstacle to the last path segment.
    auto& segment = upd_segments(getProperty_segments().size() - 1);
    segment.append_obstacles(Scholz2015GeometryPathObstacle());
    auto& ob = segment.upd_obstacles(segment.getNumObstacles() - 1);
    ob.connectSocket_obstacle(obstacle);
    ob.set_contact_hint(contactHint);
}

void Scholz2015GeometryPath::addViaPoint(const Station& viaPoint) {
    // Add the via point as a subcomponent.
    int ivp = append_via_points(viaPoint);
    finalizeFromProperties();

    // Update the insertion of the last path segment.
    auto& currSegment = upd_segments(getProperty_segments().size() - 1);
    currSegment.connectSocket_insertion(upd_via_points(ivp));

    // Create a new path segment.
    append_segments(Scholz2015GeometryPathSegment());
    auto& nextSegment = upd_segments(getProperty_segments().size() - 1);
    nextSegment.connectSocket_origin(upd_via_points(ivp));
    nextSegment.connectSocket_insertion(getInsertion());
}

//=============================================================================
// GET AND SET METHODS
//=============================================================================
const Station& Scholz2015GeometryPath::getOrigin() const {
    return getSocket<Station>("origin").getConnectee();
}

const Station& Scholz2015GeometryPath::getInsertion() const {
    return getSocket<Station>("insertion").getConnectee();
}

//=============================================================================
// ABSTRACT PATH INTERFACE
//=============================================================================
double Scholz2015GeometryPath::getLength(const SimTK::State& s) const {
    return getCableSpan().calcLength(s);
}

double Scholz2015GeometryPath::getLengtheningSpeed(const SimTK::State& s) const {
    return getCableSpan().calcLengthDot(s);
}

Scholz2015GeometryPath::MomentArmSolver::MomentArmSolver(
        const Model& model) : _model(&model) {
    _state = model.getWorkingState();
}

double Scholz2015GeometryPath::MomentArmSolver::solve(const SimTK::State& state,
        const Coordinate& coordinate,
        const Scholz2015GeometryPath& path) const {

    SimTK::State& s = _state;
    SimTK::MobilizerQIndex qIndex = coordinate.getMobilizerQIndex();

    s.updQ() = state.getQ();
    SimTK::Vector qdot = s.getQDot();
    qdot = 0.;
    qdot[qIndex] = 1.;

    SimTK::Vector u = s.getU();
    u = 0.;

    const SimTK::SimbodyMatterSubsystem& matter =
        getModel().getMultibodySystem().getMatterSubsystem();
    matter.multiplyByNInv(s, false, qdot, u);

    s.updU() = u;

    getModel().realizeVelocity(s);
    return path.getLengtheningSpeed(s);
}

double Scholz2015GeometryPath::computeMomentArm(const SimTK::State& s,
        const Coordinate& coord) const {

    if (!_maSolver) {
        const_cast<Self*>(this)->_maSolver.reset(
                new Scholz2015GeometryPath::MomentArmSolver(getModel()));
    }

    return _maSolver->solve(s, coord,  *this);
}

void Scholz2015GeometryPath::produceForces(const SimTK::State& state,
        double tension, ForceConsumer& forceConsumer) const {

    // This implementation mirrors SimTK::CableSpan::applyBodyForces().

    if (tension <= 0.) {
        return;
    }

    const SimTK::CableSpan& cable = getCableSpan();
    const Socket<Station>& viaPoints = getSocket<Station>("via_points");
    SimTK::SpatialVec unitBodyForce;

    const Station& origin = getOrigin();
    const Station& insertion = getInsertion();

    // Force applied at cable origin point.
    {
        cable.calcOriginUnitForce(state, unitBodyForce);
        forceConsumer.consumeBodySpatialVec(state,
                cable.getOriginBodyIndex(),
                unitBodyForce * tension);
    }

    // Forces applied to each obstacle body.
    for (SimTK::CableSpanObstacleIndex ix : _obstacleIndexes) {
        if (!cable.isInContactWithObstacle(state, ix)) {
            continue;
        }
        cable.calcCurveSegmentUnitForce(state, ix, unitBodyForce);
        forceConsumer.consumeBodySpatialVec(state,
                cable.getObstacleMobilizedBodyIndex(ix),
                unitBodyForce * tension);
    }

    // Forces applied to each via point.
    for (SimTK::CableSpanViaPointIndex ix : _viaPointIndexes) {
        cable.calcViaPointUnitForce(state, ix, unitBodyForce);
        const Station& viaPoint = viaPoints.getConnectee(ix);
        forceConsumer.consumeBodySpatialVec(state,
                cable.getViaPointMobilizedBodyIndex(ix),
                unitBodyForce * tension);
    }

    // Force applied at cable termination point.
    {
        cable.calcTerminationUnitForce(state, unitBodyForce);
        forceConsumer.consumeBodySpatialVec(state,
                cable.getTerminationBodyIndex(), unitBodyForce * tension);
    }

}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void Scholz2015GeometryPath::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);

    const Station& origin = getOrigin();
    const Station& insertion = getInsertion();

    SimTK::CableSubsystem& cables = system.updCableSubsystem();
    SimTK::CableSpan cable(cables,
        origin.getParentFrame().getMobilizedBodyIndex(),
        origin.get_location(),
        insertion.getParentFrame().getMobilizedBodyIndex(),
        insertion.get_location());

    for (int iseg = 0; iseg < getProperty_segments().size(); ++iseg) {
        auto& segment = get_segments(iseg);
        for (int iobs = 0; iobs < segment.getNumObstacles(); ++iobs) {
            auto& obstacle = segment.get_obstacles(iobs);
            SimTK::CableSpanObstacleIndex ix = cable.addObstacle(
                obstacle.getObstacle().getFrame().getMobilizedBodyIndex(),
                obstacle.getObstacle().getTransform(),
                std::shared_ptr<SimTK::ContactGeometry>(
                    new SimTK::ContactGeometry(
                        obstacle.getObstacle().createSimTKContactGeometry())),
                obstacle.get_contact_hint());
            _obstacleIndexes.push_back(ix);
        }

        // Add the via point if this is not the last segment.
        if (iseg < getProperty_segments().size() - 1) {
            SimTK::CableSpanViaPointIndex ix = cable.addViaPoint(
                segment.getInsertion().getParentFrame().getMobilizedBodyIndex(),
                segment.getInsertion().get_location());
            _viaPointIndexes.push_back(ix);
        }
    }

    cable.setAlgorithm(get_algorithm());
    _index = cable.getIndex();
}

void Scholz2015GeometryPath::generateDecorations(bool fixed,
        const ModelDisplayHints& hints, const SimTK::State& s,
        SimTK::Array_<SimTK::DecorativeGeometry>& geoms) const {
    if (fixed) { return; }

    getCableSpan().calcDecorativePathPoints(s,
        [&](SimTK::Vec3 x_G)
        {
            geoms.push_back(SimTK::DecorativeSphere(0.001).setTransform(x_G)
                    .setColor(SimTK::Blue));
        });
}

//=============================================================================
// CONVENIENCE METHODS
//=============================================================================
void Scholz2015GeometryPath::constructProperties() {
    constructProperty_via_points();
    constructProperty_segments();
    constructProperty_algorithm(SimTK::CableSpanAlgorithm::Scholz2015);
}

const SimTK::CableSpan& Scholz2015GeometryPath::getCableSpan() const {
    return getModel().getMultibodySystem().getCableSubsystem().getCable(_index);
}

SimTK::CableSpan& Scholz2015GeometryPath::updCableSpan() {
    return getModel().updMultibodySystem().updCableSubsystem().updCable(_index);
}
