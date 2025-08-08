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
    constructProperty_contact_hint(SimTK::Vec3(SimTK::NaN));
}

const ContactGeometry&
Scholz2015GeometryPathObstacle::getContactGeometry() const {
    return getSocket<ContactGeometry>("contact_geometry").getConnectee();
}

//=============================================================================
// SCHOLZ 2015 GEOMETRY PATH SEGMENT
//=============================================================================
Scholz2015GeometryPathSegment::Scholz2015GeometryPathSegment() : Component() {
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
// CONSTRUCTION
//=============================================================================
Scholz2015GeometryPath::Scholz2015GeometryPath() : AbstractGeometryPath() {
    constructProperties();
    append_segments(Scholz2015GeometryPathSegment());
    upd_segments(0).setName("path_segment_0");
}

Scholz2015GeometryPath::Scholz2015GeometryPath(
        const PhysicalFrame& originFrame,
        const SimTK::Vec3& originLocation,
        const PhysicalFrame& insertionFrame,
        const SimTK::Vec3& insertionLocation) : Scholz2015GeometryPath() {
    setOrigin(originFrame, originLocation);
    setInsertion(insertionFrame, insertionLocation);
}

//=============================================================================
// PATH CONFIGURATION
//=============================================================================
void Scholz2015GeometryPath::setOrigin(const PhysicalFrame& frame,
        const SimTK::Vec3& location) {
    // Create the origin station.
    set_origin(Station());
    Station& origin = upd_origin();
    origin.setName("origin");
    origin.set_location(location);
    origin.setParentFrame(frame);

    // Call finalizeFromProperties() to ensure that the station is recognized
    // as a subcomponent.
    finalizeFromProperties();

    // Connect the station to the origin of the first path segment.
    auto& segment = upd_segments(0);
    segment.connectSocket_origin(upd_origin());
}

const Station& Scholz2015GeometryPath::getOrigin() const {
    return get_origin();
}

void Scholz2015GeometryPath::setInsertion(const PhysicalFrame& frame,
        const SimTK::Vec3& location) {
    // Create the insertion station.
    set_insertion(Station());
    Station& insertion = upd_insertion();
    insertion.setName("insertion");
    insertion.set_location(location);
    insertion.setParentFrame(frame);

    // Call finalizeFromProperties() to ensure that the station is recognized
    // as a subcomponent.
    finalizeFromProperties();

    // Connect the station to the insertion of the last path segment.
    auto& segment = upd_segments(getProperty_segments().size() - 1);
    segment.connectSocket_insertion(upd_insertion());
}

const Station& Scholz2015GeometryPath::getInsertion() const {
    return get_insertion();
}

void Scholz2015GeometryPath::addObstacle(const ContactGeometry& contactGeometry,
        const SimTK::Vec3& contactHint) {
    // Add the obstacle to the last path segment.
    auto& segment = upd_segments(getProperty_segments().size() - 1);
    segment.append_obstacles(Scholz2015GeometryPathObstacle());
    auto& ob = segment.upd_obstacles(segment.getNumObstacles() - 1);
    ob.setName("path_obstacle_" + std::to_string(segment.getNumObstacles() - 1));
    ob.connectSocket_contact_geometry(contactGeometry);
    ob.set_contact_hint(contactHint);
}

int Scholz2015GeometryPath::getNumObstacles() const {
    int numObstacles = 0;
    for (int i = 0; i < getProperty_segments().size(); ++i) {
        numObstacles += get_segments(i).getNumObstacles();
    }
    return numObstacles;
}

void Scholz2015GeometryPath::addViaPoint(const PhysicalFrame& frame,
        const SimTK::Vec3& location) {

    // Create the via point station.
    int vix = append_via_points(Station());
    Station& viaPoint = upd_via_points(vix);
    viaPoint.setName(fmt::format("via_point_{}", vix));
    viaPoint.set_location(location);
    viaPoint.setParentFrame(frame);

    // Call finalizeFromProperties() to ensure that the station is recognized
    // as a subcomponent.
    finalizeFromProperties();

    // Update the insertion of the last path segment.
    auto& currSegment = upd_segments(getProperty_segments().size() - 1);
    currSegment.connectSocket_insertion(upd_via_points(vix));

    // Create a new path segment.
    append_segments(Scholz2015GeometryPathSegment());
    auto& nextSegment = upd_segments(getProperty_segments().size() - 1);
    nextSegment.setName(fmt::format("path_segment_{}",
            getProperty_segments().size() - 1));
    nextSegment.connectSocket_origin(upd_via_points(vix));
    nextSegment.connectSocket_insertion(getInsertion());
}

int Scholz2015GeometryPath::getNumViaPoints() const {
    return getProperty_via_points().size();
}

//=============================================================================
// SOLVER CONFIGURATION
//=============================================================================

void Scholz2015GeometryPath::setAlgorithm(std::string algorithm) {
    set_algorithm(std::move(algorithm));
}

const std::string& Scholz2015GeometryPath::getAlgorithm() const {
    return get_algorithm();
}

void Scholz2015GeometryPath::setCurveSegmentAccuracy(double accuracy) {
    set_curve_segment_accuracy(accuracy);
}

double Scholz2015GeometryPath::getCurveSegmentAccuracy() const {
    return get_curve_segment_accuracy();
}

void Scholz2015GeometryPath::setSmoothnessTolerance(double tolerance) {
    set_smoothness_tolerance(tolerance);
}

double Scholz2015GeometryPath::getSmoothnessTolerance() const {
    return get_smoothness_tolerance();
}

double Scholz2015GeometryPath::getSmoothness(const SimTK::State& state) const {
    return getCableSpan().getSmoothness(state);
}

void Scholz2015GeometryPath::setSolverMaxIterations(int maxIterations) {
    set_solver_max_iterations(maxIterations);
}

int Scholz2015GeometryPath::getSolverMaxIterations() const {
    return get_solver_max_iterations();
}

int Scholz2015GeometryPath::getNumSolverIterations(
        const SimTK::State& state) const {
    return getCableSpan().getNumSolverIterations(state);
}

//=============================================================================
// CURVE SEGMENT COMPUTATIONS
//=============================================================================
bool Scholz2015GeometryPath::isInContactWithObstacle(
        const SimTK::State& state, SimTK::CableSpanObstacleIndex ix) const {
    return getCableSpan().isInContactWithObstacle(state, ix);
}

SimTK::Transform Scholz2015GeometryPath::calcCurveSegmentInitialFrenetFrame(
        const SimTK::State& state, SimTK::CableSpanObstacleIndex ix) const {
    return getCableSpan().calcCurveSegmentInitialFrenetFrame(state, ix);
}

SimTK::Transform Scholz2015GeometryPath::calcCurveSegmentFinalFrenetFrame(
        const SimTK::State& state, SimTK::CableSpanObstacleIndex ix) const {
    return getCableSpan().calcCurveSegmentFinalFrenetFrame(state, ix);
}

SimTK::Real Scholz2015GeometryPath::calcCurveSegmentArcLength(
        const SimTK::State& state, SimTK::CableSpanObstacleIndex ix) const {
    return getCableSpan().calcCurveSegmentArcLength(state, ix);
}

//=============================================================================
// VIA POINT COMPUTATIONS
//=============================================================================
SimTK::Vec3 Scholz2015GeometryPath::calcViaPointLocation(
        const SimTK::State& state, SimTK::CableSpanViaPointIndex ix) const {
    return getCableSpan().calcViaPointLocation(state, ix);
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

double Scholz2015GeometryPath::computeMomentArm(const SimTK::State& s,
        const Coordinate& coord) const {

    if (!_maSolver) {
        const_cast<Self*>(this)->_maSolver.reset(
            new MomentArmSolver(getModel()));
    }

    return _maSolver->solve(s, coord,  *this);
}

void Scholz2015GeometryPath::produceForces(const SimTK::State& state,
        double tension, ForceConsumer& forceConsumer) const {

    if (tension < 0.) {
        return;
    }

    const SimTK::CableSpan& cable = getCableSpan();
    const Station& origin = getOrigin();
    const Station& insertion = getInsertion();
    SimTK::SpatialVec unitBodyForce;

    // Force applied at path origin point.
    {
        cable.calcOriginUnitForce(state, unitBodyForce);
        forceConsumer.consumeBodySpatialVec(state, origin.getParentFrame(),
                tension * unitBodyForce);
    }

    // Forces applied to each obstacle body.
    for (const auto& [index, frame] : _obstacleIndexMap) {
        SimTK::CableSpanObstacleIndex ix(index);
        if (!cable.isInContactWithObstacle(state, ix)) {
            continue;
        }

        cable.calcCurveSegmentUnitForce(state, ix, unitBodyForce);
        forceConsumer.consumeBodySpatialVec(state, frame.getRef(),
                tension * unitBodyForce);
    }

    // Forces applied to each via point.
    for (const auto& [index, frame] : _viaPointIndexMap) {
        SimTK::CableSpanViaPointIndex ix(index);
        cable.calcViaPointUnitForce(state, ix, unitBodyForce);
        forceConsumer.consumeBodySpatialVec(state, frame.getRef(),
                tension * unitBodyForce);
    }

    // Force applied at path insertion point.
    {
        cable.calcTerminationUnitForce(state, unitBodyForce);
        forceConsumer.consumeBodySpatialVec(
            state, insertion.getParentFrame(), unitBodyForce * tension);
    }
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void Scholz2015GeometryPath::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();

    SimTK_ERRCHK2_ALWAYS(get_curve_segment_accuracy() > 0,
            "Scholz2015GeometryPath::extendFinalizeFromProperties",
            "%s: curve_segment_accuracy must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_curve_segment_accuracy());

    SimTK_ERRCHK2_ALWAYS(get_smoothness_tolerance() > 0,
            "Scholz2015GeometryPath::extendFinalizeFromProperties",
            "%s: smoothness_tolerance must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_smoothness_tolerance());

    SimTK_ERRCHK2_ALWAYS(get_solver_max_iterations() > 0,
            "Scholz2015GeometryPath::extendFinalizeFromProperties",
            "%s: solver_max_iterations must be greater than zero, "
            "but it is %d.",
            getName().c_str(), get_solver_max_iterations());

    checkPropertyValueIsInSet(getProperty_algorithm(),
            {"Scholz2015", "MinimumLength"});
    if (get_algorithm() == "Scholz2015") {
        _algorithm = SimTK::CableSpanAlgorithm::Scholz2015;
    } else if (get_algorithm() == "MinimumLength") {
        _algorithm = SimTK::CableSpanAlgorithm::MinimumLength;
    }
}

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
                obstacle.getContactGeometry().getFrame().getMobilizedBodyIndex(),
                obstacle.getContactGeometry().getTransform(),
                obstacle.getContactGeometry().getSimTKContactGeometryPtr(),
                obstacle.get_contact_hint());
            _obstacleIndexMap[ix] = &obstacle.getContactGeometry().getFrame();
        }

        // Add the via point if this is not the last segment.
        if (iseg < getProperty_segments().size() - 1) {
            SimTK::CableSpanViaPointIndex ix = cable.addViaPoint(
                segment.getInsertion().getParentFrame().getMobilizedBodyIndex(),
                segment.getInsertion().get_location());
            _viaPointIndexMap[ix] = &segment.getInsertion().getParentFrame();
        }
    }

    cable.setSmoothnessTolerance(getSmoothnessTolerance());
    cable.setCurveSegmentAccuracy(getCurveSegmentAccuracy());
    cable.setSolverMaxIterations(getSolverMaxIterations());
    cable.setAlgorithm(_algorithm);
    _index = cable.getIndex();
}

void Scholz2015GeometryPath::generateDecorations(bool fixed,
        const ModelDisplayHints&, const SimTK::State& s,
        SimTK::Array_<SimTK::DecorativeGeometry>& geoms) const {
    if (fixed) { return; }

    getCableSpan().calcDecorativePathPoints(s,
        [&](SimTK::Vec3 x_G)
        {
            geoms.push_back(SimTK::DecorativeSphere(0.005).setTransform(x_G)
                    .setColor(SimTK::Blue));
        });
}

//=============================================================================
// CONVENIENCE METHODS
//=============================================================================
void Scholz2015GeometryPath::constructProperties() {
    constructProperty_origin(Station());
    constructProperty_insertion(Station());
    constructProperty_via_points();
    constructProperty_segments();
    constructProperty_algorithm("Scholz2015");
    constructProperty_solver_max_iterations(50);
    constructProperty_curve_segment_accuracy(1e-9);
    constructProperty_smoothness_tolerance(0.1 / 180. * SimTK::Pi);
}

const SimTK::CableSpan& Scholz2015GeometryPath::getCableSpan() const {
    return getModel().getMultibodySystem().getCableSubsystem().getCable(_index);
}

SimTK::CableSpan& Scholz2015GeometryPath::updCableSpan() {
    return getModel().updMultibodySystem().updCableSubsystem().updCable(_index);
}
