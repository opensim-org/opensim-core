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
Scholz2015GeometryPathObstacle::Scholz2015GeometryPathObstacle() {
    constructProperty_contact_hint(SimTK::Vec3(SimTK::NaN));
}

const ContactGeometry&
Scholz2015GeometryPathObstacle::getContactGeometry() const {
    return getSocket<ContactGeometry>("contact_geometry").getConnectee();
}

//=============================================================================
// SCHOLZ 2015 GEOMETRY PATH SEGMENT
//=============================================================================
Scholz2015GeometryPathSegment::Scholz2015GeometryPathSegment() {
    constructProperty_termination(Station());
    constructProperty_obstacles();
}

int Scholz2015GeometryPathSegment::getNumObstacles() const {
    return getProperty_obstacles().size();
}

const Station& Scholz2015GeometryPathSegment::getTermination() const {
    return get_termination();
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
Scholz2015GeometryPath::Scholz2015GeometryPath() : AbstractGeometryPath() {
    constructProperties();
}

//=============================================================================
// PATH CONFIGURATION
//=============================================================================
void Scholz2015GeometryPath::addPathPoint(const PhysicalFrame& frame,
        const SimTK::Vec3& location) {

    // If the origin is not yet connected, set the origin station and create the
    // first path segment. Otherwise, set the insertion station of the last
    // segment, creating a new segment if needed.
    if (!pathHasOrigin()) {
        set_origin(Station());
        Station& origin = upd_origin();
        origin.setName("origin");
        origin.set_location(location);
        origin.setParentFrame(frame);

        append_segments(Scholz2015GeometryPathSegment());
        upd_segments(0).setName("path_segment_0");
    } else {
        addPathSegmentIfNeeded();
        auto& segment = updLastPathSegment();
        Station& termination = segment.upd_termination();
        termination.setName("termination");
        termination.set_location(location);
        termination.setParentFrame(frame);
    }

    // Call finalizeFromProperties() to ensure that the station is recognized
    // as a subcomponent.
    finalizeFromProperties();
}

const Station& Scholz2015GeometryPath::getOrigin() const {
    return get_origin();
}

const Station& Scholz2015GeometryPath::getInsertion() const {
    OPENSIM_THROW_IF_FRMOBJ(getProperty_segments().empty(), Exception,
            "Expected at least two path points before retrieving the insertion "
            "point, but the path only has {} path point(s).",
            getNumPathPoints());

    return getLastPathSegment().getTermination();
}

void Scholz2015GeometryPath::addObstacle(const ContactGeometry& contactGeometry,
        const SimTK::Vec3& contactHint) {
    OPENSIM_THROW_IF_FRMOBJ(getProperty_segments().empty(), Exception,
            "Expected at least one path point before adding an obstacle, but "
            "no path points were found. Use addPathPoint() to add a path point "
            "to the path.");

    // Add the obstacle to the last segment, creating a new segment if needed.
    addPathSegmentIfNeeded();
    auto& segment = updLastPathSegment();
    segment.append_obstacles(Scholz2015GeometryPathObstacle());
    auto& ob = segment.upd_obstacles(segment.getNumObstacles() - 1);
    ob.setName("path_obstacle_" + std::to_string(segment.getNumObstacles() - 1));
    ob.connectSocket_contact_geometry(contactGeometry);
    ob.set_contact_hint(contactHint);
}

int Scholz2015GeometryPath::getNumPathPoints() const {
    if (getProperty_segments().empty()) {
        return 0;
    }

    int numPathPoints = getProperty_segments().size();
    numPathPoints += pathIsClosed() ? 1 : 0;
    return numPathPoints;
}

int Scholz2015GeometryPath::getNumObstacles() const {
    int numObstacles = 0;
    for (int i = 0; i < getProperty_segments().size(); ++i) {
        numObstacles += get_segments(i).getNumObstacles();
    }
    return numObstacles;
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

// //=============================================================================
// // CURVE SEGMENT COMPUTATIONS
// //=============================================================================
// bool Scholz2015GeometryPath::isInContactWithObstacle(
//         const SimTK::State& state, SimTK::CableSpanObstacleIndex ix) const {
//     return getCableSpan().isInContactWithObstacle(state, ix);
// }

// SimTK::Transform Scholz2015GeometryPath::calcCurveSegmentInitialFrenetFrame(
//         const SimTK::State& state, SimTK::CableSpanObstacleIndex ix) const {
//     return getCableSpan().calcCurveSegmentInitialFrenetFrame(state, ix);
// }

// SimTK::Transform Scholz2015GeometryPath::calcCurveSegmentFinalFrenetFrame(
//         const SimTK::State& state, SimTK::CableSpanObstacleIndex ix) const {
//     return getCableSpan().calcCurveSegmentFinalFrenetFrame(state, ix);
// }

// SimTK::Real Scholz2015GeometryPath::calcCurveSegmentArcLength(
//         const SimTK::State& state, SimTK::CableSpanObstacleIndex ix) const {
//     return getCableSpan().calcCurveSegmentArcLength(state, ix);
// }

// //=============================================================================
// // VIA POINT COMPUTATIONS
// //=============================================================================
// SimTK::Vec3 Scholz2015GeometryPath::calcViaPointLocation(
//         const SimTK::State& state, SimTK::CableSpanViaPointIndex ix) const {
//     return getCableSpan().calcViaPointLocation(state, ix);
// }

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
    for (const auto& [ix, segIx, obsIx] : _obstacleIndexes) {
        if (!cable.isInContactWithObstacle(state, ix)) {
            continue;
        }

        cable.calcCurveSegmentUnitForce(state, ix, unitBodyForce);
        const auto& segment = get_segments(segIx);
        const auto& obstacle = segment.get_obstacles(obsIx);
        const auto& frame = obstacle.getContactGeometry().getFrame();
        forceConsumer.consumeBodySpatialVec(state, frame,
                tension * unitBodyForce);
    }

    // Forces applied to each via point.
    for (const auto& [ix, segIx] : _viaPointIndexes) {
        cable.calcViaPointUnitForce(state, ix, unitBodyForce);
        const auto& segment = get_segments(segIx);
        const auto& frame = segment.getTermination().getParentFrame();
        forceConsumer.consumeBodySpatialVec(state, frame,
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

    if (get_algorithm() == "Scholz2015") {
        _algorithm = SimTK::CableSpanAlgorithm::Scholz2015;
    } else if (get_algorithm() == "MinimumLength") {
        _algorithm = SimTK::CableSpanAlgorithm::MinimumLength;
    } else {
        OPENSIM_THROW_FRMOBJ(Exception,
            fmt::format("Property 'algorithm' has invalid value {}; expected "
                        "one of the following: MinimumLength, Scholz2015",
                        get_algorithm()));
    }
}

void Scholz2015GeometryPath::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    OPENSIM_THROW_IF_FRMOBJ(getNumPathPoints() < 2, Exception,
            "Expected at least two path points before finalizing the path, but "
            "{} path point(s) were found. Use addPathPoint() to add a path "
            "point to the path.", getNumPathPoints());
    OPENSIM_THROW_IF_FRMOBJ(!pathIsClosed(),
            Exception, "Expected the final element of the path to be a path "
            "point, but it is an obstacle. Use addPathPoint() to add a path "
            "point to the path after the last obstacle.");
    OPENSIM_ASSERT_ALWAYS(getProperty_segments().size() > 0);
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
            _obstacleIndexes.emplace_back(std::make_tuple(ix, iseg, iobs));
        }

        // Add the via point if this is not the last segment.
        if (iseg < getProperty_segments().size() - 1) {
            SimTK::CableSpanViaPointIndex ix = cable.addViaPoint(
                segment.getTermination().getParentFrame().getMobilizedBodyIndex(),
                segment.getTermination().get_location());
            _viaPointIndexes.emplace_back(std::make_pair(ix, iseg));
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
    constructProperty_segments();
    constructProperty_algorithm("Scholz2015");
    constructProperty_solver_max_iterations(50);
    constructProperty_curve_segment_accuracy(1e-9);
    constructProperty_smoothness_tolerance(0.1 / 180. * SimTK::Pi);
}

bool Scholz2015GeometryPath::isStationFrameConnected(
        const Station& station) const {
    return station.getSocket<PhysicalFrame>("parent_frame").isConnected();
}

bool Scholz2015GeometryPath::pathHasOrigin() const {
    return isStationFrameConnected(getOrigin());
}

bool Scholz2015GeometryPath::pathIsClosed() const {
    return isStationFrameConnected(getLastPathSegment().getTermination());
}

const Scholz2015GeometryPathSegment&
Scholz2015GeometryPath::getLastPathSegment() const {
    OPENSIM_THROW_IF_FRMOBJ(getProperty_segments().empty(), Exception,
            "Expected at least one path segment, but none were found.");
    return get_segments(getProperty_segments().size() - 1);
}

Scholz2015GeometryPathSegment&
Scholz2015GeometryPath::updLastPathSegment() {
    OPENSIM_THROW_IF_FRMOBJ(getProperty_segments().empty(), Exception,
            "Expected at least one path segment, but none were found.");
    return upd_segments(getProperty_segments().size() - 1);
}

void Scholz2015GeometryPath::addPathSegmentIfNeeded() {
    // Only add a new segment if the last segment already has a termination
    // station connected.
    if (pathIsClosed()) {
        append_segments(Scholz2015GeometryPathSegment());
        auto& segment = upd_segments(getProperty_segments().size() - 1);
        segment.setName(fmt::format("path_segment_{}",
                getProperty_segments().size() - 1));
    }
}

const SimTK::CableSpan& Scholz2015GeometryPath::getCableSpan() const {
    return getModel().getMultibodySystem().getCableSubsystem().getCable(_index);
}

SimTK::CableSpan& Scholz2015GeometryPath::updCableSpan() {
    return getModel().updMultibodySystem().updCableSubsystem().updCable(_index);
}
