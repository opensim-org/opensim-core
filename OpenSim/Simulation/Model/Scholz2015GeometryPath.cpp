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
// SCHOLZ 2015 GEOMETRY PATH POINT
//=============================================================================
Scholz2015GeometryPathPoint::Scholz2015GeometryPathPoint() {
    constructProperty_station(Station());
}

const Station& Scholz2015GeometryPathPoint::getStation() const {
    return get_station();
}

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

const SimTK::Vec3& Scholz2015GeometryPathObstacle::getContactHint() const {
    return get_contact_hint();
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

    // Construct a new path point.
    append_path_elements(Scholz2015GeometryPathPoint());
    auto& point = updElement<Scholz2015GeometryPathPoint>(
            getProperty_path_elements().size() - 1);
    point.setName("path_point_" + std::to_string(getNumPathPoints() - 1));

    // Set the path point's Station.
    Station& station = point.upd_station();
    station.set_location(location);
    station.setParentFrame(frame);

    // Call finalizeFromProperties() to ensure that the station is recognized
    // as a subcomponent.
    finalizeFromProperties();
}

const Station& Scholz2015GeometryPath::getOrigin() const {
    const auto* point = tryGetElement<Scholz2015GeometryPathPoint>(0);
    OPENSIM_THROW_IF_FRMOBJ(point == nullptr, Exception,
            "Tried retrieving the origin point, but the first element in the "
            "path is not a path point.");

    return point->getStation();
}

const Station& Scholz2015GeometryPath::getInsertion() const {
    const auto* point = tryGetElement<Scholz2015GeometryPathPoint>(
            getProperty_path_elements().size() - 1);
    OPENSIM_THROW_IF_FRMOBJ(point == nullptr, Exception,
            "Tried retrieving the insertion point, but the last element in the "
            "path is not a path point.");

    return point->getStation();
}

void Scholz2015GeometryPath::addObstacle(const ContactGeometry& contactGeometry,
        const SimTK::Vec3& contactHint) {

    // Construct a new obstacle.
    append_path_elements(Scholz2015GeometryPathObstacle());
    auto& obstacle = updElement<Scholz2015GeometryPathObstacle>(
            getProperty_path_elements().size() - 1);
    obstacle.setName("obstacle_" + std::to_string(getNumObstacles() - 1));

    // Set the obstacle's ContactGeometry and contact hint.
    obstacle.connectSocket_contact_geometry(contactGeometry);
    obstacle.set_contact_hint(contactHint);
}

int Scholz2015GeometryPath::getNumPathPoints() const {
    int count = 0;
    for (int i = 0; i < getNumPathElements(); ++i) {
        if (tryGetElement<Scholz2015GeometryPathPoint>(i) != nullptr) {
            ++count;
        }
    }
    return count;
}

int Scholz2015GeometryPath::getNumObstacles() const {
    int count = 0;
    for (int i = 0; i < getNumPathElements(); ++i) {
        if (tryGetElement<Scholz2015GeometryPathObstacle>(i) != nullptr) {
            ++count;
        }
    }
    return count;
}

int Scholz2015GeometryPath::getNumPathElements() const {
    return getProperty_path_elements().size();
}

//=============================================================================
// ABSTRACT PATH INTERFACE
//=============================================================================
double Scholz2015GeometryPath::getLength(const SimTK::State& s) const {
    return getCableSpan().calcLength(s);
}

double Scholz2015GeometryPath::getLengtheningSpeed(
        const SimTK::State& s) const {
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
    for (const auto& [ix, eltIx] : _obstacleIndexes) {
        if (!cable.isInContactWithObstacle(state, ix)) {
            continue;
        }

        cable.calcCurveSegmentUnitForce(state, ix, unitBodyForce);
        const auto& obstacle =
                getElement<Scholz2015GeometryPathObstacle>(eltIx);
        const auto& frame = obstacle.getContactGeometry().getFrame();
        forceConsumer.consumeBodySpatialVec(state, frame,
                tension * unitBodyForce);
    }

    // Forces applied to each via point.
    for (const auto& [ix, eltIx] : _viaPointIndexes) {
        cable.calcViaPointUnitForce(state, ix, unitBodyForce);
        const auto& point = getElement<Scholz2015GeometryPathPoint>(eltIx);
        const auto& frame = point.getStation().getParentFrame();
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
void Scholz2015GeometryPath::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    OPENSIM_THROW_IF_FRMOBJ(getNumPathPoints() < 2, Exception,
            "Expected at least two path points before finalizing the path, but "
            "{} path point(s) were found. Use addPathPoint() to add a path "
            "point to the path.", getNumPathPoints());
    OPENSIM_THROW_IF_FRMOBJ(
            tryGetElement<Scholz2015GeometryPathPoint>(0) == nullptr,
            Exception, "Expected the first element of the path to be a path "
            "point, but it is an obstacle.");
    OPENSIM_THROW_IF_FRMOBJ(
            tryGetElement<Scholz2015GeometryPathPoint>(
                    getNumPathElements() - 1) == nullptr,
            Exception, "Expected the last element of the path to be a path "
            "point, but it is an obstacle.");
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

    for (int ielt = 1; ielt < getProperty_path_elements().size() - 1; ++ielt) {
        if (const auto* point =
                    tryGetElement<Scholz2015GeometryPathPoint>(ielt)) {
            SimTK::CableSpanViaPointIndex ix = cable.addViaPoint(
                point->getStation().getParentFrame().getMobilizedBodyIndex(),
                point->getStation().get_location());
            _viaPointIndexes.emplace_back(std::make_pair(ix, ielt));
        } else if (const auto* obstacle =
                    tryGetElement<Scholz2015GeometryPathObstacle>(ielt)) {
            SimTK::CableSpanObstacleIndex ix = cable.addObstacle(
                obstacle->getContactGeometry().getFrame()
                                              .getMobilizedBodyIndex(),
                obstacle->getContactGeometry().getTransform(),
                obstacle->getContactGeometry().getSimTKContactGeometryPtr(),
                obstacle->getContactHint());
            _obstacleIndexes.emplace_back(std::make_tuple(ix, ielt));
        }
    }

    cable.setSmoothnessTolerance(1e-5);
    cable.setCurveSegmentAccuracy(1e-10);
    cable.setSolverMaxIterations(50);
    cable.setAlgorithm(SimTK::CableSpanAlgorithm::Scholz2015);
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
    constructProperty_path_elements();
    constructProperty_algorithm("Scholz2015");
}

const SimTK::CableSpan& Scholz2015GeometryPath::getCableSpan() const {
    return getModel().getMultibodySystem().getCableSubsystem().getCable(_index);
}

SimTK::CableSpan& Scholz2015GeometryPath::updCableSpan() {
    return getModel().updMultibodySystem().updCableSubsystem().updCable(_index);
}
