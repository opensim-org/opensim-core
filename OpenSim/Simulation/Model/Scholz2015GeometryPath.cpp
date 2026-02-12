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

#include <optional>

using namespace OpenSim;

//=============================================================================
// SCHOLZ 2015 GEOMETRY PATH POINT
//=============================================================================
Scholz2015GeometryPathPoint::Scholz2015GeometryPathPoint() {
    constructProperty_PathPoint(PathPoint());
}

const PathPoint& Scholz2015GeometryPathPoint::getPathPoint() const {
    return get_PathPoint();
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
void Scholz2015GeometryPath::appendPathPoint(const PhysicalFrame& frame,
        const SimTK::Vec3& location) {

    // Construct a new path point.
    append_path_elements(Scholz2015GeometryPathPoint());
    auto& point = updElement<Scholz2015GeometryPathPoint>(
            getProperty_path_elements().size() - 1);
    point.setName("path_point_" + std::to_string(getNumPathPoints() - 1));

    // Set the path point's PathPoint.
    PathPoint& pathPoint = point.upd_PathPoint();
    pathPoint.set_location(location);
    pathPoint.setParentFrame(frame);

    // Call finalizeFromProperties() to ensure that the path point is recognized
    // as a subcomponent.
    finalizeFromProperties();
}

const PathPoint& Scholz2015GeometryPath::getOrigin() const {
    OPENSIM_THROW_IF_FRMOBJ(getNumPathPoints() == 0, Exception,
            "Tried retrieving the origin point, but the path contains no path "
            "points.");

    const auto* point = tryGetElement<Scholz2015GeometryPathPoint>(0);
    OPENSIM_THROW_IF_FRMOBJ(point == nullptr, Exception,
            "Tried retrieving the origin point, but the first element in the "
            "path is not a path point.");

    return point->getPathPoint();
}

const PathPoint& Scholz2015GeometryPath::getInsertion() const {
    OPENSIM_THROW_IF_FRMOBJ(getNumPathPoints() < 2, Exception,
            "Tried retrieving the insertion point, but the path contains less "
            "than two path points.");

    const auto* point = tryGetElement<Scholz2015GeometryPathPoint>(
            getProperty_path_elements().size() - 1);
    OPENSIM_THROW_IF_FRMOBJ(point == nullptr, Exception,
            "Tried retrieving the insertion point, but the last element in the "
            "path is not a path point.");

    return point->getPathPoint();
}

const PathPoint& Scholz2015GeometryPath::getPathPoint(
        int pathPointIndex) const {
    OPENSIM_THROW_IF_FRMOBJ(
            pathPointIndex < 0 || pathPointIndex >= getNumPathPoints(),
            Exception,
            "Index {} is out of range. There are {} path points in the path.",
            pathPointIndex, getNumPathPoints());

    int count = 0;
    for (int i = 0; i < getNumPathElements(); ++i) {
        if (const auto* point = tryGetElement<Scholz2015GeometryPathPoint>(i)) {
            if (count == pathPointIndex) {
                return point->getPathPoint();
            }
            ++count;
        }
    }

    // We should never reach this point.
    OPENSIM_THROW_FRMOBJ(Exception,
            "Path point index {} could not be found.", pathPointIndex);
}

void Scholz2015GeometryPath::appendObstacle(
        const ContactGeometry& contactGeometry,
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

const ContactGeometry& Scholz2015GeometryPath::getContactGeometry(
        int obstacleIndex) const {
    const Scholz2015GeometryPathObstacle* obstacle = getObstacle(obstacleIndex);
    return obstacle->getContactGeometry();
}

const SimTK::Vec3& Scholz2015GeometryPath::getContactHint(
        int obstacleIndex) const {
    const Scholz2015GeometryPathObstacle* obstacle = getObstacle(obstacleIndex);
    return obstacle->getContactHint();
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
    const PathPoint& origin = getOrigin();
    const PathPoint& insertion = getInsertion();
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
        const auto& frame = point.getPathPoint().getParentFrame();
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

    const PathPoint& origin = getOrigin();
    const PathPoint& insertion = getInsertion();
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
                point->getPathPoint().getParentFrame().getMobilizedBodyIndex(),
                point->getPathPoint().get_location());
            _viaPointIndexes.emplace_back(std::make_pair(ix, ielt));
        } else if (const auto* obstacle =
                    tryGetElement<Scholz2015GeometryPathObstacle>(ielt)) {
            SimTK::CableSpanObstacleIndex ix = cable.addObstacle(
                obstacle->getContactGeometry().getFrame()
                                              .getMobilizedBodyIndex(),
                obstacle->getContactGeometry().getTransform(),
                std::make_shared<SimTK::ContactGeometry>(
                    obstacle->getContactGeometry().createSimTKContactGeometry()),
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

void Scholz2015GeometryPath::generateDecorations(
        bool fixed,
        const ModelDisplayHints& hints,
        const SimTK::State& s,
        SimTK::Array_<SimTK::DecorativeGeometry>& geoms) const {

    if (fixed) { return; }
    const bool showPathPoints = hints.get_show_path_points();
    const SimTK::Vec3 color = getColor(s);
    int index = 0;
    std::optional<SimTK::Vec3> previous;
    getCableSpan().calcDecorativePathPoints(s, [&](SimTK::Vec3 x_G) {
        if (previous) {
            // Emit line between points
            geoms.push_back(SimTK::DecorativeLine(*previous, x_G)
                .setLineThickness(4)
                .setScaleFactors(SimTK::Vec3{1.0})
                .setColor(color)
                .setBodyId(0)
                .setIndexOnBody(index++)
            );
        }
        if (showPathPoints) {
            geoms.push_back(SimTK::DecorativeSphere(0.005)
                .setTransform(x_G)
                .setScaleFactors(SimTK::Vec3{1.0})
                .setColor(color)
                .setBodyId(0)
                .setIndexOnBody(index++)
            );
        }

        previous = x_G;
    });
}

//=============================================================================
// CONVENIENCE METHODS
//=============================================================================
void Scholz2015GeometryPath::constructProperties() {
    constructProperty_path_elements();
}

const Scholz2015GeometryPathObstacle* Scholz2015GeometryPath::getObstacle(
        int obstacleIndex) const {
    OPENSIM_THROW_IF_FRMOBJ(
            obstacleIndex < 0 || obstacleIndex >= getNumObstacles(), Exception,
            "Index {} is out of range. There are {} obstacles in the path.",
            obstacleIndex, getNumObstacles());

    int count = 0;
    for (int i = 0; i < getNumPathElements(); ++i) {
        if (const auto* obstacle =
                tryGetElement<Scholz2015GeometryPathObstacle>(i)) {
            if (count == obstacleIndex) {
                return obstacle;
            }
            ++count;
        }
    }

    // We should never reach this point.
    OPENSIM_THROW_FRMOBJ(Exception,
            "Obstacle index {} could not be found.", obstacleIndex);
}

const SimTK::CableSpan& Scholz2015GeometryPath::getCableSpan() const {
    return getModel().getMultibodySystem().getCableSubsystem().getCable(_index);
}

SimTK::CableSpan& Scholz2015GeometryPath::updCableSpan() {
    return getModel().updMultibodySystem().updCableSubsystem().updCable(_index);
}
