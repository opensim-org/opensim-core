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
 * Author(s): Andreas Scholz, Pepijn van den Bos                              *
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

#include "Scholz2015GeometryPath.h"

#include <OpenSim/Simulation/Model/ForceConsumer.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

//=============================================================================
// SCHOLZ 2015 GEOMETRY PATH SEGMENT
//=============================================================================
Scholz2015GeometryPathSegment::Scholz2015GeometryPathSegment() : 
        ModelComponent() {
    constructProperties();
}

void Scholz2015GeometryPathSegment::constructProperties() {
    constructProperty_contact_hints();
}

const Station& Scholz2015GeometryPathSegment::getOrigin() const {
    return getSocket<Station>("origin").getConnectee();
}

const Station& Scholz2015GeometryPathSegment::getInsertion() const {
    return getSocket<Station>("insertion").getConnectee();
}

int Scholz2015GeometryPathSegment::getNumObstacles() const {
    return getCableSpan().getNumObstacles();
}

const ContactGeometry& Scholz2015GeometryPathSegment::getObstacle(
        int index) const {
    // TODO: out of bounds check
    return getSocket<ContactGeometry>("obstacles").getConnectee(index);
}

SimTK::Real Scholz2015GeometryPathSegment::getLength(
        const SimTK::State& s) const {
    return getCableSpan().calcLength(s);
}

SimTK::Real Scholz2015GeometryPathSegment::getLengtheningSpeed(
        const SimTK::State& s) const {
    return getCableSpan().calcLengthDot(s);
}

void Scholz2015GeometryPathSegment::calcOriginUnitForce(
        const SimTK::State& state, SimTK::SpatialVec& unitForce_G) const {
    getCableSpan().calcOriginUnitForce(state, unitForce_G);
}   

void Scholz2015GeometryPathSegment::calcInsertionUnitForce(
        const SimTK::State& state, SimTK::SpatialVec& unitForce_G) const {
    getCableSpan().calcTerminationUnitForce(state, unitForce_G);
}

void Scholz2015GeometryPathSegment::calcCurveSegmentUnitForce(
        const SimTK::State& state, SimTK::CableSpanObstacleIndex ix,
        SimTK::SpatialVec& unitForce_G) const {
    getCableSpan().calcCurveSegmentUnitForce(state, ix, unitForce_G);
}

bool Scholz2015GeometryPathSegment::isInContactWithObstacle(
        const SimTK::State& state, SimTK::CableSpanObstacleIndex ix) const {
    return getCableSpan().isInContactWithObstacle(state, ix);
}

void Scholz2015GeometryPathSegment::extendAddToSystem(
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

    const auto& obstacles = getSocket<ContactGeometry>("obstacles");
    int numObstacles = obstacles.getNumConnectees();
    for (int i = 0; i < numObstacles; ++i) {
        const auto& obstacle = obstacles.getConnectee(i);
        const auto& contactGeometry = obstacle.createSimTKContactGeometry();
        cable.addObstacle(
            obstacle.getFrame().getMobilizedBodyIndex(),
            obstacle.getTransform(),
            std::shared_ptr<SimTK::ContactGeometry>(
                new SimTK::ContactGeometry(contactGeometry)),
            get_contact_hints(i)
        );
    }

    _index = cable.getIndex();
    cable.setAlgorithm(SimTK::CableSpanAlgorithm::MinimumLength);
}

const SimTK::CableSpan& Scholz2015GeometryPathSegment::getCableSpan() const {
    return getModel().getMultibodySystem().getCableSubsystem().getCable(_index);
}

void Scholz2015GeometryPathSegment::generateDecorations(
        bool fixed, const ModelDisplayHints& hints,
        const SimTK::State& s,
        SimTK::Array_<SimTK::DecorativeGeometry>& geoms) const {
    if (fixed) { return; }

    for (int i = 0; i < getNumObstacles(); ++i) {        
        SimTK::CableSpanObstacleIndex ix(i);
        if (!isInContactWithObstacle(s, ix)) {
            continue;
        }
        
        // Draw a fixed number of points per curve segment.
        const int numSamples = 4;
        getCableSpan().calcCurveSegmentResampledPoints(s, ix, numSamples,
            [&](SimTK::Vec3 x_G)
            {
                geoms.push_back(SimTK::DecorativeSphere(0.001).setTransform(x_G)
                        .setColor(SimTK::Blue));
            });

        // Draw the Frenet frames at the geodesic boundary points.
        geoms.push_back(SimTK::DecorativeFrame(0.1).setTransform(
            getCableSpan().calcCurveSegmentInitialFrenetFrame(s, ix)));
        geoms.push_back(SimTK::DecorativeFrame(0.1).setTransform(
            getCableSpan().calcCurveSegmentFinalFrenetFrame(s, ix)));
    }
}

//=============================================================================
// CONSTRUCTOR
//=============================================================================
Scholz2015GeometryPath::Scholz2015GeometryPath() : AbstractGeometryPath() {
    constructProperties();
}

//=============================================================================
// GET AND SET METHODS
//=============================================================================
void Scholz2015GeometryPath::createInitialPathSegment(const std::string& name, 
        const Station& origin, const Station& insertion) {

    OPENSIM_THROW_IF_FRMOBJ(getProperty_path_segments().size() > 0,
        Exception, "Attempted to create the initial path segment, but this path "
        "already has path segments. Use appendPathSegment() to add "
        "additional segments.");

    OPENSIM_THROW_IF_FRMOBJ(
        _segmentNameToIndexMap.find(name) != _segmentNameToIndexMap.end(), 
        Exception, "A path segment with the name '{}' already exists. Please "
        "choose a different name.", name);

    append_path_segments(Scholz2015GeometryPathSegment());
    auto& segment = upd_path_segments(getProperty_path_segments().size() - 1);
    segment.setName(name);
    segment.connectSocket_origin(origin);
    segment.connectSocket_insertion(insertion);
    _segmentNameToIndexMap[name] = 0;
}

void Scholz2015GeometryPath::appendPathSegment(const std::string& name,
        const Station& insertion) {

    OPENSIM_THROW_IF_FRMOBJ(
        _segmentNameToIndexMap.find(name) != _segmentNameToIndexMap.end(), 
        Exception, "A path segment with the name '{}' already exists. Please "
        "choose a different name.", name);

    int numSegments = getProperty_path_segments().size();
    const Station& origin = get_path_segments(numSegments - 1).getInsertion();
    append_path_segments(Scholz2015GeometryPathSegment());
    auto& segment = upd_path_segments(getProperty_path_segments().size() - 1);
    segment.setName(name);
    segment.connectSocket_origin(origin);
    segment.connectSocket_insertion(insertion);
    _segmentNameToIndexMap[name] = numSegments;
}

void Scholz2015GeometryPath::addObstacleToPathSegment(
        const std::string& segmentName, const ContactGeometry& obstacle,
        const SimTK::Vec3& contactHint) {

    OPENSIM_THROW_IF_FRMOBJ(
        _segmentNameToIndexMap.find(segmentName) == _segmentNameToIndexMap.end(), 
        Exception, "A path segment with the name '{}' does not exist. Please "
        "choose a different name.", segmentName);

    int segmentIndex = _segmentNameToIndexMap[segmentName];
    upd_path_segments(segmentIndex).appendSocketConnectee_obstacles(obstacle);
    upd_path_segments(segmentIndex).append_contact_hints(contactHint);
}

//=============================================================================
// ABSTRACT PATH INTERFACE
//=============================================================================
double Scholz2015GeometryPath::getLength(const SimTK::State& s) const {
    computeLength(s);
    return getCacheVariableValue<double>(s, _lengthCV);
}

double Scholz2015GeometryPath::computeMomentArm(const SimTK::State& s,
        const Coordinate& coord) const {

    if (!_maSolver)
        const_cast<Self*>(this)->_maSolver.reset(new MomentArmSolver(*_model));

    return _maSolver->solve(s, coord,  *this);
}

void Scholz2015GeometryPath::produceForces(const SimTK::State& state,
        double tension, ForceConsumer& forceConsumer) const {

    // This implementation mirrors SimTK::CableSpan::applyBodyForces().

    if (tension <= 0.) {
        return;
    }

    for (int i = 0; i < getProperty_path_segments().size(); ++i) {
        SimTK::SpatialVec unitBodyForce;
        const auto& segment = get_path_segments(i);
        const Station& origin = segment.getOrigin();
        const Station& insertion = segment.getInsertion();

        // Force applied at cable origin point.
        {
            segment.calcOriginUnitForce(state, unitBodyForce);
            forceConsumer.consumeBodySpatialVec(state, origin.getParentFrame(), 
                    unitBodyForce * tension);
        }

        // Forces applied to each obstacle body.
        for (int j = 0; j < segment.getNumObstacles(); ++j) {
            SimTK::CableSpanObstacleIndex ix(j);
            if (!segment.isInContactWithObstacle(state, ix)) {
                continue;
            }
            segment.calcCurveSegmentUnitForce(state, ix, unitBodyForce);
            forceConsumer.consumeBodySpatialVec(state, 
                    segment.getObstacle(j).getFrame(), 
                    unitBodyForce * tension);
        }

        // Force applied at cable termination point.
        {
            segment.calcInsertionUnitForce(state, unitBodyForce);
            forceConsumer.consumeBodySpatialVec(state, 
                    insertion.getParentFrame(), unitBodyForce * tension);
        }
    }
}

double Scholz2015GeometryPath::getLengtheningSpeed(const SimTK::State& s) const {
    computeSpeed(s);
    return getCacheVariableValue<double>(s, _lengtheningSpeedCV);
}

//=============================================================================
// CONVENIENCE METHODS
//=============================================================================
void Scholz2015GeometryPath::constructProperties() {
    constructProperty_path_segments();
}

//=============================================================================
// HELPER METHODS
//=============================================================================
void Scholz2015GeometryPath::computeLength(const SimTK::State& s) const {
    if (isCacheVariableValid(s, _lengthCV)) {
        return;
    }

    SimTK::Real length = 0.0;
    for (int i = 0; i < getProperty_path_segments().size(); ++i) {
        length += get_path_segments(i).getLength(s);
    }
    setCacheVariableValue(s, _lengthCV, length);
}

void Scholz2015GeometryPath::computeSpeed(const SimTK::State& s) const {
    if (isCacheVariableValid(s, _lengtheningSpeedCV)) {
        return;
    }

    SimTK::Real speed = 0.0;
    for (int i = 0; i < getProperty_path_segments().size(); ++i) {
        speed += get_path_segments(i).getLengtheningSpeed(s);
    }
    setCacheVariableValue(s, _lengtheningSpeedCV, speed);
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void Scholz2015GeometryPath::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();    
}

void Scholz2015GeometryPath::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);

    this->_lengthCV = addCacheVariable("length", 0.0, SimTK::Stage::Position);
    this->_lengtheningSpeedCV = addCacheVariable("speed", 0.0, 
            SimTK::Stage::Velocity);
}
