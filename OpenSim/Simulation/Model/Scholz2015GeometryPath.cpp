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
        ModelComponent() { }

Scholz2015GeometryPathSegment::Scholz2015GeometryPathSegment(
        const std::string& name, const Station& origin, 
        const Station& insertion) : Scholz2015GeometryPathSegment() {
    setName(name);
    connectSocket_origin(origin);
    connectSocket_insertion(insertion);
}

const Station& Scholz2015GeometryPathSegment::getOrigin() const {
    return getSocket<Station>("origin").getConnectee();
}

const Station& Scholz2015GeometryPathSegment::getInsertion() const {
    return getSocket<Station>("insertion").getConnectee();
}

SimTK::Real Scholz2015GeometryPathSegment::getLength(
        const SimTK::State& s) const {
    return getCableSpan().calcLength(s);
}

SimTK::Real Scholz2015GeometryPathSegment::getLengtheningSpeed(
        const SimTK::State& s) const {
    return getCableSpan().calcLengthDot(s);
}

void Scholz2015GeometryPathSegment::applyBodyForces(
        const SimTK::State& state,
        SimTK::Real tension,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces) const {
    getCableSpan().applyBodyForces(state, tension, bodyForces);
}

void Scholz2015GeometryPathSegment::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);

    const Station& origin = getOrigin();
    const Station& insertion = getInsertion();

    SimTK::CableSubsystem& cables = system.updCableSubsystem();
    SimTK::CableSpan cable(
        cables,
        origin.getParentFrame().getMobilizedBodyIndex(),
        origin.get_location(),
        insertion.getParentFrame().getMobilizedBodyIndex(),
        insertion.get_location());

    _index = cable.getIndex();
}

const SimTK::CableSpan& Scholz2015GeometryPathSegment::getCableSpan() const {
    return getModel().getMultibodySystem().getCableSubsystem().getCable(_index);
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

    append_path_segments(Scholz2015GeometryPathSegment(name, origin, insertion));
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
    append_path_segments(Scholz2015GeometryPathSegment(name, origin, insertion));
    _segmentNameToIndexMap[name] = numSegments;
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
    // TODO: Only valid when u = qdot.
    SimTK::State state = s;
    state.updQ() = s.getQ();
    state.updU() = SimTK::Vector(s.getNU(), 0.0);
    coord.setSpeedValue(state, 1.0);
    return getLengtheningSpeed(state);
}

void Scholz2015GeometryPath::produceForces(const SimTK::State& state,
        double tension, ForceConsumer& forceConsumer) const{
    for (int i = 0; i < getProperty_path_segments().size(); ++i) {
        const Station& origin = get_path_segments(i).getOrigin();
        const Station& insertion = get_path_segments(i).getInsertion();
        
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
