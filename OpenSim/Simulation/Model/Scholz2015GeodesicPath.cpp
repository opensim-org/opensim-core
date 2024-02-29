/* -------------------------------------------------------------------------- *
 *                     OpenSim: Scholz2015GeodesicPath.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco, Pepijn van den Bos, Andreas Scholz             *
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

#include "Scholz2015GeodesicPath.h"

using namespace OpenSim;

//=============================================================================
// GEODESIC PATH SEGMENT
//=============================================================================
GeodesicPathSegment::GeodesicPathSegment() {
    constructProperties();
}

//GeodesicPathSegment::GeodesicPathSegment(const std::string& name,
//        const PhysicalFrame& originFrame,
//        const SimTK::Vec3& locationInOriginFrame,
//        const PhysicalFrame& insertionFrame,
//        const SimTK::Vec3& locationInInsertionFrame) : GeodesicPathSegment() {
//    OPENSIM_THROW_IF(name.empty(), ComponentHasNoName, getClassName());
//    setName(name);
//
//    Station origin(originFrame, locationInOriginFrame);
//    origin.setName("origin");
//    set_origin(origin);
//
//    Station insertion(insertionFrame, locationInInsertionFrame);
//    insertion.setName("insertion");
//    set_insertion(insertion);
//
//    // 'finalizeFromProperties' recognizes the origin and insertion Stations
//    // as the GeodesicPathSegment's subcomponents.
//    finalizeFromProperties();
//
//    // When the Stations are constructed they are unaware that this
//    // GeodesicPathSegment contains them as subcomponents and the path name
//    // associated with them will not be valid. This a temporary fix to set the
//    // name once the added frames have been included as subcomponents which
//    // occurs during finalizeFromProperties() above.
//    static_cast<Station&>(upd_origin()).setParentFrame(originFrame);
//    static_cast<Station&>(upd_insertion()).setParentFrame(insertionFrame);
//
//    connectSocket_origin(origin);
//    connectSocket_insertion(insertion);
//}

GeodesicPathSegment::~GeodesicPathSegment() = default;

GeodesicPathSegment::GeodesicPathSegment(const GeodesicPathSegment&) = default;

GeodesicPathSegment& GeodesicPathSegment::operator=(
        const GeodesicPathSegment&) = default;

GeodesicPathSegment::GeodesicPathSegment(
        GeodesicPathSegment&&) noexcept = default;

GeodesicPathSegment& GeodesicPathSegment::operator=(
        GeodesicPathSegment&&) noexcept = default;

//=============================================================================
// GET AND SET
//=============================================================================
void GeodesicPathSegment::addWrapObject(
        const OpenSim::GeodesicWrapSurface& surface,
        const GeodesicInitialConditions& initialConditions) {
    appendSocketConnectee_surfaces(surface);
    append_initial_conditions(initialConditions);
}

double GeodesicPathSegment::getLength(const SimTK::State& s) const {
    calcWrappingPath(s);
    const auto& result = getCacheVariableValue<GeodesicWrapResult>(s, _resultCV);
    return result.length;
}

double GeodesicPathSegment::getLengtheningSpeed(const SimTK::State& s) const {
    calcWrappingPath(s);
    const auto& result = getCacheVariableValue<GeodesicWrapResult>(s, _resultCV);
    return result.lengtheningSpeed;
}

void GeodesicPathSegment::addInEquivalentForces(const SimTK::State& s,
        const double& tension,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
        SimTK::Vector& mobilityForces) const {
    calcWrappingPath(s);
    const auto& result = getCacheVariableValue<GeodesicWrapResult>(s, _resultCV);

    const Station& origin = getSocket<Station>("origin").getConnectee();
    const Station& insertion = getSocket<Station>("insertion").getConnectee();
    const PhysicalFrame& originFrame = origin.getParentFrame();
    const PhysicalFrame& insertionFrame = insertion.getParentFrame();

    const GeodesicBoundaryFrame& startFrame = result.startFrame;
    const GeodesicBoundaryFrame& endFrame = result.endFrame;

    // TODO add in body and mobility forces
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void GeodesicPathSegment::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    _resultCV = addCacheVariable<GeodesicWrapResult>(
            "result", GeodesicWrapResult(), SimTK::Stage::Position);
}

void GeodesicPathSegment::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    // Loop through all surfaces to create GeodesicWrapObjects to add to this
    // path segment
    std::vector<Geodesic> initialGeodesicsVec;
    _wrapObjects.clear();
    const auto& surfaces = getSocket<GeodesicWrapSurface>("surfaces");
    for (int i = 0; i < surfaces.getNumConnectees(); ++i) {
        const GeodesicWrapSurface& surface = surfaces.getConnectee(i);
        std::unique_ptr<GeodesicWrapObject> wrapObject =
                surface.generateGeodesicWrapObject();
        _wrapObjects.push_back(std::move(wrapObject));
    }
}

//=============================================================================
// CONVENIENCE METHODS
//=============================================================================
void GeodesicPathSegment::calcWrappingPath(const SimTK::State& state) const {
    if (isCacheVariableValid(state, _resultCV)) {
        return;
    }

    const Station& origin = getSocket<Station>("origin").getConnectee();
    const Station& insertion = getSocket<Station>("insertion").getConnectee();
    const SimTK::Vec3& originPoint = origin.getLocationInGround(state);
    const SimTK::Vec3& insertionPoint = insertion.getLocationInGround(state);
    // TODO move these settings somewhere else
    size_t maxIter = 20;
    double eps = 1e-13;

    // Set the transform for each wrap object.
    const auto& surfaces = getSocket<GeodesicWrapSurface>("surfaces");
    for (int i = 0; i < surfaces.getNumConnectees(); ++i) {
        const GeodesicWrapSurface& surface = surfaces.getConnectee(i);
        const PhysicalFrame& frame =
                surface.getSocket<PhysicalFrame>("frame").getConnectee();
        const SimTK::Transform& transform = frame.getTransformInGround(state);
        _wrapObjects[i]->setTransform(transform);
        _wrapObjects[i]->setInitialConditions(get_initial_conditions(i));
    }

    // Call the wrap solver.
    GeodesicWrapResult result;
    // _solver.calcWrappingPath(state, originPoint, insertionPoint, _wrapObjects,
    //         maxIter, eps, initialGeodesicsVec);

    // Update the cache.
    setCacheVariableValue<GeodesicWrapResult>(state, _resultCV, result);
}

void GeodesicPathSegment::constructProperties() {
    constructProperty_initial_conditions();
    constructProperty_origin();
    constructProperty_insertion(Station());
}

//=============================================================================
// SCHOLZ2015 GEODESIC PATH
//=============================================================================
Scholz2015GeodesicPath::Scholz2015GeodesicPath() : AbstractGeometryPath() {
    setAuthors("Nicholas Bianco, Pepijn van den Bos, Andreas Scholz");
    constructProperty_path_segments();
}

Scholz2015GeodesicPath::~Scholz2015GeodesicPath() = default;

Scholz2015GeodesicPath::Scholz2015GeodesicPath(
        const Scholz2015GeodesicPath&) = default;

Scholz2015GeodesicPath& Scholz2015GeodesicPath::operator=(
        const Scholz2015GeodesicPath&) = default;

Scholz2015GeodesicPath::Scholz2015GeodesicPath(
        Scholz2015GeodesicPath&&) noexcept = default;

Scholz2015GeodesicPath& Scholz2015GeodesicPath::operator=(
        Scholz2015GeodesicPath&&) noexcept = default;

//=============================================================================
// GET AND SET
//=============================================================================
GeodesicPathSegment* Scholz2015GeodesicPath::addPathSegment(
        const std::string& name,
        const PhysicalFrame& originFrame,
        const SimTK::Vec3& locationInOriginFrame,
        const PhysicalFrame& insertionFrame,
        const SimTK::Vec3& locationInInsertionFrame) {

    OPENSIM_THROW_IF_FRMOBJ(!getProperty_path_segments().empty(), Exception,
            "First path segment already added.");

    auto* segment = new GeodesicPathSegment();
    segment->setName(name);

    Station origin(originFrame, locationInOriginFrame);
    origin.setName("origin");
    segment->set_origin(origin);

    Station insertion(insertionFrame, locationInInsertionFrame);
    insertion.setName("insertion");
    segment->set_insertion(insertion);

    // 'finalizeFromProperties' recognizes the origin and insertion Stations
    // as the GeodesicPathSegment's subcomponents.
    segment->finalizeFromProperties();

    // When the Stations are constructed they are unaware that this
    // GeodesicPathSegment contains them as subcomponents and the path name
    // associated with them will not be valid. This a temporary fix to set the
    // name once the added frames have been included as subcomponents which
    // occurs during finalizeFromProperties() above.
    static_cast<Station&>(segment->upd_origin()).setParentFrame(originFrame);
    static_cast<Station&>(segment->upd_insertion()).setParentFrame(insertionFrame);

    segment->connectSocket_origin(segment->upd_origin());
    segment->connectSocket_insertion(segment->upd_insertion());

    addPathSegment(segment);

    return segment;
}

GeodesicPathSegment* Scholz2015GeodesicPath::addPathSegment(
        const std::string& name,
        const PhysicalFrame& insertionFrame,
        const SimTK::Vec3& locationInInsertionFrame) {

    OPENSIM_THROW_IF_FRMOBJ(getProperty_path_segments().empty(), Exception,
            "First path segment not yet added.");

    auto* segment = new GeodesicPathSegment();
    segment->setName(name);

    Station insertion(insertionFrame, locationInInsertionFrame);
    insertion.setName("insertion");
    segment->set_insertion(insertion);

    // 'finalizeFromProperties' recognizes the insertion Station
    // as the GeodesicPathSegment's subcomponent.
    segment->finalizeFromProperties();

    // When the Station is constructed it is unaware that this
    // GeodesicPathSegment contains it as a subcomponent and the path name
    // associated with it will not be valid. This a temporary fix to set the
    // the name once the added frame has been included as a subcomponent which
    // occurs during finalizeFromProperties() above.
    static_cast<Station&>(segment->upd_insertion()).setParentFrame(insertionFrame);

    // Use previous segment's insertion as this segment's origin
    const auto& prevSegment = get_path_segments(getProperty_path_segments().size() - 1);
    segment->connectSocket_origin(prevSegment.get_insertion());
    segment->connectSocket_insertion(segment->upd_insertion());

    addPathSegment(segment);

    return segment;
}

void Scholz2015GeodesicPath::addPathSegment(GeodesicPathSegment* segment) {
    OPENSIM_THROW_IF(isComponentInOwnershipTree(segment),
            ComponentAlreadyPartOfOwnershipTree,
            segment->getName(), getName());
    updProperty_path_segments().adoptAndAppendValue(segment);
    finalizeFromProperties();
    std::cout << "DEBUG" << std::endl;
    prependComponentPathToConnecteePath(*segment);
}

//=============================================================================
// ABSTRACT GEOMETRY PATH INTERFACE
//=============================================================================
bool Scholz2015GeodesicPath::isVisualPath() const {
    return true;
}

double Scholz2015GeodesicPath::getLength(const SimTK::State &s) const {
    computeLength(s);
    return getCacheVariableValue<double>(s, _lengthCV);
}

double Scholz2015GeodesicPath::getLengtheningSpeed(const SimTK::State &s) const {
    computeLengtheningSpeed(s);
    return getCacheVariableValue<double>(s, _lengtheningSpeedCV);
}

void Scholz2015GeodesicPath::addInEquivalentForces(const SimTK::State &state,
        const double &tension,
        SimTK::Vector_<SimTK::SpatialVec> &bodyForces,
        SimTK::Vector &mobilityForces) const {

    for (int i = 0; i < getProperty_path_segments().size(); ++i) {
        const auto& segment = get_path_segments(i);

        segment.addInEquivalentForces(state, tension, bodyForces,
                mobilityForces);
    }
}

double Scholz2015GeodesicPath::computeMomentArm(const SimTK::State &s,
        const Coordinate &aCoord) const {
    // TODO call MomentArmSolver or use expression based on Andreas' work
    return 0.0;
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void Scholz2015GeodesicPath::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();


}

void Scholz2015GeodesicPath::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);
    // Check the properties.
    OPENSIM_THROW_IF_FRMOBJ(getProperty_path_segments().empty(), Exception,
            "Expected at least one path segment, but none were provided.");


}

void Scholz2015GeodesicPath::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    _lengthCV = addCacheVariable<double>("length", 0.0, SimTK::Stage::Position);
    _lengtheningSpeedCV = addCacheVariable<double>("lengthening_speed", 0.0,
            SimTK::Stage::Velocity);
}

//=============================================================================
// CONVENIENCE METHODS
//=============================================================================
void Scholz2015GeodesicPath::computeLength(const SimTK::State &s) const {
    double length = 0.0;
    for (const auto& segment : getComponentList<GeodesicPathSegment>()) {
        length += segment.getLength(s);
    }
    setCacheVariableValue(s, _lengthCV, length);
}

void Scholz2015GeodesicPath::computeLengtheningSpeed(
        const SimTK::State &s) const {
    double lengtheningSpeed = 0.0;
    for (const auto& segment : getComponentList<GeodesicPathSegment>()) {
        lengtheningSpeed += segment.getLengtheningSpeed(s);
    }
    setCacheVariableValue(s, _lengtheningSpeedCV, lengtheningSpeed);
}