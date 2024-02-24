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
#include <OpenSim/Simulation/GeodesicWrapping/ImplicitSurfaceParameters.h>

using namespace OpenSim;

//=============================================================================
// GEODESIC PATH SEGMENT
//=============================================================================
GeodesicPathSegment::GeodesicPathSegment() {
    constructProperties();
}

GeodesicPathSegment::~GeodesicPathSegment() = default;

GeodesicPathSegment::GeodesicPathSegment(
        const GeodesicPathSegment& other) = default;

GeodesicPathSegment& GeodesicPathSegment::operator=(
        const GeodesicPathSegment& other) {
    if (this != &other) {
        _wrapObjects = other._wrapObjects;
        _solver = other._solver;
    }
    return *this;
}

GeodesicPathSegment::GeodesicPathSegment(GeodesicPathSegment&&) = default;

GeodesicPathSegment& GeodesicPathSegment::operator=(
        GeodesicPathSegment&&) = default;

//=============================================================================
// GET AND SET
//=============================================================================
void GeodesicPathSegment::addWrapObject(
        const OpenSim::GeodesicWrapSurface& surface,
        const SimTK::Vec3& initialPosition,
        const SimTK::Vec3& initialVelocity,
        SimTK::Real initialLength) {
    appendSocketConnectee_surfaces(surface);
    append_initial_conditions(GeodesicInitialConditions(
            initialPosition, initialVelocity, initialLength));
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

    // TODO compute body and mobility forces
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

void GeodesicPathSegment::extendRealizeTopology(SimTK::State& state) const {
    Super::extendRealizeTopology(state);
    const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
    m_discreteVarIndex = subSys.allocateDiscreteVariable(
            state, SimTK::Stage::Position, new Geodesics());
}

void GeodesicPathSegment::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    // Loop through all surfaces to create GeodesicWrapObjects to add to this
    // path segment
    _wrapObjects.clear();
    const auto& surfaces = getSocket<GeodesicWrapSurface>("surfaces");
    for (int i = 0; i < surfaces.getNumConnectees(); ++i) {
        const GeodesicWrapSurface& surface = surfaces.getConnectee(i);
        GeodesicWrapSurface::Form form = surface.getCurrentForm();
        const PhysicalFrame& frame =
                surface.getSocket<PhysicalFrame>("frame").getConnectee();
        SimTK::MobilizedBodyIndex mobodIndex = frame.getMobilizedBodyIndex();

        // TODO: use initial conditions to shoot an initial geodesic and then
        //       "connect" the dots to create an initial path for this segment
        GeodesicInitialConditions initialConditions = get_initial_conditions(i);

        if (form == GeodesicWrapSurface::Form::Implicit) {
            ImplicitSurfaceParameters surfaceParams(
                    surface.generateImplicitSurfaceParametersImpl());
            GeodesicWrapObject wrapObject(std::move(surfaceParams), mobodIndex);
            _wrapObjects.emplace_back(std::move(wrapObject));
        } else if (form == GeodesicWrapSurface::Form::Parametric) {
            // TODO
        } else if (form == GeodesicWrapSurface::Form::Analytic) {
            // TODO
        }
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

    GeodesicWrapResult result;
    // TODO
    // _solver.calcWrappingPath(state, originPoint, insertionPoint, _wrapObjects,
    //         result, maxIter, eps);
    setCacheVariableValue<GeodesicWrapResult>(state, _resultCV, result);
}

void GeodesicPathSegment::constructProperties() {
    constructProperty_initial_conditions();
}


//=============================================================================
// SCHOLZ2015 GEODESIC PATH
//=============================================================================
Scholz2015GeodesicPath::Scholz2015GeodesicPath() : AbstractGeometryPath() {
    setAuthors("Nicholas Bianco, Pepijn van den Bos, Andreas Scholz");
    constructProperties();
}

//=============================================================================
// GET AND SET
//=============================================================================
void Scholz2015GeodesicPath::addPathSegment(
        const std::vector<GeodesicWrapSurface>& surfaces,
        const std::vector<GeodesicInitialConditions>& initialConditions,
        const OpenSim::Station& origin, const OpenSim::Station& insertion) {
    OPENSIM_THROW_IF_FRMOBJ(!getProperty_path_segments().empty(), Exception,
            "The first path segment has already been set.");

    GeodesicPathSegment segment;
    for (int i = 0; i < surfaces.size(); ++i) {
        segment.addWrapObject(surfaces[i], initialConditions[i]);
    }
    segment.connectSocket_origin(origin);
    segment.connectSocket_insertion(insertion);
    append_path_segments(segment);
}

void Scholz2015GeodesicPath::addPathSegment(
        const std::vector<GeodesicWrapSurface>& surfaces,
        const std::vector<GeodesicInitialConditions>& initialConditions,
        const OpenSim::Station& insertion) {
    OPENSIM_THROW_IF_FRMOBJ(getProperty_path_segments().empty(), Exception,
            "The first path segment has not been set.");

    GeodesicPathSegment segment;
    for (int i = 0; i < surfaces.size(); ++i) {
        segment.addWrapObject(surfaces[i], initialConditions[i]);
    }
    const GeodesicPathSegment& previousSegment =
            get_path_segments(getProperty_path_segments().size() - 1);
    const Station& origin =
            previousSegment.getSocket<Station>("insertion").getConnectee();
    segment.connectSocket_origin(origin);
    segment.connectSocket_insertion(insertion);
    append_path_segments(segment);
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
        const GeodesicPathSegment& segment = get_path_segments(i);
        segment.addInEquivalentForces(
                state, tension, bodyForces, mobilityForces);
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

    // Check the properties.
    OPENSIM_THROW_IF_FRMOBJ(getProperty_path_segments().empty(), Exception,
            "Expected at least one path segment, but none were provided.");
}

void Scholz2015GeodesicPath::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

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
void Scholz2015GeodesicPath::constructProperties() {
    constructProperty_path_segments();
}

void Scholz2015GeodesicPath::computeLength(const SimTK::State &s) const {
    double length = 0.0;
    for (int i = 0; i < getProperty_path_segments().size(); ++i) {
        const GeodesicPathSegment& segment = get_path_segments(i);
        length += segment.getLength(s);
    }
    setCacheVariableValue(s, _lengthCV, length);
}

void Scholz2015GeodesicPath::computeLengtheningSpeed(
        const SimTK::State &s) const {
    double lengtheningSpeed = 0.0;
    for (int i = 0; i < getProperty_path_segments().size(); ++i) {
        const GeodesicPathSegment& segment = get_path_segments(i);
        lengtheningSpeed += segment.getLengtheningSpeed(s);
    }
    setCacheVariableValue(s, _lengtheningSpeedCV, lengtheningSpeed);
}