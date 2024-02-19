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
void GeodesicPathSegment::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    _pathCV = addCacheVariable<GeodesicWrappingPath>(
            "path", GeodesicWrappingPath(), SimTK::Stage::Position);
}

void GeodesicPathSegment::extendRealizeTopology(SimTK::State& state) const {
    Super::extendRealizeTopology(state);
    const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
    m_discreteVarIndex =
            subSys.allocateDiscreteVariable(state, SimTK::Stage::Position,
                    new Geodesics());
}

void GeodesicPathSegment::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    // Loop through all surfaces to create GeodesicWrapObjects to add to this
    // path segment. Each surface
    _wrapObjects.clear();
    const auto& surfaces = getSocket<GeodesicWrapSurface>("surfaces");
    for (int i = 0; i < surfaces.getNumConnectees(); ++i) {
        const GeodesicWrapSurface& surface = surfaces.getConnectee(i);
        GeodesicWrapSurface::Form form = surface.getCurrentForm();
        const PhysicalFrame& frame =
                surface.getSocket<PhysicalFrame>("frame").getConnectee();
        auto frameRefPtr = SimTK::ReferencePtr<const PhysicalFrame>(frame);

        if (form == GeodesicWrapSurface::Form::Implicit) {
            auto wrapSurface = surface.generateImplicitSurface();
            ImplicitSurfaceParameters surfaceParams(wrapSurface.release());
            GeodesicWrapObject wrapObject(surfaceParams, frameRefPtr);
            _wrapObjects.push_back(wrapObject);

        } else if (form == GeodesicWrapSurface::Form::Parametric) {
            // TODO
        } else if (form == GeodesicWrapSurface::Form::Analytic) {
            // TODO
        } else {
            OPENSIM_THROW_FRMOBJ(Exception,
                    "Unrecognized form '" + form + "' for GeodesicWrapSurface '" +
                    surface.getName() + "'.");
        }
    }
}

double GeodesicPathSegment::getLength(const SimTK::State& s) const {
    calcWrappingPath(s);
    const auto& path = getCacheVariableValue<GeodesicWrappingPath>(s, _pathCV);
    return path.getLength();
}

double GeodesicPathSegment::getLengtheningSpeed(const SimTK::State& s) const {
    calcWrappingPath(s);
    const auto& path = getCacheVariableValue<GeodesicWrappingPath>(s, _pathCV);
    return path.getLengtheningSpeed();
}

void GeodesicPathSegment::addInEquivalentForces(const SimTK::State& state,
        const double& tension,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
        SimTK::Vector& mobilityForces) const {
    // TODO use Darboux frames to compute direction of tension
}

void GeodesicPathSegment::calcWrappingPath(const SimTK::State &s) const {
    if (isCacheVariableValid(s, _pathCV)) {
        return;
    }
    const Station& origin = getSocket<Station>("origin").getConnectee();
    const Station& insertion = getSocket<Station>("insertion").getConnectee();
    const SimTK::Vec3& originPoint = origin.getLocationInGround(s);
    const SimTK::Vec3& insertionPoint = insertion.getLocationInGround(s);
    // TODO move these settings somewhere else
    size_t maxIter = 20;
    double eps = 1e-13;

    GeodesicWrappingPath result;
    _solver.calcWrappingPath(s, originPoint, insertionPoint, _wrapObjects,
            result, maxIter, eps);
    setCacheVariableValue<GeodesicWrappingPath>(s, _pathCV, result);
}

//=============================================================================
// SCHOLZ2015 GEODESIC PATH
//=============================================================================
Scholz2015GeodesicPath::Scholz2015GeodesicPath() : AbstractGeometryPath() {
    setAuthors("Nicholas Bianco, Pepijn van den Bos, Andreas Scholz");
    constructProperties();
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
            "Expected at least one path segment, but none were provided.")

    // If more than one segment, check that the insertion of a preceding segment
    // is equal to the origin of the following segment.
    // TODO provide an interface so that it's not necessary to check this
    for (int i = 0; i < getProperty_path_segments().size() - 1; ++i) {
        const GeodesicPathSegment &segment1 = get_path_segments(i);
        const GeodesicPathSegment &segment2 = get_path_segments(i + 1);
        const Station& insertion =
                segment1.getSocket<Station>("insertion").getConnectee();
        const Station& origin =
                segment2.getSocket<Station>("origin").getConnectee();

        const PhysicalFrame& insertionFrame = insertion.getParentFrame();
        const PhysicalFrame& originFrame = origin.getParentFrame();
        OPENSIM_THROW_IF_FRMOBJ(&insertionFrame != &originFrame, Exception,
                "The insertion frame of segment " + segment1.getName() +
                " does not match the origin frame of segment " +
                segment2.getName() + ".");

        const SimTK::Vec3& insertionLocation = insertion.get_location();
        const SimTK::Vec3& originLocation = origin.get_location();
        OPENSIM_THROW_IF_FRMOBJ(
                insertionLocation != originLocation, Exception,
                "The insertion location of segment " + segment1.getName() +
                " does not match the origin location of segment " +
                segment2.getName() + ".");
    }
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