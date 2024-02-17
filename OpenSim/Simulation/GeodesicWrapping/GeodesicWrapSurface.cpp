/* -------------------------------------------------------------------------- *
 *                           OpenSim: GeodesicWrapSurface.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco, Andreas Scholz                                 *
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

#include "GeodesicWrapSurface.h"

using namespace OpenSim;

//=============================================================================
// GEODESIC WRAP SURFACE
//=============================================================================
GeodesicWrapSurface::GeodesicWrapSurface() : m_analyticFormAvailable(false),
                                             m_parametricFormAvailable(false),
                                             m_implicitFormAvailable(false) {}

bool GeodesicWrapSurface::getAnalyticFormAvailable() const {
    return m_analyticFormAvailable;
}
void GeodesicWrapSurface::setAnalyticFormAvailable(bool tf) {
    m_analyticFormAvailable = tf;
}

bool GeodesicWrapSurface::getImplicitFormAvailable() const {
    return m_implicitFormAvailable;
}
void GeodesicWrapSurface::setImplicitFormAvailable(bool tf) {
    m_implicitFormAvailable = tf;
}

bool GeodesicWrapSurface::getParametricFormAvailable() const {
    return m_parametricFormAvailable;
}
void GeodesicWrapSurface::setParametricFormAvailable(bool tf) {
    m_parametricFormAvailable = tf;
}

//=============================================================================
// GEODESIC WRAP CYLINDER
//=============================================================================
GeodesicWrapCylinder::GeodesicWrapCylinder() {
    constructProperty_radius(1.0);
}

GeodesicWrapCylinder::GeodesicWrapCylinder(SimTK::Real radius) {
    constructProperty_radius(radius);
}

void GeodesicWrapCylinder::setRadius(SimTK::Real radius) {
    set_radius(radius);
}

SimTK::Real GeodesicWrapCylinder::getRadius() const {
    return get_radius();
}

void GeodesicWrapCylinder::setAvailableForms() {
    setParametricFormAvailable(true);
    setImplicitFormAvailable(true);
    setAnalyticFormAvailable(true);
}
