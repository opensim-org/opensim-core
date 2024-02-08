/* -------------------------------------------------------------------------- *
 *                           OpenSim: Geodesic.h                              *
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

#include "Geodesic.h"

using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
Geodesic::Geodesic() : Object(),
                       m_surfaceFrame(nullptr),
                       m_boundaryPointFrameAtStart(GeodesicBoundaryPointFrame()),
                       m_boundaryPointFrameAtEnd(GeodesicBoundaryPointFrame()),
                       m_tangentialNormalCurvatureAtStart(0.0),
                       m_binormalNormalCurvatureAtStart(0.0),
                       m_tangentialNormalCurvatureAtEnd(0.0),
                       m_binormalNormalCurvatureAtEnd(0.0),
                       m_tangentialGeodesicTorsionAtStart(0.0),
                       m_tangentialGeodesicTorsionAtEnd(0.0),
                       m_length(0.0),
                       m_speed(0.0),
                       m_acceleration(0.0),
                       m_displacementJacobiScalarAtStart(JacobiScalar()),
                       m_displacementJacobiScalarAtEnd(JacobiScalar()),
                       m_rotationJacobiScalarAtStart(JacobiScalar()),
                       m_rotationJacobiScalarAtEnd(JacobiScalar()) {
}

Geodesic::~Geodesic() noexcept = default;

Geodesic::Geodesic(const Geodesic&) = default;

Geodesic& Geodesic::operator=(const Geodesic&) = default;

Geodesic::Geodesic(Geodesic&&) = default;

Geodesic& Geodesic::operator=(Geodesic&&) = default;

//=============================================================================
// GET AND SET
//=============================================================================
const PhysicalFrame& Geodesic::getSurfaceFrame() const {
    return m_surfaceFrame.getRef();
}

void Geodesic::setSurfaceFrame(const PhysicalFrame& surfaceFrame) {
    m_surfaceFrame.reset(&surfaceFrame);
}
