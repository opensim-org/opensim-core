#ifndef OPENSIM_GEODESICWRAPSURFACE_H
#define OPENSIM_GEODESICWRAPSURFACE_H
/* -------------------------------------------------------------------------- *
 *                        OpenSim: GeodesicWrapSurface.h                      *
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

#include "OpenSim/Simulation/Model/ModelComponent.h"
#include "OpenSim/Simulation/Model/PhysicalFrame.h"

namespace OpenSim {

//=============================================================================
//                          GEODESIC WRAP SURFACE
//=============================================================================
/**
 * A base class that represents a smooth geometric surface used to define
 * wrapping obstacles for a GeodesicPath.
 *
 * Concrete implementations must specify if the surface supports a set of
 * parametric, implicit, or analytic equations (i.e., 'forms') used to compute
 * geodesics in a GeodesicPath, and multiple forms may be available for an
 * individual surface. The Socket 'frame' is used to specify the physical frame
 * that defines the position and orientation of the GeodesicWrapSurface, and is
 * used to transform points expressed in the GeodesicWrapSurface's body frame to
 * the ground frame.
 */
class OSIMSIMULATION_API GeodesicWrapSurface : public ModelComponent {
    OpenSim_DECLARE_ABSTRACT_OBJECT(GeodesicWrapSurface, ModelComponent);
public:
//==============================================================================
// SOCKETS
//==============================================================================
    OpenSim_DECLARE_SOCKET(frame, PhysicalFrame,
            "The physical frame defining the position and orientation of "
            "this GeodesicWrapSurface in ground.");

//==============================================================================
// METHODS
//==============================================================================

    // CONSTRUCTION AND DESTRUCTION
    GeodesicWrapSurface();

    // GET AND SET
    /**
     * Return whether the parametric form of the surface is available.
     */
    bool getParametricFormAvailable() const;
    /**
     * Return whether the implicit form of the surface is available.
     */
    bool getImplicitFormAvailable() const;
    /**
     * Return whether the analytic form of the surface is available.
     */
    bool getAnalyticFormAvailable() const;

protected:
    // Concrete classes must define the available forms for a
    // GeodesicWrapSurface.
    virtual void setAvailableForms() = 0;
    void setAnalyticFormAvailable(bool tf);
    void setParametricFormAvailable(bool tf);
    void setImplicitFormAvailable(bool tf);

private:
    bool m_analyticFormAvailable;
    bool m_parametricFormAvailable;
    bool m_implicitFormAvailable;
};

//=============================================================================
//                            GEODESIC WRAP CYLINDER
//=============================================================================
/**
 * A class representing a cylindrical surface used to define wrapping obstacles
 * for a GeodesicPath.
 *
 * The cylinder is defined by a radius and is centered at the origin of the
 * GeodesicWrapSurface's body frame. The cylinder's axis is aligned with the
 * z-axis of the body frame.
 */
class OSIMSIMULATION_API GeodesicWrapCylinder : public GeodesicWrapSurface {
    OpenSim_DECLARE_CONCRETE_OBJECT(GeodesicWrapCylinder, GeodesicWrapSurface);
public:

    // CONSTRUCTION AND DESTRUCTION
    GeodesicWrapCylinder();
    GeodesicWrapCylinder(SimTK::Real radius);

    // GET AND SET
    SimTK::Real getRadius() const;
    void setRadius(SimTK::Real radius);

protected:
    OpenSim_DECLARE_PROPERTY(radius, SimTK::Real,
            "The radius of the cylindrical surface (default: 1.0).");

    void setAvailableForms() override;
};

} // namespace OpenSim

#endif // OPENSIM_GEODESICWRAPSURFACE_H
