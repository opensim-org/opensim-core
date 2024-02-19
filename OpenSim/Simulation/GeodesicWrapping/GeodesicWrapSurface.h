#ifndef OPENSIM_GEODESIC_WRAP_SURFACE_H
#define OPENSIM_GEODESIC_WRAP_SURFACE_H
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

class ImplicitSurfaceParametersImpl;
class GeodesicPathSegment;

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
    enum class Form {
        Parametric,
        Implicit,
        Analytic
    };
//==============================================================================
// SOCKETS
//==============================================================================
    OpenSim_DECLARE_SOCKET(frame, PhysicalFrame,
            "The physical frame defining the position and orientation of "
            "this GeodesicWrapSurface in ground.");

//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(current_form, std::string,
            "The current form of the surface used to compute geodesics.");

//==============================================================================
// METHODS
//==============================================================================

    // CONSTRUCTION AND DESTRUCTION
    GeodesicWrapSurface();

    // GET AND SET
    void setCurrentForm(Form form);
    Form getCurrentForm() const;

    // INTERFACE METHODS
    virtual bool isParametricFormAvailable() const = 0;
    virtual bool isImplicitFormAvailable() const = 0;
    virtual bool isAnalyticFormAvailable() const = 0;
    virtual Form getDefaultForm() const = 0;

protected:
    // MODEL COMPONENT INTERFACE
    void extendFinalizeFromProperties() override;

    // INTERFACE METHODS
    virtual std::unique_ptr<ImplicitSurfaceParamsImpl>
    generateImplicitSurface() const;

    friend GeodesicPathSegment;
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

    // GEODESIC WRAP SURFACE INTERFACE
    bool isParametricFormAvailable() const override { return true; }
    bool isImplicitFormAvailable() const override { return true; }
    bool isAnalyticFormAvailable() const override { return true; }
    Form getDefaultForm() const override { return Form::Implicit; }

protected:
    OpenSim_DECLARE_PROPERTY(radius, SimTK::Real,
            "The radius of the cylindrical surface (default: 1.0).");

    // GEODESIC WRAP SURFACE INTERFACE
    virtual std::unique_ptr<ImplicitSurfaceParametersImpl>
    generateImplicitSurface() const override;
};

} // namespace OpenSim

#endif // OPENSIM_GEODESIC_WRAP_SURFACE_H
