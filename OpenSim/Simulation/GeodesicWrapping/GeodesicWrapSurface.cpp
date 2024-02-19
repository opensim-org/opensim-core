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
#include "ImplicitSurfaceParameters.h"
#include "ImplicitCylinder.h"

using namespace OpenSim;

//=============================================================================
// GEODESIC WRAP SURFACE
//=============================================================================
GeodesicWrapSurface::GeodesicWrapSurface() {
    constructProperty_current_form("");
}

void GeodesicWrapSurface::setCurrentForm(Form form) {
    // TODO check available forms
    if (form == Form::Parametric) {
        set_current_form("parametric");
    } else if (form == Form::Implicit) {
        set_current_form("implicit");
    } else if (form == Form::Analytic) {
        set_current_form("analytic");
    } else {
        throw Exception("Invalid form specified.");
    }
}

GeodesicWrapSurface::Form GeodesicWrapSurface::getCurrentForm() const {
    // TODO check available forms
    if (get_current_form() == "parametric") {
        return Form::Parametric;
    } else if (get_current_form() == "implicit") {
        return Form::Implicit;
    } else if (get_current_form() == "analytic") {
        return Form::Analytic;
    } else {
        throw Exception("Invalid form specified.");
    }
}

void GeodesicWrapSurface::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();

    // TODO check available forms
    if (get_current_form().empty()) {
        Form form = getDefaultForm();
        setCurrentForm(form);
        if (form == Form::Implicit) {
            set_current_form("implicit");
        } else if (form == Form::Parametric) {
            set_current_form("parametric");
        } else if (form == Form::Analytic) {
            set_current_form("analytic");
        }
    }
}

std::unique_ptr<ImplicitSurfaceParametersImpl>
GeodesicWrapSurface::generateImplicitSurface() const {
    OPENSIM_THROW_FRMOBJ(Exception, "generateImplicitSurface() not implemented.");
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

std::unique_ptr<ImplicitSurfaceParametersImpl>
GeodesicWrapCylinder::generateImplicitSurface() const {
    return std::make_unique<ImplicitCylinder>(getRadius());
}

