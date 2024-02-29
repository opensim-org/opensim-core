/* -------------------------------------------------------------------------- *
 *                       OpenSim: GeodesicWrapSurface.cpp                     *
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

#include <OpenSim/Common/ModelDisplayHints.h>

using namespace OpenSim;

//=============================================================================
// GEODESIC WRAP SURFACE
//=============================================================================
GeodesicWrapSurface::GeodesicWrapSurface() {
    constructProperty_appearance(Appearance());
    constructProperty_form("");
}

GeodesicWrapSurface::GeodesicWrapSurface(std::string form)
        : GeodesicWrapSurface() {
    getFormFromString(form);
    setForm(std::move(form));
}

GeodesicWrapSurface::~GeodesicWrapSurface() = default;

GeodesicWrapSurface::GeodesicWrapSurface(const GeodesicWrapSurface&) = default;

GeodesicWrapSurface& GeodesicWrapSurface::operator=(
        const GeodesicWrapSurface&) = default;

GeodesicWrapSurface::GeodesicWrapSurface(
        GeodesicWrapSurface&&) noexcept = default;

GeodesicWrapSurface& GeodesicWrapSurface::operator=(
        GeodesicWrapSurface&&) noexcept = default;

void GeodesicWrapSurface::setForm(std::string form) {
    getFormFromString(form);
    set_form(std::move(form));
}

const std::string& GeodesicWrapSurface::getForm() const {
    return get_form();
}

std::string GeodesicWrapSurface::getFormAsString(Form form) {
    if (form == Form::Parametric) {
        return "parametric";
    } else if (form == Form::Implicit) {
        return "implicit";
    } else if (form == Form::Analytic) {
        return "analytic";
    } else {
        OPENSIM_THROW(Exception, "Invalid form specified.");
    }
}

GeodesicWrapSurface::Form GeodesicWrapSurface::getFormFromString(
        const std::string& form) {
    if (form == "parametric") {
        return Form::Parametric;
    } else if (form == "implicit") {
        return Form::Implicit;
    } else if (form == "analytic") {
        return Form::Analytic;
    } else {
        OPENSIM_THROW_IF(form.empty(), Exception,
                "Expected a valid wrap surface form ('implicit', 'parametric', "
                "or 'analytic'), but none were specified.");
        OPENSIM_THROW(Exception,
                "Expected a valid wrap surface form ('implicit', 'parametric', "
                "or 'analytic'), but received '{}'.", form);
    }
}

bool GeodesicWrapSurface::isFormAvailable(Form form) const {
    if (form == Form::Parametric) {
        return isParametricFormAvailable();
    } else if (form == Form::Implicit) {
        return isImplicitFormAvailable();
    } else if (form == Form::Analytic) {
        return isAnalyticFormAvailable();
    } else {
        throw Exception("Invalid form specified.");
    }
}

//=============================================================================
// GEODESIC WRAP CYLINDER
//=============================================================================
GeodesicWrapCylinder::GeodesicWrapCylinder() : GeodesicWrapSurface() {
    constructProperty_radius(0);
}

GeodesicWrapCylinder::GeodesicWrapCylinder(SimTK::Real radius,
        std::string form) : GeodesicWrapSurface(std::move(form)) {
    constructProperty_radius(radius);
    OPENSIM_THROW_IF_FRMOBJ(!isFormAvailable(getFormFromString(getForm())),
            Exception,
            "Form '{}' is not available for a GeodesicWrapCylinder.",
            getForm());
}

void GeodesicWrapCylinder::setRadius(SimTK::Real radius) {
    set_radius(radius);
}

SimTK::Real GeodesicWrapCylinder::getRadius() const {
    return get_radius();
}

std::unique_ptr<GeodesicWrapObject>
GeodesicWrapCylinder::generateGeodesicWrapObject() const {
    if (getFormFromString(getForm()) == Form::Implicit) {
        return std::make_unique<ImplicitCylinderWrapObject>(getRadius());
    } else {
        throw Exception("Invalid form specified.");
    }
}

void GeodesicWrapCylinder::generateDecorations(bool fixed,
        const ModelDisplayHints& hints,
        const SimTK::State& state,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometries) const {

    Super::generateDecorations(fixed, hints, state, geometries);
    if (!fixed) return;

    if (hints.get_show_wrap_geometry()) {
        const Appearance& defaultAppearance = get_appearance();
        if (!defaultAppearance.get_visible()) return;
        const SimTK::Vec3 color = defaultAppearance.get_color();

        SimTK::Transform transform;
        // TODO what transform to use?
        // TODO what length to use?
        transform.updR().setRotationFromAngleAboutX(SimTK_PI / 2);

        SimTK::DecorativeCylinder cylinder(getRadius(), 0.5);
        cylinder.setTransform(transform);
        cylinder.setResolution(2.0);
        cylinder.setColor(color);
        cylinder.setOpacity(defaultAppearance.get_opacity());
        cylinder.setScale(1);
        cylinder.setRepresentation(defaultAppearance.get_representation());

        const auto& frame = getSocket<PhysicalFrame>("frame").getConnectee();
        cylinder.setBodyId(frame.getMobilizedBodyIndex());

        geometries.push_back(cylinder);
    }
}

