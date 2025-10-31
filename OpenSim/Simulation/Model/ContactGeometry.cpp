/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ContactGeometry.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Peter Eastman                                                   *
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

#include "ContactGeometry.h"

#include <OpenSim/Common/ModelDisplayHints.h>

using namespace OpenSim;

//=============================================================================
// CONSTRUCTION
//=============================================================================
ContactGeometry::ContactGeometry() : ModelComponent()
{
    setNull();
    constructProperties();
}

ContactGeometry::ContactGeometry(const PhysicalFrame& frame) : ContactGeometry()
{
    setFrame(frame);
}

ContactGeometry::ContactGeometry(const SimTK::Vec3& location,
    const SimTK::Vec3& orientation, const PhysicalFrame& frame) :
    ContactGeometry(frame)
{
    set_location(location);
    set_orientation(orientation);
}

void ContactGeometry::setNull() {
    setAuthors("Peter Eastman");
}

void ContactGeometry::constructProperties()
{
    constructProperty_location(SimTK::Vec3(0));
    constructProperty_orientation(SimTK::Vec3(0));
    Appearance defaultAppearance;
    defaultAppearance.set_color(SimTK::Cyan);
    defaultAppearance.set_representation(VisualRepresentation::DrawWireframe);
    constructProperty_Appearance(defaultAppearance);
}

//=============================================================================
// ACCESSORS
//=============================================================================
const PhysicalFrame& ContactGeometry::getFrame() const
{
    return getSocket<PhysicalFrame>("frame").getConnectee();
}

void ContactGeometry::setFrame(const PhysicalFrame& frame)
{
    connectSocket_frame(frame);
}

SimTK::Transform ContactGeometry::getTransform() const
{
    return SimTK::Transform(
            SimTK::Rotation(SimTK::BodyRotationSequence,
                get_orientation()[0], SimTK::XAxis,
                get_orientation()[1], SimTK::YAxis,
                get_orientation()[2], SimTK::ZAxis),
            get_location());
}

SimTK::ContactGeometry ContactGeometry::createSimTKContactGeometry() const
{
    return createSimTKContactGeometryImpl();
}

//=============================================================================
// OBJECT INTERFACE
//=============================================================================
void ContactGeometry::updateFromXMLNode(SimTK::Xml::Element& node,
        int versionNumber) {
    if (versionNumber < XMLDocument::getLatestVersion()) {
        if (versionNumber < 30505) {
            SimTK::Xml::element_iterator bodyElement =
                node.element_begin("body_name");
            std::string body_name("");
            // Element may not exist if body_name property had default value.
            if (bodyElement != node.element_end()) {
                bodyElement->getValueAs<std::string>(body_name);
            }
            // ContactGeometry in pre-4.0 models are necessarily 1 level deep
            // (model, contact_geom), and Bodies are necessarily 1 level deep.
            // Here we create the correct relative path (accounting for sets
            // being components).
            body_name = XMLDocument::updateConnecteePath30517("bodyset",
                                                              body_name);
            XMLDocument::addConnector(node, "Connector_PhysicalFrame_",
                    "frame", body_name);
        }

        if (versionNumber < 30507) {
            // display_preference and color were replaced with Appearance in PR
            // #1122, at which point the latest XML version number was 30507;
            // however, the corresponding updateFromXMLNode code below was
            // added a while later.
            // https://github.com/opensim-org/opensim-core/pull/1122

            SimTK::Xml::Element appearanceNode("Appearance");

            // Move color inside Appearance.
            SimTK::Xml::element_iterator colorIter =
                        node.element_begin("color");
            bool addAppearanceNode = false;
            if (colorIter != node.element_end()) {
                appearanceNode.insertNodeAfter(appearanceNode.element_end(),
                        node.removeNode(colorIter));
                addAppearanceNode = true;
            } else {
                // If the user didn't set a color but set one of the other
                // Appearance properties, then we must set color explicitly so
                // that the Appearance's default of white is not used.
                SimTK::Xml::Element color("color");
                color.setValue("0 1 1"); // SimTK::Cyan
                appearanceNode.insertNodeAfter(appearanceNode.element_end(),
                        color);
            }

            // Move <display_preference> to
            // <Appearance><SurfaceProperties><representation>
            SimTK::Xml::element_iterator reprIter =
                    node.element_begin("display_preference");
            if (reprIter != node.element_end()) {
                if (reprIter->getValue() == "0") {
                    SimTK::Xml::Element visible("visible");
                    visible.setValue("false");
                    appearanceNode.insertNodeAfter(appearanceNode.element_end(),
                            visible);
                } else {
                    reprIter->setElementTag("representation");
                    if (reprIter->getValue() == "4") {
                        // Enum changed to go 0-3 instead of 0-4
                        reprIter->setValue("3");
                    }
                    SimTK::Xml::Element surfProp("SurfaceProperties");
                    surfProp.insertNodeAfter(surfProp.element_end(),
                            node.removeNode(reprIter));
                    appearanceNode.insertNodeAfter(appearanceNode.element_end(),
                            surfProp);
                }
                addAppearanceNode = true;
            }
            if (addAppearanceNode)
                node.insertNodeAfter(node.element_end(), appearanceNode);
            else if (appearanceNode.isOrphan())
                appearanceNode.clearOrphan();
        }
    }
    Super::updateFromXMLNode(node, versionNumber);
}

//=============================================================================
// CONTACT SPHERE
//=============================================================================
ContactSphere::ContactSphere() : ContactGeometry()
{
    setAuthors("Peter Eastman");
    constructProperty_radius(0);
}

ContactSphere::ContactSphere(double radius, const SimTK::Vec3& location,
        const PhysicalFrame& frame) :
    ContactGeometry(location, SimTK::Vec3(0.0), frame)
{
    setAuthors("Peter Eastman");
    constructProperty_radius(0);
    set_radius(radius);
}

ContactSphere::ContactSphere(double radius, const SimTK::Vec3& location,
        const PhysicalFrame& frame, const std::string& name) :
    ContactSphere(radius, location, frame)
{
    setName(name);
}

double ContactSphere::getRadius() const
{
    return get_radius();
}

void ContactSphere::setRadius(double radius)
{
    set_radius(radius);
}

SimTK::ContactGeometry ContactSphere::createSimTKContactGeometryImpl() const
{
    return SimTK::ContactGeometry::Sphere(get_radius());
}

void ContactSphere::generateDecorations(
        bool fixed,
        const ModelDisplayHints& hints,
        const SimTK::State& s,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const {

    // There is no fixed geometry to generate here.
    if (fixed) { return; }

    // Model-wide hints indicate that contact geometry shouldn't be shown.
    if (!hints.get_show_contact_geometry()) { return; }

    // The decoration has been toggled off by its `Appearance` block.
    if (!get_Appearance().get_visible())  { return; }

    // Create a SimTK::DecorativeSphere object for this ContactSphere.
    SimTK::DecorativeSphere decoration(get_radius());

    // B: base Frame (Body or Ground)
    // F: PhysicalFrame that this ContactGeometry is connected to
    // P: the frame defined (relative to F) by the location and orientation
    //    properties defined in ContactGeometry.
    // D: the frame defined (relative to P) by the decoration's location and
    //    orientation properties defined by the SimTK::DecorativeGeometry object.
    const auto& X_BF = getFrame().findTransformInBaseFrame();
    const auto& X_FP = getTransform();
    const auto& X_PD = decoration.getTransform();
    const auto X_BD = X_BF.compose(X_FP).compose(X_PD);
    geometry.push_back(decoration
            .setScale(1)
            .setTransform(X_BD)
            .setRepresentation(get_Appearance().get_representation())
            .setBodyId(getFrame().getMobilizedBodyIndex())
            .setColor(get_Appearance().get_color())
            .setOpacity(get_Appearance().get_opacity()));
}

//=============================================================================
// CONTACT CYLINDER
//=============================================================================
ContactCylinder::ContactCylinder() : ContactGeometry()
{
    constructProperty_radius(0);
}

ContactCylinder::ContactCylinder(double radius, const SimTK::Vec3& location,
        const SimTK::Vec3& orientation, const PhysicalFrame& frame) :
    ContactGeometry(location, orientation, frame)
{
    constructProperty_radius(0);
    set_radius(radius);
}

ContactCylinder::ContactCylinder(double radius, const SimTK::Vec3& location,
        const SimTK::Vec3& orientation, const PhysicalFrame& frame,
        const std::string& name) :
    ContactCylinder(radius, location, orientation, frame)
{
    setName(name);
}

double ContactCylinder::getRadius() const
{
    return get_radius();
}

void ContactCylinder::setRadius(double radius)
{
    set_radius(radius);
}

SimTK::ContactGeometry ContactCylinder::createSimTKContactGeometryImpl() const
{
    return SimTK::ContactGeometry::Cylinder(get_radius());
}

void ContactCylinder::generateDecorations(
        bool fixed,
        const ModelDisplayHints& hints,
        const SimTK::State& s,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const {

    // There is no fixed geometry to generate here.
    if (fixed) { return; }

    // Model-wide hints indicate that contact geometry shouldn't be shown.
    if (!hints.get_show_contact_geometry()) { return; }

    // The decoration has been toggled off by its `Appearance` block.
    if (!get_Appearance().get_visible())  { return; }

    // Create a SimTK::DecorativeCylinder object for this ContactCylinder.
    SimTK::DecorativeCylinder decoration(get_radius(), get_radius()*2);
    // SimTK::DecorativeCylinder's axis is defined as the y-axis,
    // whereas SimTK::ContactGeometry::Cylinder axis is defined as the z-axis.
    decoration.setTransform(SimTK::Rotation(SimTK::Pi/2, SimTK::XAxis));

    // B: base Frame (Body or Ground)
    // F: PhysicalFrame that this ContactGeometry is connected to
    // P: the frame defined (relative to F) by the location and orientation
    //    properties defined in ContactGeometry.
    // D: the frame defined (relative to P) by the decoration's location and
    //    orientation properties defined by the SimTK::DecorativeGeometry object.
    const auto& X_BF = getFrame().findTransformInBaseFrame();
    const auto& X_FP = getTransform();
    const auto& X_PD = decoration.getTransform();
    const auto X_BD = X_BF.compose(X_FP).compose(X_PD);
    geometry.push_back(decoration
            .setScale(1)
            .setTransform(X_BD)
            .setRepresentation(get_Appearance().get_representation())
            .setBodyId(getFrame().getMobilizedBodyIndex())
            .setColor(get_Appearance().get_color())
            .setOpacity(get_Appearance().get_opacity()));
}

//=============================================================================
// CONTACT ELLIPSOID
//=============================================================================
ContactEllipsoid::ContactEllipsoid() : ContactGeometry()
{
    constructProperty_radii(SimTK::Vec3(0));
}

ContactEllipsoid::ContactEllipsoid(const SimTK::Vec3& radii,
        const SimTK::Vec3& location, const SimTK::Vec3& orientation,
        const PhysicalFrame& frame) :
    ContactGeometry(location, orientation, frame)
{
    constructProperty_radii(SimTK::Vec3(0));
    set_radii(radii);
}

ContactEllipsoid::ContactEllipsoid(const SimTK::Vec3& radii,
        const SimTK::Vec3& location, const SimTK::Vec3& orientation,
        const PhysicalFrame& frame, const std::string& name) :
    ContactEllipsoid(radii, location, orientation, frame)
{
    setName(name);
}

const SimTK::Vec3& ContactEllipsoid::getRadii() const
{
    return get_radii();
}

void ContactEllipsoid::setRadii(const SimTK::Vec3& radii)
{
    set_radii(radii);
}

SimTK::ContactGeometry ContactEllipsoid::createSimTKContactGeometryImpl() const
{
    return SimTK::ContactGeometry::Ellipsoid(get_radii());
}

void ContactEllipsoid::generateDecorations(
        bool fixed,
        const ModelDisplayHints& hints,
        const SimTK::State& s,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const {

    // There is no fixed geometry to generate here.
    if (fixed) { return; }

    // Model-wide hints indicate that contact geometry shouldn't be shown.
    if (!hints.get_show_contact_geometry()) { return; }

    // The decoration has been toggled off by its `Appearance` block.
    if (!get_Appearance().get_visible())  { return; }

    // Create a SimTK::DecorativeEllipsoid object for this ContactEllipsoid.
    SimTK::DecorativeEllipsoid decoration(get_radii());

    // B: base Frame (Body or Ground)
    // F: PhysicalFrame that this ContactGeometry is connected to
    // P: the frame defined (relative to F) by the location and orientation
    //    properties defined in ContactGeometry.
    // D: the frame defined (relative to P) by the decoration's location and
    //    orientation properties defined by the SimTK::DecorativeGeometry object.
    const auto& X_BF = getFrame().findTransformInBaseFrame();
    const auto& X_FP = getTransform();
    const auto& X_PD = decoration.getTransform();
    const auto X_BD = X_BF.compose(X_FP).compose(X_PD);
    geometry.push_back(decoration
            .setScale(1)
            .setTransform(X_BD)
            .setRepresentation(get_Appearance().get_representation())
            .setBodyId(getFrame().getMobilizedBodyIndex())
            .setColor(get_Appearance().get_color())
            .setOpacity(get_Appearance().get_opacity()));
    }

//=============================================================================
// CONTACT TORUS
//=============================================================================
ContactTorus::ContactTorus() : ContactGeometry()
{
    constructProperty_torus_radius(0);
    constructProperty_tube_radius(0);
}

ContactTorus::ContactTorus(double torus_radius, double tube_radius,
        const SimTK::Vec3& location, const SimTK::Vec3& orientation,
        const PhysicalFrame& frame) :
    ContactGeometry(location, orientation, frame)
{
    constructProperty_torus_radius(0);
    constructProperty_tube_radius(0);
    set_torus_radius(torus_radius);
    set_tube_radius(tube_radius);
}

ContactTorus::ContactTorus(double torus_radius, double tube_radius,
        const SimTK::Vec3& location, const SimTK::Vec3& orientation,
        const PhysicalFrame& frame, const std::string& name) :
    ContactTorus(torus_radius, tube_radius, location, orientation, frame)
{
    setName(name);
}

double ContactTorus::getTorusRadius() const
{
    return get_torus_radius();
}

void ContactTorus::setTorusRadius(double radius)
{
    set_torus_radius(radius);
}

double ContactTorus::getTubeRadius() const
{
    return get_tube_radius();
}

void ContactTorus::setTubeRadius(double radius)
{
    set_tube_radius(radius);
}

SimTK::ContactGeometry ContactTorus::createSimTKContactGeometryImpl() const
{
    return SimTK::ContactGeometry::Torus(get_torus_radius(), get_tube_radius());
}

void ContactTorus::generateDecorations(
        bool fixed,
        const ModelDisplayHints& hints,
        const SimTK::State& s,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const {

    // There is no fixed geometry to generate here.
    if (fixed) { return; }

    // Model-wide hints indicate that contact geometry shouldn't be shown.
    if (!hints.get_show_contact_geometry()) { return; }

    // The decoration has been toggled off by its `Appearance` block.
    if (!get_Appearance().get_visible())  { return; }

    // Create a SimTK::DecorativeTorus object for this ContactTorus.
    SimTK::DecorativeGeometry decoration =
            createSimTKContactGeometry().createDecorativeGeometry();

    // B: base Frame (Body or Ground)
    // F: PhysicalFrame that this ContactGeometry is connected to
    // P: the frame defined (relative to F) by the location and orientation
    //    properties defined in ContactGeometry.
    // D: the frame defined (relative to P) by the decoration's location and
    //    orientation properties defined by the SimTK::DecorativeGeometry object.
    const auto& X_BF = getFrame().findTransformInBaseFrame();
    const auto& X_FP = getTransform();
    const auto& X_PD = decoration.getTransform();
    const auto X_BD = X_BF.compose(X_FP).compose(X_PD);
    geometry.push_back(decoration
            .setScale(1)
            .setTransform(X_BD)
            .setRepresentation(get_Appearance().get_representation())
            .setBodyId(getFrame().getMobilizedBodyIndex())
            .setColor(get_Appearance().get_color())
            .setOpacity(get_Appearance().get_opacity()));
}
