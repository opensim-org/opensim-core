/* -------------------------------------------------------------------------- *
 *                         OpenSim:  CustomJoint.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Peter Loan, Ajay Seth                        *
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

#include "CustomJoint.h"
#include "SpatialTransform.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/Assertion.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/LinearFunction.h>
#include "simbody/internal/MobilizedBody_FunctionBased.h"

using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
CustomJoint::CustomJoint() {
    constructProperties();
}

CustomJoint::CustomJoint(const std::string& name,
        const PhysicalFrame& parent,
        const PhysicalFrame& child,
        const SpatialTransform& spatialTransform) :
        Super(name, parent, child) {
    constructProperties();
    set_SpatialTransform(spatialTransform);
}

CustomJoint::CustomJoint(const std::string& name,
        const PhysicalFrame& parent,
        const SimTK::Vec3& locationInParent,
        const SimTK::Vec3& orientationInParent,
        const PhysicalFrame& child,
        const SimTK::Vec3& locationInChild,
        const SimTK::Vec3& orientationInChild,
        const SpatialTransform& spatialTransform) :
        Super(name, parent, locationInParent, orientationInParent,
                child, locationInChild, orientationInChild) {
    constructProperties();
    set_SpatialTransform(spatialTransform);
}

//=============================================================================
// ACCESSOR(S)
//=============================================================================
const SpatialTransform& CustomJoint::getSpatialTransform() const {
    return get_SpatialTransform();
}

SpatialTransform& CustomJoint::updSpatialTransform() {
    return upd_SpatialTransform();
}

void CustomJoint::setSpatialTransform(const SpatialTransform& transform) {
    set_SpatialTransform(transform);
}

const Coordinate& CustomJoint::getCoordinate(unsigned idx) const {
    OPENSIM_THROW_IF(numCoordinates() == 0,
                     JointHasNoCoordinates);
    OPENSIM_THROW_IF((int)idx > numCoordinates()-1,
                     InvalidCall,
                     "Index passed to getCoordinate() exceeds the largest "
                     "index available");

    return get_coordinates(idx);
}

Coordinate& CustomJoint::updCoordinate(unsigned idx) {
    OPENSIM_THROW_IF(numCoordinates() == 0,
                     JointHasNoCoordinates);
    OPENSIM_THROW_IF((int)idx > numCoordinates()-1,
                     InvalidCall,
                     "Index passed to updCoordinate() exceeds the largest "
                     "index available");

    return upd_coordinates(idx);
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void CustomJoint::extendScale(const SimTK::State& s, const ScaleSet& scaleSet) {
    Super::extendScale(s, scaleSet);

    // Get scale factors (if an entry for the parent Frame's base Body exists).
    const SimTK::Vec3& scaleFactors =
            getScaleFactors(scaleSet, getParentFrame());
    if (scaleFactors == ModelComponent::InvalidScaleFactors)
        return;

    //TODO: Need to scale transforms appropriately, given an arbitrary axis.
    updSpatialTransform().scale(scaleFactors);
}

void CustomJoint::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();

    // CustomJoint can only construct the coordinates once the SpatialTransform
    // is specified.
    constructCoordinates();

    // Axes should be independent otherwise Simbody throws an exception in
    // extendAddToSystem.
    double tol = 1e-5;
    // Verify that none of the rotation axes are collinear.
    const std::vector<SimTK::Vec3> axes = getSpatialTransform().getAxes();
    for (int startIndex = 0; startIndex <= 3; startIndex += 3) {
        if (((axes[startIndex + 0] % axes[startIndex + 1]).norm() < tol) ||
            ((axes[startIndex + 0] % axes[startIndex + 2]).norm() < tol) ||
            ((axes[startIndex + 1] % axes[startIndex + 2]).norm() < tol)) {
            throw(Exception("CustomJoint '" + getName() +
                "' has collinear axes and are not well-defined."
                " Please fix and retry loading."));
        }
    }
}

void CustomJoint::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);
    // SpatialTransform "connect" methods are poorly named. This
    // establishing the SpatialTransform and its Transform axes as
    // sub objects (not updated to Components) of the CustomJoint and this
    // should be done here.
    /* Set up spatial transform for this custom joint. */
    updSpatialTransform().connectToJoint(*this);
}


void CustomJoint::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    SimTK::MobilizedBody inb;
    SimTK::Body outb;
    SimTK::Transform inbX = getParentFrame().findTransformInBaseFrame();
    SimTK::Transform outbX = getChildFrame().findTransformInBaseFrame();
    const OpenSim::PhysicalFrame* mobilized = &getChildFrame();
    // if the joint is reversed then flip the underlying tree representation
    // of inboard and outboard bodies, although the joint direction will be
    // preserved, the inboard must exist first.
    if (isReversed){
        inb = getChildFrame().getMobilizedBody();
        inbX = getChildFrame().findTransformInBaseFrame();

        outb = getParentInternalRigidBody();
        outbX = getParentFrame().findTransformInBaseFrame();

        mobilized = &getParentFrame();
    }
    else{
        inb = getParentFrame().getMobilizedBody();
        outb = getChildInternalRigidBody();
    }

    const Array<std::string>& coordNames =
            getSpatialTransform().getCoordinateNames();

    // Some initializations
    // Note: should check that all coordinates are used.
    const int numCoords = numCoordinates();
    std::vector<std::vector<int> > coordinateIndices =
        getSpatialTransform().getCoordinateIndices();
    std::vector<const SimTK::Function*> functions =
        getSpatialTransform().getFunctions();
    std::vector<SimTK::Vec3> axes = getSpatialTransform().getAxes();

    SimTK_ASSERT1(numCoords == coordNames.getSize(),
        "%s list of coordinates does not match number of mobilities.",
        getConcreteClassName().c_str());

    SimTK_ASSERT1(numCoords > 0,
        "%s must have 1 or more mobilities (dofs).",
                  getConcreteClassName().c_str());

    SimTK_ASSERT1(numCoords <= 6,
        "%s cannot exceed 6 mobilities (dofs).",
        getConcreteClassName().c_str());
    OPENSIM_ASSERT_FRMOBJ(functions.size() == 6);

    SimTK_ASSERT2(numCoords <= 6,
        "%s::%s must specify functions for complete spatial (6 axes) motion.",
        getConcreteClassName().c_str(),
                  getSpatialTransform().getConcreteClassName().c_str());
    OPENSIM_ASSERT_FRMOBJ(coordinateIndices.size() == 6);

    SimTK_ASSERT2(axes.size() == 6,
        "%s::%s must specify 6 independent axes to span spatial motion.",
        getConcreteClassName().c_str(),
        getSpatialTransform().getConcreteClassName().c_str());

    SimTK::MobilizedBody::Direction dir =
        SimTK::MobilizedBody::Direction(isReversed);

    SimTK::MobilizedBody::FunctionBased
        simtkBody(inb, inbX,
                  outb, outbX,
                    numCoords, functions,
                  coordinateIndices, axes, dir);

    assignSystemIndicesToBodyAndCoordinates(simtkBody, mobilized, numCoords, 0);
}

//=============================================================================
// OBJECT INTERFACE
//=============================================================================
void CustomJoint::updateFromXMLNode(SimTK::Xml::Element& aNode,
        int versionNumber) {
    int documentVersion = versionNumber;
    if ( documentVersion < XMLDocument::getLatestVersion()){
        log_debug("Updating CustomJoint to latest format...");
        // Version before refactoring spatialTransform
        if (documentVersion<10901){
            // replace TransformAxisSet with SpatialTransform
            /////renameChildNode("TransformAxisSet", "SpatialTransform");
            // Check how many TransformAxes are defined
            SimTK::Xml::element_iterator spatialTransformNode =
                aNode.element_begin("TransformAxisSet");
            if (spatialTransformNode == aNode.element_end()) return;

            SimTK::Xml::element_iterator axesSetNode =
                spatialTransformNode->element_begin("objects");
            /////if (axesSetNode != NULL)
            /////   spatialTransformNode->removeChild(axesSetNode);
            /////DOMElement*grpNode = XMLNode::GetFirstChildElementByTagName(spatialTransformNode,"groups");
            /////if (grpNode != NULL)
            /////   spatialTransformNode->removeChild(grpNode);
            // (0, 1, 2 rotations & 3, 4, 5 translations then remove the is_rotation node)
            SimTK::Array_<SimTK::Xml::Element> list =
                axesSetNode->getAllElements();
            unsigned int listLength = list.size();
            //int objectsFound = 0;
            Array<int> translationIndices(-1, 0);
            Array<int>  rotationIndices(-1, 0);
            int nextAxis = 0;
            std::vector<TransformAxis *> axes;
            // Add children for all six axes here
            //////_node->removeChild(SpatialTransformNode);

            // Create a blank Spatial Transform and use it to populate the
            // XML structure
            for(int i=0; i<6; i++)
                updSpatialTransform()[i].setFunction(new OpenSim::Constant(0));

            Array<OpenSim::TransformAxis*> oldAxes;
            for(unsigned int j=0;j<listLength;j++) {
                // getChildNodes() returns all types of DOMNodes including
                // comments, text, etc., but we only want
                // to process element nodes
                SimTK::Xml::Element objElmt = list[j];
                std::string objectType = objElmt.getElementTag();
                // (sherm) this is cleaning up old TransformAxis here but
                // that should really be done in TransformAxis instead.
                if (objectType == "TransformAxis"){
                    OpenSim::TransformAxis* readAxis =
                        new OpenSim::TransformAxis(objElmt);
                    OPENSIM_ASSERT_FRMOBJ(nextAxis <=5);
                    bool isRotation = false;
                    SimTK::Xml::element_iterator rotationNode =
                        objElmt.element_begin("is_rotation");
                    if (rotationNode != objElmt.element_end()){
                        SimTK::String sValue =
                            rotationNode->getValueAs<SimTK::String>();
                        bool value = (sValue.toLower() == "true");
                                isRotation = value;
                        objElmt.eraseNode(rotationNode);
                            }
                    SimTK::Xml::element_iterator coordinateNode =
                        objElmt.element_begin("coordinate");
                    SimTK::String coordinateName =
                        coordinateNode->getValueAs<SimTK::String>();
                    Array<std::string> names("");
                    names.append(coordinateName);
                    readAxis->setCoordinateNames(names);
                    SimTK::Xml::element_iterator axisNode =
                        objElmt.element_begin("axis");

                    SimTK::Vec3 axisVec= axisNode->getValueAs<SimTK::Vec3>();
                    readAxis->setAxis(axisVec);
                    if (isRotation){
                        rotationIndices.append(nextAxis);
                    }
                    else {
                        translationIndices.append(nextAxis);
                    }
                    axes.push_back(readAxis);
                    nextAxis++;
                }
            }
            OPENSIM_ASSERT_FRMOBJ(rotationIndices.getSize() <=3);
            OPENSIM_ASSERT_FRMOBJ(translationIndices.getSize() <=3);
            //XMLNode::RemoveChildren(SpatialTransformAxesNode);
            int nRotations = rotationIndices.getSize();
            int nTranslations = translationIndices.getSize();
            // Now copy coordinateName, Axis, Function into proper slot
            for (int i=0; i<nRotations; i++){
                updSpatialTransform()[i] = *axes[rotationIndices[i]];
            }
            updSpatialTransform().constructIndependentAxes(nRotations, 0);
            // Add Translations from old list then pad with default ones,
            // make sure no singularity.
            for (int i=0; i<nTranslations; i++){
                updSpatialTransform()[i+3] = *axes[translationIndices[i]];
            }
            updSpatialTransform().constructIndependentAxes(nTranslations, 3);
        }
    }

    // Delegate to superclass now.
    Super::updateFromXMLNode(aNode, versionNumber);
}

//=============================================================================
// CONVENIENCE METHODS
//=============================================================================
void CustomJoint::constructProperties() {
    setAuthors("Frank C. Anderson, Ajay Seth");
    constructProperty_SpatialTransform(SpatialTransform());
}

void CustomJoint::constructCoordinates()
{
    const SpatialTransform& spatialTransform = getSpatialTransform();
    OpenSim::Array<std::string> coordinateNames =
            spatialTransform.getCoordinateNames(); // in order of appearance in
                                                   // spatialTransform

    // Check how many coordinates are required
    int ncoords = coordinateNames.getSize();
    std::vector<int> coordIndices;
    bool coordinatesInTransformOrder = true;
    for (int i = 0; i < ncoords; ++i) {
        std::string coordName = coordinateNames[i];
        // Locate the coordinate in the set if it has already been defined (e.g.
        // in XML)
        int coordIndex = getProperty_coordinates().findIndexForName(coordName);
        if (coordIndex < 0) {
            coordIndex = constructCoordinate(
                    Coordinate::MotionType::Undefined, i);
        } else {
            // Coordinate was defined in XML, make sure the order in
            // spatialTransform matches the order in the XML file/property
            coordinatesInTransformOrder = coordinatesInTransformOrder &&
                    coordIndex == i;
            coordIndices.push_back(coordIndex);
        }
        Coordinate& coord = upd_coordinates(coordIndex);
        coord.setName(coordName);

        // Determine the MotionType of the Coordinate based
        // on which TransformAxis it operates upon: 0-2 are Rotational
        // and 3-5 are Translational. Coordinates appearing
        // in both categories are Coupled
        Coordinate::MotionType mt = Coordinate::MotionType::Undefined;
        for (int j = 0; j < 6; ++j){
            auto coordNamesForAxis = spatialTransform[j]
                .getCoordinateNamesInArray();
            if (coordNamesForAxis.findIndex(coordName) >= 0) {
                const LinearFunction* lf = nullptr;
                if (spatialTransform[j].hasFunction()) {
                    lf = dynamic_cast<const LinearFunction*>(
                            &spatialTransform[j].get_function() );
                    // if displacement on axis is linear (w/ slope of +/-1)
                    // w.r.t. the coordinate value, we have a pure
                    // rotation/translation
                    if (lf && (lf->getSlope() == 1.0 || lf->getSlope() == -1.0))
                    {
                        // coordinate is pure axis displacement
                        if (j < 3)
                            // coordinate about rotational axis
                            mt = Coordinate::MotionType::Rotational;
                        else // otherwise translational axis
                            mt = Coordinate::MotionType::Translational;
                    } else {
                        // scaled (slope != +/-1) or nonlinear relationship
                        // means not a pure rotational or translational for
                        // this axis. designate as Coupled unless already
                        // defined as pure Rotational or Translational about
                        // another axis
                        mt = (mt == Coordinate::MotionType::Undefined ?
                            Coordinate::MotionType::Coupled : mt);
                    }
                }
            }
        }
        setMotionType(CoordinateIndex(i), mt);
    }
    // Reorder Coordinates list in property to match the order in
    // SpatialTransform issue #3191
    if (!coordinatesInTransformOrder) {
        // Make a fresh copy of the CoordinateSet in Property
        CoordinateSet newCoordinates;
        for (int ix = 0; ix < getProperty_coordinates().size(); ix++)
            newCoordinates.cloneAndAppend(
                    getProperty_coordinates()[coordIndices.at(ix)]);
        // Copy back values to proper entry in Property_coordinates.
        // Ideally this would be done using copy construction or operators
        // unfortunately these wipe out pointers thus breaking downstream code.
        // so copying one property at a time -Ayman 5/22
        for (int ix = 0; ix < getProperty_coordinates().size(); ix++) {
            Coordinate newCoord = newCoordinates[ix];
            updProperty_coordinates()[ix].setName(newCoord.getName());
            updProperty_coordinates()[ix].set_default_value(
                    newCoord.get_default_value());
            updProperty_coordinates()[ix].set_default_speed_value(
                    newCoord.get_default_speed_value());
            updProperty_coordinates()[ix].setRangeMin(newCoord.getRangeMin());
            updProperty_coordinates()[ix].setRangeMax(newCoord.getRangeMax());
            updProperty_coordinates()[ix].set_clamped(newCoord.get_clamped());
            updProperty_coordinates()[ix].set_locked(newCoord.get_locked());
            updProperty_coordinates()[ix].set_prescribed(
                    newCoord.get_prescribed());
            updProperty_coordinates()[ix].set_is_free_to_satisfy_constraints(
                    newCoord.get_is_free_to_satisfy_constraints());
            if (!newCoord.getProperty_prescribed_function()
                     .empty())
                updProperty_coordinates()[ix].set_prescribed_function(
                        *(newCoord.get_prescribed_function().clone()));

        }
    }
}
