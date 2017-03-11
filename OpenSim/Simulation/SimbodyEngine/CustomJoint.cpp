/* -------------------------------------------------------------------------- *
 *                         OpenSim:  CustomJoint.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "CustomJoint.h"
#include "SpatialTransform.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/LinearFunction.h>
#include "simbody/internal/MobilizedBody_FunctionBased.h"


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/* Default Constructor */
CustomJoint::CustomJoint()
{
    constructProperties();
}


/**
 * Constructor with specified SpatialTransform.
 */
CustomJoint::CustomJoint(const std::string&    name,
                         const PhysicalFrame&  parent,
                         const PhysicalFrame&  child,
                         SpatialTransform&     spatialTransform) :
                         Super(name, parent, child)
{
    constructProperties();
    set_SpatialTransform(spatialTransform);
}

CustomJoint::CustomJoint(const std::string&    name,
                         const PhysicalFrame&  parent,
                         const SimTK::Vec3&    locationInParent,
                         const SimTK::Vec3&    orientationInParent,
                         const PhysicalFrame&  child,
                         const SimTK::Vec3&    locationInChild,
                         const SimTK::Vec3&    orientationInChild,
                         SpatialTransform&     spatialTransform) :
    Super(name, parent, locationInParent, orientationInParent,
          child, locationInChild, orientationInChild)
{
    constructProperties();
    set_SpatialTransform(spatialTransform);
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void CustomJoint::constructProperties()
{
    setAuthors("Frank C. Anderson, Ajay Seth");
    constructProperty_SpatialTransform(SpatialTransform());
}

/*
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 */
void CustomJoint::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

    //CustomJoint can only construct the coordinates once SpatialTransform is specified 
    constructCoordinates();

    // Axes should be independent otherwise Simbody throws an exception in extendAddToSystem
    double tol = 1e-5;
    // Verify that none of the rotation axes are collinear
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


//=============================================================================
// GET AND SET
//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale a joint based on XYZ scale factors for the bodies.
 *
 * @param aScaleSet Set of XYZ scale factors for the bodies.
 * @todo Need to scale transforms appropriately, given an arbitrary axis.
 */
void CustomJoint::scale(const ScaleSet& aScaleSet)
{
    Vec3 scaleFactors(1.0);

    // Joint knows how to scale locations of the joint in parent and on the body
    Super::scale(aScaleSet);

    // SCALING TO DO WITH THE PARENT BODY -----
    // Joint kinematics are scaled by the scale factors for the
    // parent body, so get those body's factors
    const string& parentName = getParentFrame().getName();
    for (int i=0; i<aScaleSet.getSize(); i++) {
        Scale& scale = aScaleSet.get(i);
        if (scale.getSegmentName()==parentName) {
            scale.getScaleFactors(scaleFactors);
            break;
        }
    }

    updSpatialTransform().scale(scaleFactors);
}

void CustomJoint::constructCoordinates()
{
    const SpatialTransform& spatialTransform = getSpatialTransform();
    OpenSim::Array<std::string> coordinateNames
        = spatialTransform.getCoordinateNames();

    // Check how many coordinates are required
    int ncoords = coordinateNames.getSize();

    for (int i = 0; i < ncoords; ++i){
        std::string coordName = spatialTransform.getCoordinateNames()[i];
        // Locate the coordinate in the set if it has already been defined (e.g. in XML) 
        int coordIndex = getProperty_coordinates().findIndexForName(coordName);
        if (coordIndex < 0) {
            coordIndex = constructCoordinate(Coordinate::MotionType::Undefined,
                                             i);
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
                    // if displacement on axis is linear (w/ slope of 1) w.r.t.
                    // the coordinate value, we have a pure rotation/translation
                    if (lf && lf->getSlope() == 1.0) {
                        // coordinate is pure axis displacement
                        if (j < 3)
                            // coordinate about rotational axis 
                            mt = Coordinate::MotionType::Rotational;
                        else // otherwise translational axis 
                            mt = Coordinate::MotionType::Translational;
                    }
                    else { // scaled (slope !=1) or nonlinear relationship means
                           // not a pure rotational or translational for this axis.
                           // designate as Coupled unless already defined as
                           // pure Rotational or Translational about another axis
                        mt = (mt == Coordinate::MotionType::Undefined ?
                            Coordinate::MotionType::Coupled : mt);
                    }
                }
            }
        }
        setMotionType(CoordinateIndex(i), mt);
    }
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
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

    const Array<std::string>& coordNames = getSpatialTransform().getCoordinateNames();

    // Some initializations
    const int numCoords = numCoordinates();  // Note- should check that all coordinates are used.
    std::vector<std::vector<int> > coordinateIndices =
        getSpatialTransform().getCoordinateIndices();
    std::vector<const SimTK::Function*> functions =
        getSpatialTransform().getFunctions();
    std::vector<Vec3> axes = getSpatialTransform().getAxes();

    SimTK_ASSERT1(numCoords == coordNames.getSize(),
        "%s list of coordinates does not match number of mobilities.",
        getConcreteClassName().c_str());

    SimTK_ASSERT1(numCoords > 0,
        "%s must have 1 or more mobilities (dofs).",
                  getConcreteClassName().c_str());

    SimTK_ASSERT1(numCoords <= 6,
        "%s cannot exceed 6 mobilities (dofs).",
        getConcreteClassName().c_str());
    assert(functions.size() == 6);

    SimTK_ASSERT2(numCoords <= 6,
        "%s::%s must specify functions for complete spatial (6 axes) motion.",
        getConcreteClassName().c_str(),
                  getSpatialTransform().getConcreteClassName().c_str());
    assert(coordinateIndices.size() == 6);

    SimTK_ASSERT2(axes.size() == 6,
        "%s::%s must specify 6 independent axes to span spatial motion.",
        getConcreteClassName().c_str(), getSpatialTransform().getConcreteClassName().c_str());

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
// Utilities for versioning
//=============================================================================
//_____________________________________________________________________________

/** Override of the default implementation to account for versioning. */
void CustomJoint::
updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    int documentVersion = versionNumber;
    if ( documentVersion < XMLDocument::getLatestVersion()){
        if (Object::getDebugLevel()>=1)
            cout << "Updating CustomJoint to latest format..." << endl;
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
                string objectType = objElmt.getElementTag();
                // (sherm) this is cleaning up old TransformAxis here but
                // that should really be done in TransformAxis instead.
                if (objectType == "TransformAxis"){
                    OpenSim::TransformAxis* readAxis = 
                        new OpenSim::TransformAxis(objElmt);
                    assert(nextAxis <=5);
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
            assert(rotationIndices.getSize() <=3);
            assert(translationIndices.getSize() <=3);
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
