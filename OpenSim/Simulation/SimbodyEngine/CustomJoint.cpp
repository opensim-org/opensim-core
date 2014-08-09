/* -------------------------------------------------------------------------- *
 *                         OpenSim:  CustomJoint.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/LinearFunction.h>


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
/**
 * Default constructor.
 */
CustomJoint::CustomJoint() : Super()
{
    constructProperties();
}
//_____________________________________________________________________________
/**
 * Constructor with specified SpatialTransform.
 */
CustomJoint::CustomJoint(const std::string &name, const OpenSim::Body &parent,
    const SimTK::Vec3& locationInParent, const SimTK::Vec3& orientationInParent,
    const OpenSim::Body& child,
    const SimTK::Vec3& locationInchild, const SimTK::Vec3& orientationInChild,
    SpatialTransform& aSpatialTransform, bool reverse) :
        Super(name, parent, locationInParent, orientationInParent,
                child, locationInchild, orientationInChild, reverse)
{
    constructProperties();
    set_SpatialTransform(aSpatialTransform);

    constructCoordinates();
    updSpatialTransform().connectToJoint(*this);
}

//_____________________________________________________________________________
/**
 * Convenience Constructor; use default SpatialTransform.
 */
CustomJoint::CustomJoint(const std::string &name, const OpenSim::Body &parent,
    const SimTK::Vec3& locationInParent, const SimTK::Vec3& orientationInParent,
    const OpenSim::Body& child,
    const SimTK::Vec3& locationInchild, const SimTK::Vec3& orientationInChild,
    bool reverse) : 
        Super(name, parent, locationInParent,orientationInParent,
            child, locationInchild, orientationInChild, reverse)
{
    constructProperties();

    constructCoordinates();
    updSpatialTransform().connectToJoint(*this);
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

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this CustomJoint.
 */
void CustomJoint::connectToModel(Model& aModel)
{
    Super::connectToModel(aModel);

    /* Set up spatial transform for this custom joint. */
    updSpatialTransform().connectToJoint(*this);

    // Reorder the coordinates so that they match their order in the SpatialTransform.
    int nc = getCoordinateSet().getSize();

    // Form an array of pointers to the coordinates in the order that they are
    // referenced in the SpatialTransform. All of the coordinate names should be
    // valid because connectToJoint() has already been called on the 
    // SpatialTransform.
    Array<Coordinate*> coords;
    for (int i=0; i<6; i++) {
        const TransformAxis& transform = getSpatialTransform()[i];
        for (int j=0; j<transform.getCoordinateNames().size(); j++) {
            Coordinate* c = &(upd_CoordinateSet().get(transform.getCoordinateNames()[j]));
            // If the coordinate is not already in the array, add it.
            if (coords.findIndex(c) == -1)
                coords.append(c);
        }
    }
    // Now remove all the coordinates from the set (without deleting),
    // and add them back in in the right order.
    bool currentOwnership = getCoordinateSet().getMemoryOwner();
    upd_CoordinateSet().setMemoryOwner(false);
    for (int i=0; i<nc; i++)
        upd_CoordinateSet().remove(0);
    upd_CoordinateSet().setMemoryOwner(currentOwnership);
    for (int i=0; i<coords.getSize(); i++)
        upd_CoordinateSet().adoptAndAppend(coords[i]);
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
    const string& parentName = getParentBody().getName();
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
    CoordinateSet& coordinateSet = upd_CoordinateSet();

    // In case we're making a copy, we need to restore "properties" from 
    // _coordinateSet after we construct them anew
    CoordinateSet savedCoordinates = coordinateSet;

    coordinateSet.setSize(0);

    // Check how many coordinates are required
    int ncoords = getSpatialTransform().getCoordinateNames().getSize();

    for(int i = 0; i< ncoords ; i++){
        Coordinate *coord = new Coordinate();
        std::string coordName = getSpatialTransform().getCoordinateNames()[i];
        coord->setName(coordName);
        int coordIndex = savedCoordinates.getIndex(coordName);
        if (coordIndex!=-1){
            Coordinate& origCoord = savedCoordinates.get(coordIndex);
            coord->setDefaultValue(origCoord.getDefaultValue());
            coord->setDefaultSpeedValue(origCoord.getDefaultSpeedValue());
            coord->setRangeMin(origCoord.getRangeMin());
            coord->setRangeMax(origCoord.getRangeMax());
            coord->setMotionType(origCoord.getMotionType());
            coord->setDefaultClamped(origCoord.getDefaultClamped());
            coord->setDefaultLocked(origCoord.getDefaultLocked());
            coord->setDefaultIsPrescribed(origCoord.getDefaultIsPrescribed());
            if (origCoord.getDefaultIsPrescribed())
                coord->setPrescribedFunction(origCoord.getPrescribedFunction());
        }
        coordinateSet.adoptAndAppend(coord);
    }
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void CustomJoint::addToSystem(SimTK::MultibodySystem& system) const
{
    SimTK::MobilizedBody inb;
    SimTK::Body outb;
    const SimTK::Transform* inbX = &getParentTransform();
    const SimTK::Transform* outbX = &getChildTransform();
    const OpenSim::Body* mobilized = &getChildBody();
    // if the joint is reversed then flip the underlying tree representation
    // of inboard and outboard bodies, although the joint direction will be 
    // preserved, the inboard must exist first.
    if (get_reverse()){
        inb = system.updMatterSubsystem().updMobilizedBody(
            getChildBody().getIndex());
        inbX = &getChildTransform();

        outb = getParentInternalRigidBody();
        outbX = &getParentTransform();

        mobilized = &getParentBody();
    }
    else{
        inb = system.updMatterSubsystem().updMobilizedBody(
            getParentBody().getIndex());
        outb = getChildInternalRigidBody();
    }

    const CoordinateSet& coords = get_CoordinateSet();
    // Some initializations
    int numMobilities = coords.getSize();  // Note- should check that all coordinates are used.
    std::vector<std::vector<int> > coordinateIndices =
        getSpatialTransform().getCoordinateIndices();
    std::vector<const SimTK::Function*> functions =
        getSpatialTransform().getFunctions();
    std::vector<Vec3> axes = getSpatialTransform().getAxes();

    SimTK_ASSERT1(numMobilities > 0,
        "%s must have 1 or more mobilities (dofs).",
                  getConcreteClassName().c_str());
    SimTK_ASSERT1(numMobilities <= 6,
        "%s cannot exceed 6 mobilities (dofs).",
        getConcreteClassName().c_str());
    assert(functions.size() == 6);
    SimTK_ASSERT2(numMobilities <= 6,
        "%s::%s must specify functions for complete spatial (6 axes) motion.",
        getConcreteClassName().c_str(),
                  getSpatialTransform().getConcreteClassName().c_str());
    assert(coordinateIndices.size() == 6);
    SimTK_ASSERT2(axes.size() == 6,
        "%s::%s must specify 6 independent axes to span spatial motion.",
        getConcreteClassName().c_str(), getSpatialTransform().getConcreteClassName().c_str());

    SimTK::MobilizedBody::Direction dir =
        SimTK::MobilizedBody::Direction(get_reverse());

    SimTK::MobilizedBody::FunctionBased
        simtkBody(inb, *inbX, 
                  outb, *outbX, 
                  numMobilities, functions,
                  coordinateIndices, axes, dir);

    int nc = numCoordinates();

    SimTK_ASSERT1(nc == numMobilities, "%s list of coordinates does not match number of mobilities.",
        getConcreteClassName().c_str());

    assignSystemIndicesToBodyAndCoordinates(simtkBody, mobilized, nc, 0);
    
    // TODO: Joints require super class to be called last.
    Super::addToSystem(system);
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
            int objectsFound = 0;
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

    const CoordinateSet& coordinateSet = get_CoordinateSet();

    // Fix coordinate type post deserialization
    // Fix coordinate type post deserialization
    for (int i=0; i<coordinateSet.getSize(); i++){
        OpenSim::Coordinate& nextCoord = coordinateSet.get(i);
        // Find TransformAxis for the coordinate and use it to set Coordinate's motionType
        for(int axisIndex=0; axisIndex<6; axisIndex++){
            const TransformAxis& nextAxis = getSpatialTransform()[axisIndex];
            const Property<std::string>& coordNames = nextAxis.getCoordinateNames();
            if (coordNames.findIndex(nextCoord.getName())!=-1){
                coordinateSet.get(i).setMotionType((axisIndex>2)? 
                        Coordinate::Translational : Coordinate::Rotational);
                break;
            }
        }
    }
    // Axes should be independent otherwise Simbody throws an exception in addToSystem
    double tol = 1e-5;
    // Verify that none of the rotation axes are colinear
    const std::vector<SimTK::Vec3> axes=getSpatialTransform().getAxes();
    for(int startIndex=0; startIndex<=3; startIndex+=3){
        if(((axes[startIndex+0]%axes[startIndex+1]).norm() < tol)||
            ((axes[startIndex+0]%axes[startIndex+2]).norm() < tol)||
            ((axes[startIndex+1]%axes[startIndex+2]).norm() < tol)){
                throw(Exception("CustomJoint " + getName() + 
                    " has colinear axes and so is not well-defined."
                    " Please fix and retry loading.."));
        }
    }
    updProperty_SpatialTransform().setValueIsDefault(false);
}
