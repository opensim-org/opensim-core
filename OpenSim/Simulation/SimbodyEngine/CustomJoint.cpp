// CustomJoint.cpp
// Author: Frank C. Anderson, Peter Loan, Ajay Seth
/*
 * Copyright (c)  2007, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>

#include "CustomJoint.h"

#include <iostream>
#include <cmath>

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
CustomJoint::CustomJoint()
{
	constructProperties();
}
//_____________________________________________________________________________
/**
 * Constructor with specified SpatialTransform.
 */
CustomJoint::CustomJoint(const std::string &name, OpenSim::Body& parent, 
                         SimTK::Vec3 locationInParent, 
                         SimTK::Vec3 orientationInParent,
					     OpenSim::Body& body, SimTK::Vec3 locationInBody, 
                         SimTK::Vec3 orientationInBody,
					     SpatialTransform& aSpatialTransform, bool reverse) 
:	Super(name, parent, locationInParent,orientationInParent,
			body, locationInBody, orientationInBody,reverse)
{
	constructProperties();
	set_SpatialTransform(aSpatialTransform);

	_body->setJoint(*this);
	constructCoordinates();
	updSpatialTransform().connectToJoint(*this);
}

//_____________________________________________________________________________
/**
 * Convenience Constructor; use default SpatialTransform.
 */
CustomJoint::CustomJoint(const std::string &name, OpenSim::Body& parent, 
                         SimTK::Vec3 locationInParent, 
                         SimTK::Vec3 orientationInParent,
					     OpenSim::Body& body, SimTK::Vec3 locationInBody, 
                         SimTK::Vec3 orientationInBody, bool reverse) 
:	Super(name, parent, locationInParent,orientationInParent,
		  body, locationInBody, orientationInBody, reverse)
{
	constructProperties();

	_body->setJoint(*this);
	constructCoordinates();
	updSpatialTransform().connectToJoint(*this);
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void CustomJoint::constructProperties()
{
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
			Coordinate* c = &(getCoordinateSet().get(transform.getCoordinateNames()[j]));
			// If the coordinate is not already in the array, add it.
			if (coords.findIndex(c) == -1)
				coords.append(c);
		}
	}
	// Now remove all the coordinates from the set (without deleting),
	// and add them back in in the right order.
	bool currentOwnership = getCoordinateSet().getMemoryOwner();
	getCoordinateSet().setMemoryOwner(false);
	for (int i=0; i<nc; i++)
		getCoordinateSet().remove(0);
	getCoordinateSet().setMemoryOwner(currentOwnership);
	for (int i=0; i<coords.getSize(); i++)
		getCoordinateSet().append(coords[i]);
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
	Joint::scale(aScaleSet);

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
	// In case we're making a copy, we need to restore "properties" from 
	// _coordinateSet after we construct them anew
	CoordinateSet savedCoordinates = _coordinateSet;

	_coordinateSet.setSize(0);

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
		_coordinateSet.append(coord);
	}
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void CustomJoint::addToSystem(SimTK::MultibodySystem& system) const
{
    // CHILD TRANSFORM
	Rotation rotation(BodyRotationSequence, _orientation[0],XAxis, _orientation[1],YAxis, _orientation[2],ZAxis);
	SimTK::Transform childTransform(rotation,_location);

	// PARENT TRANSFORM
	Rotation parentRotation(BodyRotationSequence,_orientationInParent[0],XAxis,_orientationInParent[1],YAxis,_orientationInParent[2],ZAxis);
	SimTK::Transform parentTransform(parentRotation, _locationInParent);

	// Some initializations
	int numMobilities = _coordinateSet.getSize();  // Note- should check that all coordinates are used.
	std::vector<std::vector<int> > coordinateIndices = 
        getSpatialTransform().getCoordinateIndices();
	std::vector<const SimTK::Function*> functions = 
        getSpatialTransform().getFunctions();
	std::vector<Vec3> axes = getSpatialTransform().getAxes();

	SimTK::MobilizedBodyIndex parentIndex = getMobilizedBodyIndex(_parentBody);
	if (!parentIndex.isValid())
		throw(Exception("CustomJoint " + getName() + " has invalid parent body "
                        +_parentBody->getName()));
	int nu = numMobilities;
	assert(nu > 0);
    assert(nu <= 6);
    assert(functions.size() == 6);
    assert(coordinateIndices.size() == 6);
    assert(axes.size() == 6);
	// CREATE MOBILIZED BODY
	MobilizedBody::FunctionBased
		simtkBody(system.updMatterSubsystem().updMobilizedBody(parentIndex),
			parentTransform,SimTK::Body::Rigid(_body->getMassProperties()),
			childTransform,numMobilities,functions,coordinateIndices,axes, 
            (getReverse() ? MobilizedBody::Reverse : MobilizedBody::Forward));
	setMobilizedBodyIndex(_body, simtkBody.getMobilizedBodyIndex());

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
			/////	spatialTransformNode->removeChild(axesSetNode);
			/////DOMElement*grpNode = XMLNode::GetFirstChildElementByTagName(spatialTransformNode,"groups");
			/////if (grpNode != NULL)
			/////	spatialTransformNode->removeChild(grpNode);
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

	// Fix coordinate type post deserialization
	// Fix coordinate type post deserialization
	for (int i=0; i<_coordinateSet.getSize(); i++){
		OpenSim::Coordinate& nextCoord=_coordinateSet.get(i);
		// Find TransformAxis for the coordinate and use it to set Coordinate's motionType
		for(int axisIndex=0; axisIndex<6; axisIndex++){
			const TransformAxis& nextAxis = getSpatialTransform()[axisIndex];
			const Property<std::string>& coordNames = nextAxis.getCoordinateNames();
			if (coordNames.findIndex(nextCoord.getName())!=-1){
				_coordinateSet.get(i).setMotionType((axisIndex>2)? 
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
}
