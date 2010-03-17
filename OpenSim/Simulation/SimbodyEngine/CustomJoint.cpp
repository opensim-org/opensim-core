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
#include <iostream>
#include <math.h>
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/XMLNode.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/LinearFunction.h>
#include "CustomJoint.h"
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>

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
 * Destructor.
 */
CustomJoint::~CustomJoint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
CustomJoint::CustomJoint() :
	Joint(),
	_spatialTransformProp(PropertyObj("", SpatialTransform())),
	_spatialTransform((SpatialTransform&)_spatialTransformProp.getValueObj())
{
	setNull();
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Constructor.
 */
	CustomJoint::CustomJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
					OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody,
					SpatialTransform &aSpatialTransform, bool reverse) :
	Joint(name, parent, locationInParent,orientationInParent,
			body, locationInBody, orientationInBody,reverse),
	_spatialTransformProp(PropertyObj("", SpatialTransform())),
	_spatialTransform((SpatialTransform&)_spatialTransformProp.getValueObj())
{
	setNull();
	setupProperties();

	_spatialTransform = aSpatialTransform;
	_body->setJoint(*this);
	constructCoordinates();
	_spatialTransform.setup(*this);
}

//_____________________________________________________________________________
/**
 * Convenience Constructor.
 */
	CustomJoint::CustomJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
					OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody, bool reverse) :
	Joint(name, parent, locationInParent,orientationInParent,
			body, locationInBody, orientationInBody, reverse),
	_spatialTransformProp(PropertyObj("", SpatialTransform())),
	_spatialTransform((SpatialTransform&)_spatialTransformProp.getValueObj())
{
	setNull();
	setupProperties();

	_body->setJoint(*this);
	constructCoordinates();
	_spatialTransform.setup(*this);
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aJoint CustomJoint to be copied.
 */
CustomJoint::CustomJoint(const CustomJoint &aJoint) :
   Joint(aJoint),
	_spatialTransformProp(PropertyObj("", SpatialTransform())),
	_spatialTransform((SpatialTransform&)_spatialTransformProp.getValueObj())
{
	setNull();
	setupProperties();
	copyData(aJoint);
	constructCoordinates();
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy this body and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this OpenSim::Body.
 */
Object* CustomJoint::copy() const
{
	CustomJoint *joint = new CustomJoint(*this);
	return(joint);
}
//_____________________________________________________________________________
/**
 * Copy data members from one CustomJoint to another.
 *
 * @param aJoint CustomJoint to be copied.
 */
void CustomJoint::copyData(const CustomJoint &aJoint)
{
	Joint::copyData(aJoint);
	_spatialTransform = aJoint._spatialTransform;
}

//_____________________________________________________________________________
/**
 * Set the data members of this CustomJoint to their null values.
 */
void CustomJoint::setNull()
{
	setType("CustomJoint");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void CustomJoint::setupProperties()
{
	_spatialTransformProp.setName("SpatialTransform");
	_propertySet.append(&_spatialTransformProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this CustomJoint.
 */
void CustomJoint::setup(Model& aModel)
{
	// Base class
	Joint::setup(aModel);

	/* Set up spatial transform for this custom joint. */
	_spatialTransform.setup(*this);

	// Reorder the coordinates so that they match their order in the SpatialTransform.
	{
		int nc = getCoordinateSet().getSize();

		// Form an array of pointers to the coordinates in the order that they are
		// referenced in the SpatialTransform. All of the coordinate names should be
		// valid because setup() has already been called on the SpatialTransform.
		Array<Coordinate*> coords;
		for (int i=0; i<6; i++) {
			for (int j=0; j<_spatialTransform[i].getCoordinateNames().getSize(); j++) {
				Coordinate* c = &(getCoordinateSet().get(_spatialTransform[i].getCoordinateNames().get(j)));
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
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
CustomJoint& CustomJoint::operator=(const CustomJoint &aJoint)
{
	Joint::operator=(aJoint);
	copyData(aJoint);
	return(*this);
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

	_spatialTransform.scale(scaleFactors);
}

void CustomJoint::constructCoordinates()
{
	// In case we're making a copy, we need to restore "properties" from 
	// _coorindateSet after we construct them anew
	CoordinateSet savedCoordinates = _coordinateSet;

	_coordinateSet.setSize(0);

	// Check how many coordinates are required
	int ncoords = _spatialTransform.getCoordinateNames().getSize();

	for(int i = 0; i< ncoords ; i++){
		Coordinate *coord = new Coordinate();
		std::string coordName = _spatialTransform.getCoordinateNames()[i];
		coord->setName(coordName);
		int coordIndex = savedCoordinates.getIndex(coordName);
		if (coordIndex!=-1){
			Coordinate& origCoord = savedCoordinates.get(coordIndex);
			coord->setRangeMin(origCoord.getRangeMin());
			coord->setRangeMax(origCoord.getRangeMax());
			coord->setMotionType(origCoord.getMotionType());
			coord->setDefaultClamped(origCoord.getDefaultClamped());
			coord->setDefaultLocked(origCoord.getDefaultLocked());
		}
		_coordinateSet.append(coord);
	}
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void CustomJoint::createSystem(SimTK::MultibodySystem& system) const
{
	// CHILD TRANSFORM
	Rotation rotation(BodyRotationSequence, _orientation[0],XAxis, _orientation[1],YAxis, _orientation[2],ZAxis);
	SimTK::Transform childTransform(rotation,_location);

	// PARENT TRANSFORM
	Rotation parentRotation(BodyRotationSequence,_orientationInParent[0],XAxis,_orientationInParent[1],YAxis,_orientationInParent[2],ZAxis);
	SimTK::Transform parentTransform(parentRotation, _locationInParent);

	// Some initializations
	int numMobilities = _coordinateSet.getSize();  // Note- should check that all coordinates are used.
	std::vector<std::vector<int> > coordinateIndices = _spatialTransform.getCooridinateIndices();
	std::vector<const SimTK::Function*> functions = _spatialTransform.getFunctions();
	std::vector<Vec3> axes = _spatialTransform.getAxes();

	// CREATE MOBILIZED BODY
	MobilizedBody::FunctionBased
		simtkBody(system.updMatterSubsystem().updMobilizedBody(getMobilizedBodyIndex(_parentBody)),
			parentTransform,SimTK::Body::Rigid(_body->getMassProperties()),
			childTransform,numMobilities,functions,coordinateIndices,axes, (getReverse() ? MobilizedBody::Reverse : MobilizedBody::Forward));
	setMobilizedBodyIndex(_body, simtkBody.getMobilizedBodyIndex());

    // Let the superclass do its construction.
    Joint::createSystem(system);
}

//=============================================================================
// Utilities for versioning
//=============================================================================
//_____________________________________________________________________________

/** Override of the default implementation to account for versioning. */
void CustomJoint::updateFromXMLNode()
{
	int documentVersion = getDocument()->getDocumentVersion();
	if ( documentVersion < XMLDocument::getLatestVersion()){
		if (Object::getDebugLevel()>=1)
			cout << "Updating CustomJoint to latest format..." << endl;
		// Version before refactoring spatialTransform
		if (_node!=NULL && documentVersion<10901){
			// replace TransformAxisSet with SpatialTransform
			/////renameChildNode("TransformAxisSet", "SpatialTransform");
			// Check how many TransformAxes are defined
			DOMElement* SpatialTransformNode = XMLNode::GetFirstChildElementByTagName(_node,"TransformAxisSet");
			if (SpatialTransformNode == NULL) return;

			DOMElement*axesSetNode = XMLNode::GetFirstChildElementByTagName(SpatialTransformNode,"objects");
			/////if (axesSetNode != NULL)
			/////	SpatialTransformNode->removeChild(axesSetNode);
			/////DOMElement*grpNode = XMLNode::GetFirstChildElementByTagName(SpatialTransformNode,"groups");
			/////if (grpNode != NULL)
			/////	SpatialTransformNode->removeChild(grpNode);
			DOMElement* SpatialTransformAxesNode = axesSetNode;			// Get (at most six) TransformAxis nodes and move them to proper slots 
			// (0, 1, 2 rotations & 3, 4, 5 translations then remove the is_rotation node)
			DOMNodeList *list = SpatialTransformAxesNode->getChildNodes();
			unsigned int listLength = list->getLength();
			int objectsFound = 0;
			Array<int> translationIndices(-1, 0);
			Array<int>  rotationIndices(-1, 0);
			int nextAxis = 0;
			std::vector<TransformAxis *> axes;
			// Add children for all six axes here
			//////_node->removeChild(SpatialTransformNode);
			
			// Create a blank Spatial Transform and use it to populate the XML structure
			for(int i=0; i<6; i++)
				_spatialTransform[i].setFunction(new OpenSim::Constant(0));
			
			Array<OpenSim::TransformAxis*> oldAxes;
			for(unsigned int j=0;j<listLength;j++) {
				// getChildNodes() returns all types of DOMNodes including comments, text, etc., but we only want
				// to process element nodes
				if (!list->item(j) || (list->item(j)->getNodeType() != DOMNode::ELEMENT_NODE)) continue;
				DOMElement *objElmt = (DOMElement*) list->item(j);
				string objectType = XMLNode::TranscodeAndTrim(objElmt->getTagName());
				if (objectType == "TransformAxis"){
					OpenSim::TransformAxis* readAxis = new OpenSim::TransformAxis(objElmt);
					assert(nextAxis <=5);
					bool isRotation = false;
					DOMElement* rotationNode = XMLNode::GetFirstChildElementByTagName(objElmt,"is_rotation");
					if (rotationNode){
						DOMText* txtNode=NULL;
						if(rotationNode && (txtNode=XMLNode::GetTextNode(rotationNode))) {
							// Could still be empty or whiteSpace
							string transcoded = XMLNode::TranscodeAndTrim(txtNode->getNodeValue());
							if (transcoded.length()>0){
								bool value = XMLNode::GetValue<bool>(rotationNode);
								isRotation = value;
							}
						}
						objElmt->removeChild(rotationNode);
					}
					DOMElement* coordinateNode = XMLNode::GetFirstChildElementByTagName(objElmt,"coordinate");
					std::string cooridnateName = XMLNode::GetValue<std::string>(coordinateNode);
					Array<std::string> names("");
					names.append(cooridnateName);
					readAxis->setCoordinateNames(names);
					/*
					Array<double> coeffs;
					coeffs.append(1.0);	coeffs.append(0.0);
					readAxis->setFunction(new LinearFunction(coeffs));
					*/
					DOMElement* axisNode = XMLNode::GetFirstChildElementByTagName(objElmt,"axis");
					
					double *axisVec;
					XMLNode::GetDblArray(axisNode, axisVec);
					readAxis->setAxis(SimTK::Vec3(axisVec));
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
			XMLNode::RemoveChildren(SpatialTransformAxesNode);
			int nRotations = rotationIndices.getSize();
			int nTranslations = translationIndices.getSize();
			// Now copy coordinateName, Axis, Function into proper slot
			for (int i=0; i<nRotations; i++){
				_spatialTransform[i].copyData(*(axes[rotationIndices[i]]));
			}
			_spatialTransform.constructIndepndentAxes(nRotations, 0);
			// Add Translations from old list then pad with default ones, make sure no singularity
			for (int i=0; i<nTranslations; i++){
				_spatialTransform[i+3].copyData(*(axes[translationIndices[i]]));
			}
			_spatialTransform.constructIndepndentAxes(nTranslations, 3);
		}
	}
	Joint::updateFromXMLNode();
	// Fix coordinate type post deserialization
	// Fix coordinate type post deserialization
	for (int i=0; i<_coordinateSet.getSize(); i++){
		OpenSim::Coordinate& nextCoord=_coordinateSet.get(i);
		// Find TransformAxis for the coordinate and use it to set Coordinate's motionType
		for(int axisIndex=0; axisIndex<6; axisIndex++){
			TransformAxis& nextAxis = _spatialTransform[axisIndex];
			const Array<std::string>& coordNames = nextAxis.getCoordinateNames();
			if (coordNames.findIndex(nextCoord.getName())!=-1){
				_coordinateSet.get(i).setMotionType((axisIndex>2)? 
						Coordinate::Translational : Coordinate::Rotational);
				break;
			}
		}
	}
}
