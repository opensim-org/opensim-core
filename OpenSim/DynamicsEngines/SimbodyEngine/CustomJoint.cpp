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
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Constant.h>
#include "CustomJoint.h"
#include "TransformAxis.h"
#include "SimbodyEngine.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Simulation/Model/BodySet.h>

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
	_transformAxisSetProp(PropertyObj("", TransformAxisSet())),
	_transformAxisSet((TransformAxisSet&)_transformAxisSetProp.getValueObj())
{
	setNull();
	setupProperties();
	updateSimbody();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aJoint CustomJoint to be copied.
 */
CustomJoint::CustomJoint(const CustomJoint &aJoint) :
   Joint(aJoint),
	_transformAxisSetProp(PropertyObj("", TransformAxisSet())),
	_transformAxisSet((TransformAxisSet&)_transformAxisSetProp.getValueObj())
{
	setNull();
	setupProperties();
	copyData(aJoint);
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
	_transformAxisSet = aJoint._transformAxisSet;
}

//_____________________________________________________________________________
/**
 * Set the data members of this CustomJoint to their null values.
 */
void CustomJoint::setNull()
{
	setType("CustomJoint");
	_parentBody = NULL;
	_body = NULL;
	_dynamicsEngine = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void CustomJoint::setupProperties()
{
	_transformAxisSetProp.setName("TransformAxisSet");
	_propertySet.append(&_transformAxisSetProp);
}

//_____________________________________________________________________________
/**
 * Update the underlying SDFast parameters, such as the inboard to joint and
 * body to joint vectors.
 *
 * @return True if the new inboard to joint was set; false otherwise.
 */
void CustomJoint::updateSimbody()
{
	setLocationInParent(_locationInParent);
	setLocation(_location);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this CustomJoint.
 */
void CustomJoint::setup(AbstractDynamicsEngine* aEngine)
{
	string errorMessage;

	// Base class
	AbstractJoint::setup(aEngine);
	//_engine = dynamic_cast<SimbodyEngine*>(aEngine);

	// Look up the parent and child bodies by name in the
	// dynamics engine and store pointers to them.
	_parentBody = dynamic_cast<OpenSim::Body*>(aEngine->getBodySet()->get(_parentName));
	if (!_parentBody) {
		errorMessage += "Invalid parent body (" + _parentName + ") specified in joint " + getName();
		throw (Exception(errorMessage.c_str()));
	}

	/* Set up each of the dofs. */
	int i;
	for(i=0; i<_transformAxisSet.getSize(); i++)
		_transformAxisSet.get(i)->setup(aEngine, this);
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
	AbstractJoint::operator=(aJoint);
	copyData(aJoint);
	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the CustomJoint's forward transform.
 *
 * @return Reference to the forward transform.
 */
const OpenSim::Transform& CustomJoint::getForwardTransform()
{

	return _forwardTransform;
}

//_____________________________________________________________________________
/**
 * Get the CustomJoint's inverse transform.
 *
 * @return Reference to the inverse transform.
 */
const OpenSim::Transform& CustomJoint::getInverseTransform()
{

	return _inverseTransform;
}


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

	// SCALING TO DO WITH THE PARENT BODY -----
	// Joint kinematics are scaled by the scale factors for the
	// parent body, so get those body's factors
	const string& parentName = getParentBody()->getName();
	for (int i=0; i<aScaleSet.getSize(); i++) {
		Scale *scale = aScaleSet.get(i);
		if (scale->getSegmentName()==parentName) {
			scale->getScaleFactors(scaleFactors);
			break;
		}
	}

	// If all three factors are equal to 1.0, do nothing.
	if (EQUAL_WITHIN_ERROR(scaleFactors[0], 1.0) &&
		EQUAL_WITHIN_ERROR(scaleFactors[1], 1.0) &&
		EQUAL_WITHIN_ERROR(scaleFactors[2], 1.0)) return;

	for(int i=0; i<3; i++)
		_locationInParent[i]*= scaleFactors[i];
	// Scale
	for (int i = 0; i < _transformAxisSet.getSize(); i++) {
		if (_transformAxisSet.get(i)->getMotionType() == AbstractTransformAxis::Translational) {
			TransformAxis* transform = (TransformAxis*)_transformAxisSet.get(i);
			Function* function = transform->getFunction();
         if (function) {
			   Vec3 axis;
			   transform->getAxis(axis);
			   double scaleFactor = ~axis * scaleFactors;
			   function->scaleY(scaleFactor);
         }
		}
	}
	getEngine()->setInvalid();
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void CustomJoint::connectBody()
{
	// Joint checks that parent body being connected to is valid.
	Joint::connectBody();

	// CHILD TRANSFORM
	Rotation rotation(BodyRotationSequence, _orientation[0],XAxis, _orientation[1],YAxis, _orientation[2],ZAxis);
	SimTK::Transform childTransform(rotation,_location);

	// PARENT TRANSFORM
	Rotation parentRotation(BodyRotationSequence,_orientationInParent[0],XAxis,_orientationInParent[1],YAxis,_orientationInParent[2],ZAxis);
	SimTK::Transform parentTransform(parentRotation, _locationInParent);

	// COORDINATE SET
	// The order in which the coordinates are listed in the coordinate set
	// only determines the order in which the coordinates will appear in the
	// q array.  The order does not specify the convention used for
	// how the child body transfroms with respect to the parent body;
	// the transform set determines this.

	// Some initializations
	int numMobilities = _coordinateSet.getSize();  // Note- should check that all coordinates are used.
	std::vector<std::vector<int> > coordinateIndices;
	std::vector<const SimTK::Function<1>*> functions;
	std::vector<Vec3> axes;

	// Six functions must be specified.
	// If there is no function, a linear function is assumed.
	// If there is no transform, a constant function is assumed.
	// Three rotations are first.  Three translations are second.
	// The order in which the rotational transforms appear in the
	// transform set specifies the convention for how the child
	// transforms with respect to the parent (e.g., Euler X-Y-Z).
	// Translations are always specified wrt the parent frame, even
	// if they are specified following the rotations.
	// The order in which the coordinates appear in the
	// q array (i.e., the coordinate set) has no bearing on the
	// parrent to child transform.

	// ROTATIONS
	int i;
	TransformAxis *transform;
	int numTransforms = _transformAxisSet.getSize();
	int numRotations = 0;
	for(i=0;i<numTransforms;i++) {
		transform = (TransformAxis*) _transformAxisSet.get(i);
		if(transform->getMotionType() == AbstractTransformAxis::Rotational) {
			numRotations++;
			appendAxisCoordinateIndicesFunctionsForFunctionBasedMobilizer(&_coordinateSet,transform,axes,coordinateIndices,functions);
			}
		if(numRotations>3) {
			cerr<<"SimbodyEngine.connectBodies:  Error- only 3 rotations are allowed."<<endl;
			break;
		}
	}
	for(i=numRotations;i<3;i++) {
		appendAxisCoordinateIndicesFunctionsForFunctionBasedMobilizer(&_coordinateSet,NULL,axes,coordinateIndices,functions);
	}

	// TRANSLATIONS
	int numTranslations = 0;
	for(i=0;i<numTransforms;i++) {
		transform = (TransformAxis*) _transformAxisSet.get(i);
		if(transform->getMotionType() == AbstractTransformAxis::Translational) {
			numTranslations++;
			appendAxisCoordinateIndicesFunctionsForFunctionBasedMobilizer(&_coordinateSet,transform,axes,coordinateIndices,functions);
			}
		if(numTranslations>3) {
			cerr<<"SimbodyEngine.connectBodies:  Error- only 3 translations are allowed."<<endl;
			break;
		}
	}

	for(i=numTranslations;i<3;i++) {
		appendAxisCoordinateIndicesFunctionsForFunctionBasedMobilizer(&_coordinateSet,NULL,axes,coordinateIndices,functions);
	}

	// CREATE MOBILIZED BODY
	MobilizedBody::FunctionBased
		simtkBody(getMultibodySystem(getEngine())->updMatterSubsystem().updMobilizedBody(getMobilizedBodyIndex(_parentBody)),
			parentTransform,SimTK::Body::Rigid(_body->getMassProperties()),
			childTransform,numMobilities,functions,coordinateIndices,axes);
	setMobilizedBodyIndex(_body, simtkBody.getMobilizedBodyIndex());

	//_body->_dynamicsEngine= getEngine();
	associateCoordinatesAndSpeeds();
}

//_____________________________________________________________________________
/**
 * Utility method for appending the axis, coordinate, and function for
 * a function-based mobilizer.
 */
void CustomJoint::
appendAxisCoordinateIndicesFunctionsForFunctionBasedMobilizer(CoordinateSet *coordinateSet,TransformAxis *transform,
	std::vector<Vec3> &axes,
	std::vector<std::vector<int> > &coordinateIndices,
	std::vector<const SimTK::Function<1>*> &functions)
{
	// TRANSFORM
	if(transform) {
		Vec3 axis;
		transform->getAxis(axis);
		axes.push_back(axis);

		// Coordinate index
		// For now, just handle dependence on 1 coordinate.
		// Note that the function-based mobilizers can handle more, however,
		// which is why the index is done as a vector<int>.
		string coordName = transform->getCoordinateName();
		std::vector<int> index(1);
		index[0] = coordinateSet->getIndex(coordName);				
		coordinateIndices.push_back(index);

		// FUNCTION
		OpenSim::Function *func = transform->getFunction();
		if(func!=NULL) {
			functions.push_back(func->createSimTKFunction());

		// NO FUNCTION - LINEAR FUNCTION ASSUMED
		} else {

			// slope = 1.0, intercept = 0.0.
			Vector_<Vec1> coefficients(2);
			coefficients[0] = 1.0;
			coefficients[1] = 0.0;
			functions.push_back(new SimTK::Function<1>::Linear(coefficients));
	
			// SET MOTION TYPE ON COORDINATE
			// By default, the motion type of coordinates is translational
			// If the coordinate drives a rotation, the motion type
			// is set to Rotational.  The first three axes specified
			// are for rotations; the second three are for translations.
			Coordinate *q = (Coordinate*)coordinateSet->get(index[0]);
			if(axes.size()<=3) q->setMotionType(AbstractTransformAxis::Rotational); // Allowed axes.size()==3 since axes are appended before the test!
			}

	// NO TRANSFORM - CONSTANT FUNCTION ASSUMED
	// These transforms are constant and do nothing. They just have to be done because
	// the FunctionBased mobilizer always assumes 6 degrees of freedom.
			} else {
		// The axis direction is not important because there is no movement; however, all
		// the axes do need to be different (i.e., no two axes can be parallel).
		Vec3 axis(1,0,0);
		int size = axes.size();
		// Make up a non-colinear axis for the second axis
		if((size==1)||(size==4)) {
			// Axis 0 and axis 3 represent the first rotational and first translational
			// axes respectively.
			Vec3 axis0 = axes[size-1].abs();
			axis = axis0;

			// find min and max
			int i,imin,imax;
			double max = axis0[0];
			double min = axis0[0];
			for(imin=imax=0,i=1;i<3;i++) {
				if(axis0[i] < min) {
					imin = i;
					min = axis0[i];
			}
				if(axis0[i] >= max) {
					imax = i;
					max = axis0[i];
			}
				}
			axis[imin] = 1.0;
			axis[imax] = 0.0;

		// Use cross product of first two for the third axis
		} else if((size==2)||(size==5)) {
			axis = axes[size-2] % axes[size-1];
		}

		axes.push_back(axis);
		// Size is 0 because there is no coordinate dependence.
		std::vector<int> index(0);
		coordinateIndices.push_back(index);
		// Create a single constant value of 0.0 (Vec1(0.0),...); it depends on no arguments (...,0).
		functions.push_back(new SimTK::Function<1>::Constant(Vec1(0.0),0));  
	}
}