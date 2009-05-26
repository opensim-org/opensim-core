// CoordinateCouplerConstraint.cpp
// Author: Ajay Seth, Frank C. Anderson
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
#include <OpenSim/Simulation/Model/BodySet.h>

#include "CustomJoint.h"
#include "TransformAxis.h"
#include "CoordinateCouplerConstraint.h"

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
CoordinateCouplerConstraint::~CoordinateCouplerConstraint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
CoordinateCouplerConstraint::CoordinateCouplerConstraint() :
	Constraint(),
	_function(_functionProp.getValueObjPtrRef()),
	_independentCoordNames(_independentCoordNamesProp.getValueStrArray()),
	_dependentCoordName(_dependentCoordNameProp.getValueStr())
{
	setNull();
	setupProperties();
	updateSimbody();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aConstraint CoordinateCouplerConstraint to be copied.
 */
CoordinateCouplerConstraint::CoordinateCouplerConstraint(const CoordinateCouplerConstraint &aConstraint) :
   Constraint(aConstraint),
	_function(_functionProp.getValueObjPtrRef()),
	_independentCoordNames(_independentCoordNamesProp.getValueStrArray()),
	_dependentCoordName(_dependentCoordNameProp.getValueStr())
{
	setNull();
	setupProperties();
	copyData(aConstraint);
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
Object* CoordinateCouplerConstraint::copy() const
{
	CoordinateCouplerConstraint *constraint = new CoordinateCouplerConstraint(*this);
	return(constraint);
}
//_____________________________________________________________________________
/**
 * Copy data members from one CoordinateCouplerConstraint to another.
 * 
 * @param aConstraint CoordinateCouplerConstraint to be copied.
 */
void CoordinateCouplerConstraint::copyData(const CoordinateCouplerConstraint &aConstraint)
{
	Constraint::copyData(aConstraint);
	// Create a fresh copy (deep copy)
	if (aConstraint._function!=NULL){
		_function = (Function*)(aConstraint._function->copy());
		_simtkCouplerFunction = new CompoundFunction(_function->createSimTKFunction());
	}
	_independentCoordNames = aConstraint._independentCoordNames;
	_dependentCoordName = aConstraint._dependentCoordName;
}

//_____________________________________________________________________________
/**
 * Set the data members of this CoordinateCouplerConstraint to their null values.
 */
void CoordinateCouplerConstraint::setNull()
{
	setType("CoordinateCouplerConstraint");
	_dynamicsEngine = NULL;
	_simtkCouplerFunction = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void CoordinateCouplerConstraint::setupProperties()
{
	// Coordinate Coupler Function
	_functionProp.setName("coupled_coordinates_function");
	_propertySet.append(&_functionProp);

	// coordinates that are coupled (by name)
	_independentCoordNamesProp.setName("independent_coordinate_names");
	_propertySet.append(&_independentCoordNamesProp);

		// coordinates that are coupled (by name)
	_dependentCoordNameProp.setName("dependent_coordinate_name");
	_propertySet.append(&_dependentCoordNameProp);
}

//_____________________________________________________________________________
/**
 * Update the underlying Simbody parameters if necessary
 */
void CoordinateCouplerConstraint::updateSimbody()
{

}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this CoordinateCouplerConstraint.
 */
void CoordinateCouplerConstraint::setup(AbstractDynamicsEngine* aEngine)
{
	string errorMessage;

	// Make sure bodies and coordinates lists are empty
	_bodies.clear();
	_coordinates.clear();

	// Base class
	AbstractConstraint::setup(aEngine);
	//_engine = dynamic_cast<SimbodyEngine*>(aEngine);

	// Look up the bodies and coordinates being coupled by name in the
	// dynamics engine and keep lists of their indices
	for(int i=0; i<_independentCoordNames.getSize(); i++){
		Coordinate *aCoordinate = dynamic_cast<OpenSim::Coordinate*>(aEngine->getCoordinateSet()->get(_independentCoordNames[i]));
		if (aCoordinate==NULL){
			errorMessage = "Coordinate coupler: unknown independent coordinate " ;
			errorMessage += _independentCoordNames[i];
			throw (Exception(errorMessage));
		}
		_bodies.push_back(aCoordinate->_bodyIndex);
		_coordinates.push_back(SimTK::MobilizerQIndex(aCoordinate->_mobilityIndex));
	}

	// Last coordinate in the coupler is the dependent coordinate
	Coordinate *aCoordinate = dynamic_cast<OpenSim::Coordinate*>(aEngine->getCoordinateSet()->get(_dependentCoordName));
	if (aCoordinate==NULL){
		errorMessage = "Coordinate coupler: unknown dependent coordinate " ;
		errorMessage += _dependentCoordName;
		throw (Exception(errorMessage));
	}
	_bodies.push_back(aCoordinate->_bodyIndex);
	_coordinates.push_back(SimTK::MobilizerQIndex(aCoordinate->_mobilityIndex));

	if (!_coordinates.size() & (_coordinates.size() != _bodies.size())) {
		errorMessage = "Coordinate coupler requires at least one body and coordinate." ;
		throw (Exception(errorMessage));
	}

	// Create and set the underlying coupler constraint function;
	_simtkCouplerFunction = new CompoundFunction(_function->createSimTKFunction());

	// Now create a Simbody Constraint::CoordinateCoupler
	SimTK::Constraint::CoordinateCoupler simtkCoordinateCoupler( getEngine()->_system->updMatterSubsystem() ,
																 _simtkCouplerFunction, 
																 _bodies, _coordinates);

	// Get the constraint index so we can access the SimTK::Constraint later 
	_index = simtkCoordinateCoupler.getConstraintIndex();
}

//=============================================================================
// GET AND SET
//=============================================================================
void CoordinateCouplerConstraint::setFunction(OpenSim::Function* aFunction)
{
	_function = aFunction;
}

//=============================================================================
// SCALE
//=============================================================================
/**
 * Scale the coordinate coupler constraint according to the mobilized body that
 * the dependent coordinate belongs too. The scale factor is determined by 
 * dotting the coordinate axis with that of the translation. Rotations are NOT
 * scaled.
 *
 * @param aEngine dynamics engine containing this CoordinateCouplerConstraint.
 */
void CoordinateCouplerConstraint::scale(const ScaleSet& aScaleSet)
{
	Coordinate *depCoordinate = dynamic_cast<OpenSim::Coordinate*>(getEngine()->getCoordinateSet()->get(_dependentCoordName));
	
	// Only scale if the dependent coordinate is a translation
	if (depCoordinate->getMotionType() == AbstractTransformAxis::Translational){
		// Constraint scale factor
		double scaleFactor = 1.0;
		// Get appropriate scale factors from parent body
		Vec3 bodyScaleFactors(1.0); 
		const string& parentName = depCoordinate->_joint->getParentName();

		// Cycle through the scale set to get the appropriate factors
		for (int i=0; i<aScaleSet.getSize(); i++) {
			Scale *scale = aScaleSet.get(i);
			if (scale->getSegmentName()==parentName) {
				scale->getScaleFactors(bodyScaleFactors);
				break;
			}
		}

		// Assume unifrom scaling unless proven otherwise
		scaleFactor = bodyScaleFactors[0];

		// We can handle non-unifrom scaling along transform axes of custom joints ONLY at this time
		CustomJoint *joint =  dynamic_cast<CustomJoint*>(depCoordinate->_joint);
		// Simplifies things if we have uniform scaling so check first
		// TODO: Non-uniform scaling below has not been exercised! - ASeth
		if(joint && bodyScaleFactors[0] != bodyScaleFactors[1] ||  bodyScaleFactors[0] != bodyScaleFactors[2] )
		{
			// Get the coordinate axis defined on the parent body
			Vec3 xyzEuler;
			joint->getOrientationInParent(xyzEuler);
			Rotation orientInParent(BodyRotationSequence,xyzEuler[0],XAxis,xyzEuler[1],YAxis,xyzEuler[2],ZAxis);
			TransformAxisSet* axisSet = joint->getTransformAxisSet();
			Vec3 axis;

			// Cycle through the scale set to get the appropriate factors
			for (int i=0; i<axisSet->getSize(); i++) {
				TransformAxis *transformAxis = dynamic_cast<TransformAxis*>(axisSet->get(i));
				if (transformAxis->getCoordinateName()==parentName) {
					transformAxis->getAxis(axis);
				break;
				}
			}

			Vec3 depCoordAxisInParent = ~orientInParent*axis;

			scaleFactor = ~bodyScaleFactors*axis;
		}

		// scale the user-defined OpenSim 
		_function->scaleY(scaleFactor);

		// set the underlying function for the constraint
		_simtkCouplerFunction->setFunction(_function->createSimTKFunction());
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
CoordinateCouplerConstraint& CoordinateCouplerConstraint::operator=(const CoordinateCouplerConstraint &aConstraint)
{
	AbstractConstraint::operator=(aConstraint);
	copyData(aConstraint);
	return(*this);
}