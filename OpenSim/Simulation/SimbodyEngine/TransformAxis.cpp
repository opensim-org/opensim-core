// TransformAxis.cpp
// Author: Peter Loan, Frank C. Anderson, Jeffrey A. Reinbolt, Ajay Seth
/*
 * Copyright (c)  2006-2007, Stanford University. All rights reserved. 
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
#include "TransformAxis.h"
#include "Joint.h"
#include "Coordinate.h"
#include "SimbodyEngine.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/Function.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
TransformAxis::TransformAxis() :
	Object(),
    _function(_functionProp.getValueObjPtrRef()),
    _coordinateNames((Array<std::string>&)_coordinateNamesProp.getValueStrArray()),
	_axis(_axisProp.getValueDblVec())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor.
 */
	TransformAxis::TransformAxis(const Array<string> &coordNames, const SimTK::Vec3& aAxis) :
    Object(),
	_function(_functionProp.getValueObjPtrRef()),
    _coordinateNames((Array<std::string>&)_coordinateNamesProp.getValueStrArray()),
	_axis(_axisProp.getValueDblVec())
{
	setNull();
	setupProperties();
	setCoordinateNames(coordNames);
	setAxis(aAxis);
}
/**
 * Constructor from XML node
 */
TransformAxis::TransformAxis(SimTK::Xml::Element& aNode) :
    _function(_functionProp.getValueObjPtrRef()),
    _coordinateNames((Array<std::string>&)_coordinateNamesProp.getValueStrArray()),
	_axis(_axisProp.getValueDblVec())
{
	setNull();
	setupProperties();
	updateFromXMLNode(aNode);
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
TransformAxis::~TransformAxis()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param anAxis TransformAxis to be copied.
 */
TransformAxis::TransformAxis(const TransformAxis &anAxis) :
    Object(anAxis),
    _function(_functionProp.getValueObjPtrRef()),
    _coordinateNames((Array<std::string>&)_coordinateNamesProp.getValueStrArray()),
	_axis(_axisProp.getValueDblVec())
{
	setNull();
	setupProperties();
	copyData(anAxis);
}
//_____________________________________________________________________________
/**
 * Copy data members from one TransformAxis to another.
 *
 * @param anAxis TransformAxis to be copied.
 */
void TransformAxis::copyData(const TransformAxis &anAxis)
{
	setFunction((OpenSim::Function*)Object::SafeCopy(anAxis._function));
    _coordinateNames.setSize(0);
	_coordinateNames.append(anAxis._coordinateNames);
    _joint = anAxis._joint;
    _axis = anAxis._axis;
}

//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this dof.
 * @param aJoint joint containing this dof.
 */
void TransformAxis::setup(Joint& aJoint)
{
	string errorMessage;

	_joint = &aJoint;

	// Look up the coordinates by name
	int nc = _coordinateNames.getSize();
	SimTK::Vector workX(nc, 0.0);
	CoordinateSet& coords = _joint->getCoordinateSet();

	for(int i =0; i< nc; i++){
		try{
			// If coordinate is not in the set an excpetion is thrown
			Coordinate& coord = coords.get(_coordinateNames.get(i));
		}
		catch(Exception &){
			// no coordinate is OK, unless the transform for this axis is governed by a function.
			if (hasFunction()){
				errorMessage += "Invalid coordinate (" + _coordinateNames.get(i) + ") specified for TransformAxis " + getName() + " in joint " + aJoint.getName();
				throw (Exception(errorMessage));
			}
		}
	}
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================

//_____________________________________________________________________________
/**
 * Set the data members of this TransformAxis to their null values.
 */
void TransformAxis::setNull()
{
    _joint = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void TransformAxis::setupProperties()
{
    _functionProp.setName("function");
    _propertySet.append(&_functionProp);

    _coordinateNamesProp.setName("coordinates");
    _propertySet.append(&_coordinateNamesProp);

	// Transformation axis
	const SimTK::Vec3 defaultAxis(1.0, 0.0, 0.0);
	_axisProp.setName("axis");
	_axisProp.setValue(defaultAxis);
	_propertySet.append(&_axisProp);
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
TransformAxis& TransformAxis::operator=(const TransformAxis &anAxis)
{
	// BASE CLASS
	Object::operator=(anAxis);

	copyData(anAxis);

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================

//_____________________________________________________________________________
/**
 * Set the transformation axis.
 *
 * @param aAxis Transformation axis.
 */
void TransformAxis::
setAxis(const SimTK::Vec3& aAxis)
{
	_axis = aAxis;
}
//_____________________________________________________________________________
/**
 * Get the transformation axis.
 *
 * @param rAxis Transformation axis is returned here.
 */
void TransformAxis::
getAxis(SimTK::Vec3& rAxis) const
{
	rAxis = _axis;
}

//_____________________________________________________________________________
/**
 * Get the current value of the transform.
 *
 * @return Current value of the transform.
 */
double TransformAxis::getValue(const SimTK::State& s )
{
	int nc = _coordinateNames.getSize();
	SimTK::Vector workX(nc, 0.0);
	CoordinateSet& coords = _joint->getCoordinateSet();

	for(int i =0; i< nc; i++){
		workX[i] = coords.get(_coordinateNames.get(i)).getValue(s);
	}

	return _function->calcValue(workX);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the names of the generalized coordinates that disctate the motion
 * of freedom.
 *
 * @param coordNames Names of the generalized coordinates.
 */
void TransformAxis::setCoordinateNames(const Array<string>& coordNames)
{
	_coordinateNames = coordNames;
}

//_____________________________________________________________________________
/**
 * Utility to return the function that this dof uses.
 *
 * @return Function used by this dof.
 */
OpenSim::Function& TransformAxis::getFunction() const
{
    if (_function == NULL)
        throw Exception("TransformAxis::getFunction(): no Function is defined");
	return *_function;
}

bool TransformAxis::hasFunction() const
{
    return (_function != NULL);
}

//_____________________________________________________________________________
/**
 * Utility to change the function that this dof uses.
 *
 * @param aFunction the function to change to.
 */
void TransformAxis::setFunction(OpenSim::Function* aFunction)
{
    if (_function != NULL)
        delete _function;
	_function = aFunction;
}

void TransformAxis::setFunction(const OpenSim::Function& aFunction)
{
	_function = aFunction.clone();
}


void TransformAxis::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
	// Version before refactoring spatialTransform
	//if (_node!=NULL){
	//	renameChildNode("coordinate", "coordinates");
	//}
	Object::updateFromXMLNode(aNode, versionNumber);
}