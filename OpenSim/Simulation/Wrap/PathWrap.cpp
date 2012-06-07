// PathWrap.cpp
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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
#include "PathWrap.h"
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/Model/GeometryPath.h>

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
PathWrap::PathWrap() :
	Object(),
	_wrapObjectName(_wrapObjectNameProp.getValueStr()),
	_methodName(_methodNameProp.getValueStr()),
   _range(_rangeProp.getValueIntArray())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
PathWrap::~PathWrap()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPathWrap PathWrap to be copied.
 */
PathWrap::PathWrap(const PathWrap& aPathWrap) :
	Object(aPathWrap),
	_wrapObjectName(_wrapObjectNameProp.getValueStr()),
	_methodName(_methodNameProp.getValueStr()),
   _range(_rangeProp.getValueIntArray())
{
	setNull();
	setupProperties();
	copyData(aPathWrap);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this PathWrap to their null values.
 */
void PathWrap::setNull()
{
	_method = hybrid;

	resetPreviousWrap();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PathWrap::setupProperties()
{
	_wrapObjectNameProp.setName("wrap_object");
	_propertySet.append(&_wrapObjectNameProp);

	_methodNameProp.setName("method");
	_methodNameProp.setValue("Unassigned");
	_propertySet.append(&_methodNameProp);

	const int defaultRange[] = {-1, -1};
	_rangeProp.setName("range");
	_rangeProp.setValue(2, defaultRange);
	_rangeProp.setAllowableListSize(2);
	_propertySet.append(&_rangeProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 * @param s current state of  system
 * @param aModel pointer to the OpenSim model.
 */
void PathWrap::connectToModelAndPath(const Model& aModel, GeometryPath& aPath)
{
	int i;
	const BodySet& bodySet = aModel.getBodySet();

	_path = &aPath;

	for (i = 0; i < bodySet.getSize(); i++) {
		WrapObject* wo = bodySet.get(i).getWrapObject(getWrapObjectName());
		if (wo) {
			_wrapObject = wo;
			_wrapPoints[0].setBody(bodySet.get(i));
			_wrapPoints[0].setWrapObject(wo);
			_wrapPoints[1].setBody(bodySet.get(i));
			_wrapPoints[1].setWrapObject(wo);
			break;
		}
	}

	// connectToModelAndPath() must be called after setBody() because it requires
	// that _bodyName already be assigned.
	_wrapPoints[0].connectToModelAndPath(aModel, aPath);
	_wrapPoints[1].connectToModelAndPath(aModel, aPath);

	if (_methodName == "hybrid" || _methodName == "Hybrid" || _methodName == "HYBRID")
		_method = hybrid;
	else if (_methodName == "midpoint" || _methodName == "Midpoint" || _methodName == "MIDPOINT")
		_method = midpoint;
	else if (_methodName == "axial" || _methodName == "Axial" || _methodName == "AXIAL")
		_method = axial;
	else if (_methodName == "Unassigned") {  // method was not specified in wrap object definition; use default
		_method = hybrid;
		_methodName = "hybrid";
	} else {  // method was specified incorrectly in wrap object definition; throw an exception
		string errorMessage = "Error: wrapping method for wrap object " + getName() + " was either not specified, or specified incorrectly.";
		throw Exception(errorMessage);
	}
}

//_____________________________________________________________________________
/**
 * Copy data members from one PathWrap to another.
 *
 * @param aPathWrap PathWrap to be copied.
 */
void PathWrap::copyData(const PathWrap& aPathWrap)
{
	_wrapObjectName = aPathWrap._wrapObjectName;
	_methodName = aPathWrap._methodName;
	_method = aPathWrap._method;
	_range = aPathWrap._range;
	_wrapObject = aPathWrap._wrapObject;
	_previousWrap = aPathWrap._previousWrap;

	_wrapPoints[0] = aPathWrap._wrapPoints[0];
	_wrapPoints[1] = aPathWrap._wrapPoints[1];
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
PathWrap& PathWrap::operator=(const PathWrap& aPathWrap)
{
	// BASE CLASS
	Object::operator=(aPathWrap);

	copyData(aPathWrap);

	return(*this);
}

PathWrapPoint& PathWrap::getWrapPoint(int aIndex)
{
	if (aIndex < 0 || aIndex > 1)
	{
		// TODO string errorMessage = "PathWrap::getWrapPoint(): invalid index (" + aIndex + ")";
		// throw Exception(errorMessage);
	}

	return _wrapPoints[aIndex];
}

void PathWrap::setStartPoint( const SimTK::State& s, int aIndex)
{
	if ((aIndex != _range[0]) && (aIndex == -1 || _range[1] == -1 || (aIndex >= 1 && aIndex <= _range[1])))
	{
		_range[0] = aIndex;
		_rangeProp.setValueIsDefault(false);
	}
}

void PathWrap::setEndPoint( const SimTK::State& s, int aIndex)
{
	if ((aIndex != _range[1]) && (aIndex == -1 || _range[0] == -1 || (aIndex >= _range[0] && aIndex <= _path->getPathPointSet().getSize())))
	{
		_range[1] = aIndex;
		_rangeProp.setValueIsDefault(false);
	}
}

void PathWrap::resetPreviousWrap()
{
	_previousWrap.startPoint = -1;
	_previousWrap.endPoint = -1;

	_previousWrap.wrap_pts.setSize(0);
	_previousWrap.wrap_path_length = 0.0;

	int i;
	for (i = 0; i < 3; i++) {
		_previousWrap.r1[i] = -std::numeric_limits<SimTK::Real>::infinity();
		_previousWrap.r2[i] = -std::numeric_limits<SimTK::Real>::infinity();
		_previousWrap.sv[i] = -std::numeric_limits<SimTK::Real>::infinity();
	}
}

void PathWrap::setPreviousWrap(const WrapResult& aWrapResult)
{
	_previousWrap = aWrapResult;
}

void PathWrap::setWrapObject(WrapObject& aWrapObject)
{
	_wrapObject = &aWrapObject;
	_wrapObjectName = aWrapObject.getName();
}

void PathWrap::setMethod(WrapMethod aMethod)
{
	if (aMethod == axial) {
		_method = axial;
		_methodName = "axial";
	} else if (aMethod == midpoint) {
		_method = midpoint;
		_methodName = "midpoint";
	} else if (aMethod == hybrid) {
		_method = hybrid;
		_methodName = "hybrid";
	}
}
