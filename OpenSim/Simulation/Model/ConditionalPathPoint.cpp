// ConditionalPathPoint.cpp
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
#include "ConditionalPathPoint.h"
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/XMLNode.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/GeometryPath.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>

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
ConditionalPathPoint::ConditionalPathPoint() :
   _range(_rangeProp.getValueDblArray()),
	_coordinateName(_coordinateNameProp.getValueStr()),
	_coordinate(NULL)
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
ConditionalPathPoint::~ConditionalPathPoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPoint ConditionalPathPoint to be copied.
 */
ConditionalPathPoint::ConditionalPathPoint(const ConditionalPathPoint &aPoint) :
   PathPoint(aPoint),
   _range(_rangeProp.getValueDblArray()),
	_coordinateName(_coordinateNameProp.getValueStr()),
	_coordinate(NULL)
{
	setNull();
	setupProperties();
	copyData(aPoint);
}

//_____________________________________________________________________________
/**
 * Copy this conditional point and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this ConditionalPathPoint.
 */
Object* ConditionalPathPoint::copy() const
{
	ConditionalPathPoint *pt = new ConditionalPathPoint(*this);
	return(pt);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one ConditionalPathPoint to another.
 *
 * @param aPoint ConditionalPathPoint to be copied.
 */
void ConditionalPathPoint::copyData(const ConditionalPathPoint &aPoint)
{
	_range = aPoint._range;
	_coordinateName = aPoint._coordinateName;
	_coordinate = aPoint._coordinate;
}

//_____________________________________________________________________________
/**
 * Initialize a ConditionalPathPoint with data from a PathPoint.
 * This function should not do anything to invalidate the path,
 * because the via point may not be added to it.
 *
 * @param aPoint PathPoint to be copied.
 */
void ConditionalPathPoint::init(const PathPoint& aPoint)
{
	PathPoint::copyData(aPoint);

	// If aPoint is a ConditionalPathPoint, then you can copy all of its members over.
	// Otherwise, set the range to default values and leave the coordinate unassigned.
   const ConditionalPathPoint* mmp = dynamic_cast<const ConditionalPathPoint*>(&aPoint);
	if (mmp) {
		copyData(*mmp);
	} else {
		_range[0] = _range[1] = 0.0;
	}
}

//_____________________________________________________________________________
/**
 * Set the data members of this ConditionalPathPoint to their null values.
 */
void ConditionalPathPoint::setNull()
{
	setType("ConditionalPathPoint");
}

//_____________________________________________________________________________
/**
 * Override default implementation by object to intercept and fix the XML node
 * underneath the model to match current version
 */
void ConditionalPathPoint::updateFromXMLNode()
{
	int documentVersion = getDocument()->getDocumentVersion();
	if ( documentVersion < XMLDocument::getLatestVersion()){
		DOMDocument* document = _node->getOwnerDocument();
		if (Object::getDebugLevel()>=1)
			cout << "Updating ConditionalPathPoint object to latest format..." << endl;
		// Version has to be 1.6 or later, otherwise assert
		if (_node!=NULL){
			renameChildNode("coordinates", "coordinate", _node);
		}
	}
	// Call base class now assuming _node has been corrected for current version
	PathPoint::updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ConditionalPathPoint::setupProperties()
{
	const double defaultRange[] = {0.0, 0.0};
	_rangeProp.setName("range");
	_rangeProp.setValue(2, defaultRange);
	_rangeProp.setAllowableArraySize(2);
	_propertySet.append(&_rangeProp);

	_coordinateNameProp.setName("coordinate");
	_propertySet.append(&_coordinateNameProp);
}

//_____________________________________________________________________________
/**
 * Set the coordinate that this point is linked to.
 *
 * @return Whether or not this point is active.
 */
void ConditionalPathPoint::setCoordinate(const SimTK::State& s, Coordinate& aCoordinate)
{
	if (&aCoordinate != _coordinate)
	{
	   _coordinate = &aCoordinate;
	   _coordinateName = _coordinate->getName();
	}
}

//_____________________________________________________________________________
/**
 * Set the range min.
 *
 * @param aRange range min to change to.
 */
void ConditionalPathPoint::setRangeMin(const SimTK::State& s, double aMin)
{
	if (aMin <= _range[1])
	{
		_range[0] = aMin;
	}
}

//_____________________________________________________________________________
/**
 * Set the range max.
 *
 * @param aRange range max to change to.
 */
void ConditionalPathPoint::setRangeMax( const SimTK::State& s, double aMax)
{
	if (aMax >= _range[0])
	{
		_range[1] = aMax;
	}
}

//_____________________________________________________________________________
/**
 * Determine if this point is active by checking the value of the
 * coordinate that it is linked to.
 *
 * @return Whether or not this point is active.
 */
bool ConditionalPathPoint::isActive(const SimTK::State& s) const
{
	if (_coordinate)
	{
		double value = _coordinate->getValue(s);
		if (value >= _range[0] - 1e-5 &&
			 value <= _range[1] + 1e-5)
			return true;
	}

	return false;
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this ConditionalPathPoint.
 */
void ConditionalPathPoint::setup(const Model& aModel, GeometryPath& aPath)
{
	// base class
	PathPoint::setup(aModel, aPath);

	/* Look up the coordinate by name in the dynamics engine and
	 * store a pointer to it.
	 */
    if (aModel.getCoordinateSet().contains(_coordinateName))
    	_coordinate = &aModel.getCoordinateSet().get(_coordinateName);
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
ConditionalPathPoint& ConditionalPathPoint::operator=(const ConditionalPathPoint &aPoint)
{
	// BASE CLASS
	PathPoint::operator=(aPoint);

	copyData(aPoint);

	return(*this);
}
