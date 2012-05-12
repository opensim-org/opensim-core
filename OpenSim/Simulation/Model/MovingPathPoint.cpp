// MovingPathPoint.cpp
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
#include <OpenSim/Common/XMLDocument.h>
#include "MovingPathPoint.h"
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/MultiplierFunction.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/GeometryPath.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
MovingPathPoint::MovingPathPoint() :
   _xLocation(_xLocationProp.getValueObjPtrRef()),
	_xCoordinateName(_xCoordinateNameProp.getValueStr()),
   _yLocation(_yLocationProp.getValueObjPtrRef()),
	_yCoordinateName(_yCoordinateNameProp.getValueStr()),
   _zLocation(_zLocationProp.getValueObjPtrRef()),
	_zCoordinateName(_zCoordinateNameProp.getValueStr())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
MovingPathPoint::~MovingPathPoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPoint MovingPathPoint to be copied.
 */
MovingPathPoint::MovingPathPoint(const MovingPathPoint &aPoint) :
   PathPoint(aPoint),
   _xLocation(_xLocationProp.getValueObjPtrRef()),
	_xCoordinateName(_xCoordinateNameProp.getValueStr()),
   _yLocation(_yLocationProp.getValueObjPtrRef()),
	_yCoordinateName(_yCoordinateNameProp.getValueStr()),
   _zLocation(_zLocationProp.getValueObjPtrRef()),
	_zCoordinateName(_zCoordinateNameProp.getValueStr())
{
	setNull();
	setupProperties();
	copyData(aPoint);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one MovingPathPoint to another.
 *
 * @param aPoint MovingPathPoint to be copied.
 */
void MovingPathPoint::copyData(const MovingPathPoint &aPoint)
{
	_xLocation = (Function*)Object::SafeCopy(aPoint._xLocation);
	_xCoordinateName = aPoint._xCoordinateName;
	_yLocation = (Function*)Object::SafeCopy(aPoint._yLocation);
	_yCoordinateName = aPoint._yCoordinateName;
	_zLocation = (Function*)Object::SafeCopy(aPoint._zLocation);
	_zCoordinateName = aPoint._zCoordinateName;
}

//_____________________________________________________________________________
/**
 * Initialize a MovingPathPoint with data from a PathPoint.
 *
 * @param aPoint PathPoint to be copied.
 */
void MovingPathPoint::init(const PathPoint& aPoint)
{
	PathPoint::copyData(aPoint);

	// If aPoint is a MovingPathPoint, then you can copy all of its members over.
	// Otherwise, create new functions for X, Y, and Z so that the point starts
	// out in a reasonable state.
   const MovingPathPoint* mmp = dynamic_cast<const MovingPathPoint*>(&aPoint);
	if (mmp) {
		copyData(*mmp);
	} else {
		double x[2], y[2];

		// See if you can find a coordinate to use as the default for this point.
		const Coordinate* coord = NULL;
		GeometryPath* path = aPoint.getPath();
		if (path) {
			ModelComponent* comp = dynamic_cast<ModelComponent*>(path->getOwner());
			if (comp) {
				const CoordinateSet& coordSet = comp->getModel().getCoordinateSet();
				if (coordSet.getSize() > 0) {
					int index = 0;
					coord = &coordSet.get(index);
				}
			}
		}

		// If there is a coordinate, use its range as the range for each function.
		if (coord) {
			x[0] = coord->getRangeMin();
			x[1] = coord->getRangeMax();
		} else {
			x[0] = 0.0;
			x[1] = 1.0;
		}

		// Create the default X component.
		if (coord)
			_xCoordinateName = coord->getName();
		y[0] = y[1] = aPoint.getLocation()[0];
		_xLocation = new SimmSpline(2, x, y);

		// Create the default Y component.
		if (coord)
			_yCoordinateName = coord->getName();
		y[0] = y[1] = aPoint.getLocation()[1];
		_yLocation = new SimmSpline(2, x, y);

		// Create the default Z component.
		if (coord)
			_zCoordinateName = coord->getName();
		y[0] = y[1] = aPoint.getLocation()[2];
		_zLocation = new SimmSpline(2, x, y);
	}
}

//_____________________________________________________________________________
/**
 * Set the data members of this MovingPathPoint to their null values.
 */
void MovingPathPoint::setNull()
{
	_xCoordinate = NULL;
	_yCoordinate = NULL;
	_zCoordinate = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MovingPathPoint::setupProperties()
{
	_xLocationProp.setName("x_location");
	_propertySet.append(&_xLocationProp);

	_xCoordinateNameProp.setName("x_coordinate");
	_propertySet.append(&_xCoordinateNameProp);

	_yLocationProp.setName("y_location");
	_propertySet.append(&_yLocationProp);

	_yCoordinateNameProp.setName("y_coordinate");
	_propertySet.append(&_yCoordinateNameProp);

	_zLocationProp.setName("z_location");
	_propertySet.append(&_zLocationProp);

	_zCoordinateNameProp.setName("z_coordinate");
	_propertySet.append(&_zCoordinateNameProp);
}

//_____________________________________________________________________________
/**
 * Override default implementation by Object to intercept and fix the XML node
 * underneath the MovingPathPoint to match the current version.
 */
void MovingPathPoint::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
	int documentVersion = versionNumber;
	if (documentVersion < XMLDocument::getLatestVersion()) {
		if (Object::getDebugLevel()>=1)
			cout << "Updating MovingPathPoint object to latest format..." << endl;
		XMLDocument::renameChildNode(aNode, "XAttachment", "x_location");
		XMLDocument::renameChildNode(aNode,"YAttachment", "y_location");
		XMLDocument::renameChildNode(aNode,"ZAttachment", "z_location");
		}
	
	// Call base class now assuming _node has been corrected for current version
	Object::updateFromXMLNode(aNode, versionNumber);
}

//_____________________________________________________________________________
/**
 * Set the coordinate used for the X location function.
 *
 * @param aCoordinate the coordinate to set to.
 */
void MovingPathPoint::setXCoordinate( const SimTK::State& s, Coordinate& aCoordinate)
{
	if (&aCoordinate != _xCoordinate) {
	   _xCoordinate = &aCoordinate;
	   _xCoordinateName = _xCoordinate->getName();
	}
}

//_____________________________________________________________________________
/**
 * Set the coordinate used for the Y location function.
 *
 * @param aCoordinate the coordinate to set to.
 */
void MovingPathPoint::setYCoordinate( const SimTK::State& s, Coordinate& aCoordinate)
{
	if (&aCoordinate != _yCoordinate) {
	   _yCoordinate = &aCoordinate;
	   _yCoordinateName = _yCoordinate->getName();
	}
}

//_____________________________________________________________________________
/**
 * Set the coordinate used for the Z location function.
 *
 * @param aCoordinate the coordinate to set to.
 */
void MovingPathPoint::setZCoordinate( const SimTK::State& s, Coordinate& aCoordinate)
{
	if (&aCoordinate != _zCoordinate) {
	   _zCoordinate = &aCoordinate;
	   _zCoordinateName = _zCoordinate->getName();
	}
}

//_____________________________________________________________________________
/**
 * Set the function used for the X location.
 *
 * @param aFunction the function to set to.
 */
void MovingPathPoint::setXFunction( const SimTK::State& s, OpenSim::Function& aFunction)
{
	_xLocation = &aFunction;

}

//_____________________________________________________________________________
/**
 * Set the function used for the Y location.
 *
 * @param aFunction the function to set to.
 */
void MovingPathPoint::setYFunction( const SimTK::State& s, OpenSim::Function& aFunction)
{
	_yLocation = &aFunction;

}

//_____________________________________________________________________________
/**
 * Set the function used for the Z location.
 *
 * @param aFunction the function to set to.
 */
void MovingPathPoint::setZFunction( const SimTK::State& s, OpenSim::Function& aFunction)
{
	_zLocation = &aFunction;

}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this MovingPathPoint.
 */
void MovingPathPoint::setup(const Model& aModel, GeometryPath& aPath)
{
	// base class
	PathPoint::setup(aModel, aPath);

	// Look up the coordinates by name in the dynamics engine and
	// store pointers to them.
    if (aModel.getCoordinateSet().contains(_xCoordinateName))
        _xCoordinate = &aModel.getCoordinateSet().get(_xCoordinateName);
    if (aModel.getCoordinateSet().contains(_yCoordinateName))
        _yCoordinate = &aModel.getCoordinateSet().get(_yCoordinateName);
    if (aModel.getCoordinateSet().contains(_zCoordinateName))
        _zCoordinate = &aModel.getCoordinateSet().get(_zCoordinateName);
	// Update the XYZ location of the point (stored in _location).
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
MovingPathPoint& MovingPathPoint::operator=(const MovingPathPoint &aPoint)
{
	// BASE CLASS
	PathPoint::operator=(aPoint);

	copyData(aPoint);

	return(*this);
}

//_____________________________________________________________________________
/**
 * Update the point's location.
 *
 */
void MovingPathPoint::update(const SimTK::State& s)
{
	if (_xCoordinate)
		_location[0] = _xLocation->calcValue(SimTK::Vector(1, _xCoordinate->getValue(s)));
	else // type == Constant
		_location[0] = _xLocation->calcValue(SimTK::Vector(1, 0.0));

	if (_yCoordinate)
		_location[1] = _yLocation->calcValue(SimTK::Vector(1, _yCoordinate->getValue(s)));
	else // type == Constant
		_location[1] = _yLocation->calcValue(SimTK::Vector(1, 0.0));

	if (_zCoordinate)
		_location[2] = _zLocation->calcValue(SimTK::Vector(1, _zCoordinate->getValue(s)));
	else // type == Constant
		_location[2] = _zLocation->calcValue(SimTK::Vector(1, 0.0));
}

//_____________________________________________________________________________
/**
 * Get the velocity of the point in the body's local reference frame.
 *
 * @param aVelocity The velocity.
 */

void MovingPathPoint::getVelocity(const SimTK::State& s, SimTK::Vec3& aVelocity)
{
	std::vector<int> derivComponents;
	derivComponents.push_back(0);

	if (_xCoordinate){
		//Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
		aVelocity[0] = _xLocation->calcDerivative(derivComponents, SimTK::Vector(1, _xCoordinate->getValue(s)))*
			_xCoordinate->getSpeedValue(s);
	}
	else
		aVelocity[0] = 0.0;

	if (_yCoordinate){
		//Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
		aVelocity[1] = _yLocation->calcDerivative(derivComponents, SimTK::Vector(1, _yCoordinate->getValue(s)))*
			_yCoordinate->getSpeedValue(s);
	}
	else
		aVelocity[1] = 0.0;

	if (_zCoordinate){
		//Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
		aVelocity[2] = _zLocation->calcDerivative(derivComponents, SimTK::Vector(1, _zCoordinate->getValue(s)))*
			_zCoordinate->getSpeedValue(s);
	}
	else
		aVelocity[2] = 0.0;
}

void MovingPathPoint::scale(const SimTK::State& s, const SimTK::Vec3& aScaleFactors)
{
	if (_xCoordinate) {
		// If the function is already a MultiplierFunction, just update its scale factor.
		// Otherwise, make a MultiplierFunction from it and make the muscle point use
		// the new MultiplierFunction.
		MultiplierFunction* mf = dynamic_cast<MultiplierFunction*>(_xLocation);
		if (mf) {
			mf->setScale(mf->getScale() * aScaleFactors[0]);
		} else {
			// Make a copy of the original function and delete the original
			// (so its node will be removed from the XML document).
			mf = new MultiplierFunction(_xLocation->clone(), aScaleFactors[0]);
			delete _xLocation;
			setXFunction(s, *mf);
		}
	}

	if (_yCoordinate) {
		// If the function is already a MultiplierFunction, just update its scale factor.
		// Otherwise, make a MultiplierFunction from it and make the muscle point use
		// the new MultiplierFunction.
		MultiplierFunction* mf = dynamic_cast<MultiplierFunction*>(_yLocation);
		if (mf) {
			mf->setScale(mf->getScale() * aScaleFactors[1]);
		} else {
			// Make a copy of the original function and delete the original
			// (so its node will be removed from the XML document).
			mf = new MultiplierFunction(_yLocation->clone(), aScaleFactors[1]);
			delete _yLocation;
			setYFunction(s, *mf);
		}
	}

	if (_zCoordinate) {
		// If the function is already a MultiplierFunction, just update its scale factor.
		// Otherwise, make a MultiplierFunction from it and make the muscle point use
		// the new MultiplierFunction.
		MultiplierFunction* mf = dynamic_cast<MultiplierFunction*>(_zLocation);
		if (mf) {
			mf->setScale(mf->getScale() * aScaleFactors[2]);
		} else {
			mf = new MultiplierFunction(_zLocation->clone(), aScaleFactors[2]);
			delete _zLocation;
			setZFunction(s, *mf);
		}
	}

	updateGeometry();
}
