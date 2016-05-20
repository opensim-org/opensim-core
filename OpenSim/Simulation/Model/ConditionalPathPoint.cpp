/* -------------------------------------------------------------------------- *
 *                     OpenSim:  ConditionalPathPoint.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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
#include "ConditionalPathPoint.h"
#include <OpenSim/Common/XMLDocument.h>
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
    // Otherwise, set the range and coordinate name to default values (using the first
    // coordinate in the model, if there is one).
   const ConditionalPathPoint* mmp = dynamic_cast<const ConditionalPathPoint*>(&aPoint);
    if (mmp) {
        copyData(*mmp);
    } else {
        _range[0] = 0.0;
        _range[1] = 0.0;
        GeometryPath* path = aPoint.getPath();
        if (path) {
            ModelComponent* comp = dynamic_cast<ModelComponent*>(path->getOwner());
            if (comp) {
                const CoordinateSet& coords = comp->getModel().getCoordinateSet();
                if (coords.getSize() > 0) {
                    int index = 0;
                    _coordinateName = coords.get(index).getName();
                    _range[0] = coords.get(index).getRangeMin();
                    _range[1] = coords.get(index).getRangeMax();
                }
            }
        }
    }
}

//_____________________________________________________________________________
/**
 * Set the data members of this ConditionalPathPoint to their null values.
 */
void ConditionalPathPoint::setNull()
{
}

//_____________________________________________________________________________
/**
 * Override default implementation by object to intercept and fix the XML node
 * underneath the model to match current version
 */
void ConditionalPathPoint::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    if ( versionNumber < XMLDocument::getLatestVersion()){
        if (versionNumber<=20001)
        // Version has to be 1.6 or later, otherwise assert
        XMLDocument::renameChildNode(aNode, "coordinates", "coordinate");
        }
    // Call base class now assuming _node has been corrected for current version
    PathPoint::updateFromXMLNode(aNode, versionNumber);
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
    _rangeProp.setAllowableListSize(2);
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
void ConditionalPathPoint::connectToModelAndPath(Model& aModel, GeometryPath& aPath)
{
    // base class
    Super::connectToModelAndPath(aModel, aPath);

    // Setup() can be called before the model's coordinate set has been constructed, in which
    // case you don't want to throw an exception if the coordinate is not found.
    if (aModel.getCoordinateSet().getSize() > 0) {
        // Look up the coordinate by name and store a pointer to it.
        if (aModel.getCoordinateSet().contains(_coordinateName)) {
            _coordinate = &aModel.getCoordinateSet().get(_coordinateName);
        } else {
            string errorMessage = "Error: Coordinate " + _coordinateName + " referenced in muscle " + aPath.getOwner()->getName() +
                " does not exist in model " +   aModel.getName();
            throw Exception(errorMessage);
        }
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
ConditionalPathPoint& ConditionalPathPoint::operator=(const ConditionalPathPoint &aPoint)
{
    // BASE CLASS
    PathPoint::operator=(aPoint);

    copyData(aPoint);

    return(*this);
}
