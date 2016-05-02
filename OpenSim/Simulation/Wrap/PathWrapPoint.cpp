/* -------------------------------------------------------------------------- *
 *                        OpenSim:  PathWrapPoint.cpp                         *
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
PathWrapPoint::PathWrapPoint()
{
    setNull();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
PathWrapPoint::~PathWrapPoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPoint PathWrapPoint to be copied.
 */
PathWrapPoint::PathWrapPoint(const PathWrapPoint &aPoint) :
   PathPoint(aPoint)
{
    setNull();
    copyData(aPoint);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one PathWrapPoint to another.
 *
 * @param aPoint PathWrapPoint to be copied.
 */
void PathWrapPoint::copyData(const PathWrapPoint &aPoint)
{
    _wrapPath = aPoint._wrapPath;
    _wrapPathLength = aPoint._wrapPathLength;
    _wrapObject = aPoint._wrapObject;
}

//_____________________________________________________________________________
/**
 * Set the data members of this PathWrapPoint to their null values.
 */
void PathWrapPoint::setNull()
{
    _wrapPath.setSize(0);
    _wrapPathLength = 0.0;
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
PathWrapPoint& PathWrapPoint::operator=(const PathWrapPoint &aPoint)
{
    // BASE CLASS
    PathPoint::operator=(aPoint);

    copyData(aPoint);

    return(*this);
}
