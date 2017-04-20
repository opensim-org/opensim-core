/* -------------------------------------------------------------------------- *
 *                           SimbodySimmDof.cpp                               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, P2C HD065690, U54 EB020405)   *
 * and by DARPA through the Warrior Web program.                              *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <fstream>
#include <iostream>
#include <string>
#include <math.h>
#include <float.h>
#include <time.h>

#include "SimbodySimmDof.h"


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SimbodySimmDof::~SimbodySimmDof()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimbodySimmDof::SimbodySimmDof()
{
   setNull();
}

//_____________________________________________________________________________
/**
 * Set NULL values for all the variable members of this class.
 */
void SimbodySimmDof::setNull()
{
    _name = "undefined";
   _type = Coordinate::Translational;
    _userFunctionNumber = -1;
   _coordinateName = "";
   _axis[0] = _axis[1] = _axis[2] = 0.0;
}

//_____________________________________________________________________________
/**
 * Set values for a constant DOF
 *
 * @param aName Name of the DOF.
 * @param aType Type of the DOF (rotational, translational).
 * @param aAxis Axis of rotation (not used for translations).
 * @param aValue Value of the constant DOF.
 */
void SimbodySimmDof::setConstant(const string& aName, Coordinate::MotionType aType,  const double* aAxis, double aValue)
{
   _name = aName;
   _type = aType;
   _userFunctionNumber = -1;
   _coordinateName = "";
   if (aAxis != NULL) {
      _axis[0] = aAxis[0];
      _axis[1] = aAxis[1];
      _axis[2] = aAxis[2];
   }
   _value = aValue;
}

//_____________________________________________________________________________
/**
 * Set values for a DOF that is a function of a gencoord
 *
 * @param aName Name of the DOF.
 * @param aType Type of the DOF (rotational, translational).
 * @param aFunctionNumber User-number of the function controlling this DOF.
 * @param aCoordinateName Name of the gencoord that this DOF is a function of.
 * @param aAxis Axis of rotation (not used for translations).
 */
void SimbodySimmDof::setFunction(const string& aName, Coordinate::MotionType aType, int aFunctionNumber,
                                 const string& aCoordinateName, const double* aAxis)
{
   _name = aName;
   _type = aType;
   _userFunctionNumber = aFunctionNumber;
   _coordinateName = aCoordinateName;
   if (aAxis != NULL) {
      _axis[0] = aAxis[0];
      _axis[1] = aAxis[1];
      _axis[2] = aAxis[2];
   }
}

//_____________________________________________________________________________
/**
 * Write the DOF to a [SIMM joint] file.
 *
 * @param aStream File to write to.
 */
void SimbodySimmDof::write(ofstream& aStream)
{
   if (_userFunctionNumber >= 0) {
      aStream << _name << " function f" << _userFunctionNumber << "(" << _coordinateName << ")" << endl;
   } else {
      aStream << _name << " constant " << _value << endl;
   }
   if (_type == Coordinate::Rotational)
      aStream << "axis" << _name[1] << " " << _axis[0] << " " << _axis[1] << " " << _axis[2] << endl;
}

//_____________________________________________________________________________
/**
 * Return the DOF's axis.
 *
 * @param rAxis The axis.
 */
void SimbodySimmDof::getAxis(double rAxis[]) const
{
    rAxis[0] = _axis[0];
    rAxis[1] = _axis[1];
    rAxis[2] = _axis[2];
}
