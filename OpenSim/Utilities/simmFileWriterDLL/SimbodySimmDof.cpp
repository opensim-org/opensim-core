// SimbodySimmDof.cpp
// Authors: Peter Loan
/*
 * Copyright (c)  2008, Stanford University. All rights reserved. 
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
