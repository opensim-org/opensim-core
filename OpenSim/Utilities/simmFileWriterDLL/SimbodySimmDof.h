#ifndef __SimbodySimmDof_h__
#define __SimbodySimmDof_h__
/* -------------------------------------------------------------------------- *
 *                           SimbodySimmDof.h                                 *
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


// INCLUDES
#include <iostream>
#include <string>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Function.h>

#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class to hold a DOF in a SimbodySimmJoint.
 *
 * @authors Peter Loan
 * @version 1.0
 */
class SimbodySimmDof
{

//=============================================================================
// DATA
//=============================================================================
protected:
   std::string _name; // name of the DOF (e.g., "tx", "r1")

   Coordinate::MotionType _type;

   int _userFunctionNumber; // -1 means the DOF is a constant

   std::string _coordinateName;

   double _value;   // used for constants, not functions
   double _axis[3]; // used for rotations, not translations

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION AND DESTRUCTION
    //--------------------------------------------------------------------------
public:
    virtual ~SimbodySimmDof();
    SimbodySimmDof();
   const std::string& getName() const { return _name; }
   void setConstant(const std::string& aName, Coordinate::MotionType aType,
                    const double* aAxis, double aValue);
   void setFunction(const std::string& aName, Coordinate::MotionType aType, int aFunctionNumber,
                    const std::string& aCoordinateName, const double* aAxis);
   void setNull();
   void write(std::ofstream& aStream);
    void getAxis(double rAxis[]) const;

//=============================================================================
};  // END of class SimbodySimmDof
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimbodySimmDof_h__


