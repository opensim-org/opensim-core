#ifndef __SimbodySimmDof_h__
#define __SimbodySimmDof_h__
// SimbodySimmDof.h
// Authors: Peter Loan
/*
 * Copyright (c)  2008, Stanford University. All rights reserved.
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
*   1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
*   2. The software is not distributed or redistributed.  Software distribution is allowed
*     only through https://simtk.org/home/opensim.
*   3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
*   4. Credits to developers may not be removed from executables
*     created from modifications of the source.
*   5. Modifications of source code must retain the above copyright notice, this list of
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


