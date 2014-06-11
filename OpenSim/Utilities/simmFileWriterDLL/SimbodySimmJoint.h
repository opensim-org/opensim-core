#ifndef __SimbodySimmJoint_h__
#define __SimbodySimmJoint_h__
// SimbodySimmJoint.h
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

// INCLUDES
#include <iostream>
#include <string>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>

#include "SimbodySimmDof.h"

namespace OpenSim {

class Joint;
class TransformAxis;

static std::string _translationNames[3] = {"tx","ty","tz"};
static std::string _rotationNames[3] = {"r1","r2","r3"};

//=============================================================================
//=============================================================================
/**
 * A class to hold a body in a SimbodySimmModel.
 *
 * @authors Peter Loan
 * @version 1.0
 */
class SimbodySimmJoint
{

//=============================================================================
// DATA
//=============================================================================
protected:
   std::string _name;
   std::string _parentBodyName;
   std::string _childBodyName;

   std::string _order;  // DOF order (e.g., "t r2 r1 r3")

   SimbodySimmDof _dof[6];

	bool _dofUsed[6];
   int _rotationsUsed;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~SimbodySimmJoint();
   SimbodySimmJoint(const std::string& aName, const std::string& aParentBodyName,
                    const std::string& aChildBodyName);
   void setNull();
   bool addFunctionDof(const SimTK::Vec3& aAxis, const std::string& aCoordinateName,
		int aFunctionNumber, Coordinate::MotionType aMotionType);
   bool addConstantDof(const std::string& aName, const double* aAxis, double aValue);
   void updateOrder(const std::string& aDofName);
	void makeUniqueAxis(int aDofIndex, double rAxis[]) const;
   void finalize();
   void write(std::ofstream& aStream);
   const std::string& getName() const { return _name; }
   void setName(const std::string& aName);

//=============================================================================
};	// END of class SimbodySimmJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimbodySimmJoint_h__


