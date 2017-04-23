#ifndef __SimbodySimmJoint_h__
#define __SimbodySimmJoint_h__
/* -------------------------------------------------------------------------- *
 *                            SimbodySimmJoint.h                              *
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
};  // END of class SimbodySimmJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimbodySimmJoint_h__


