// SimbodySimmJoint.cpp
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <math.h>
#include <float.h>
#include <time.h>

#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Simulation/SimbodyEngine/TransformAxis.h>
#include "SimbodySimmJoint.h"


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;

static const double defaultAxes[][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SimbodySimmJoint::~SimbodySimmJoint()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 *
 * @param aName Name of the joint.
 * @param aParentBodyName Name of the parent segment in the joint.
 * @param aChildBodyName Name of the child segment in the joint.
 */
SimbodySimmJoint::SimbodySimmJoint(const string& aName, const string& aParentBodyName,
                                   const string& aChildBodyName)
{
   setNull();
   _name = aName;
   _parentBodyName = aParentBodyName;
   _childBodyName = aChildBodyName;
}

//_____________________________________________________________________________
/**
 * Set NULL values for all the variable members of this class.
 */
void SimbodySimmJoint::setNull()
{
    _name = "undefined";
   _order = "";
    for (int i=0; i<6; i++)
        _dofUsed[i] = false;
   _rotationsUsed = 0;
   _parentBodyName = "";
   _childBodyName = "";
}

void SimbodySimmJoint::setName(const std::string& aName)
{
    _name = aName;
}

//_____________________________________________________________________________
/**
 * Add a DOF that is a function of a gencoord.
 *
 * @param aDof TransformAxis that this DOF is based on.
 * @param aCoordinateName Name of the gencoord that the DOF is a function of.
 * @param aFunctionNumber User-number of the function.
 * @return Whether or not the DOF was added to the joint.
 */
bool SimbodySimmJoint::addFunctionDof(const SimTK::Vec3& aAxis, const string& aCoordinateName,
                                                  int aFunctionNumber, Coordinate::MotionType aMotionType)
{
    double axis[3];
    axis[0] = aAxis[0]; axis[1] = aAxis[1]; axis[2] = aAxis[2];

    if (aMotionType == Coordinate::Translational) {
      int component = -1;
      if (EQUAL_WITHIN_ERROR(axis[0], 1.0))
         component = 0;
      else if (EQUAL_WITHIN_ERROR(axis[1], 1.0))
         component = 1;
      else if (EQUAL_WITHIN_ERROR(axis[2], 1.0))
         component = 2;
      if (component == -1)
         return false;
      // TODO: check if there is something like tx, r1, ty
      _dof[component+3].setFunction(_translationNames[component], aMotionType, aFunctionNumber, aCoordinateName, NULL);
      updateOrder(_translationNames[component]);
      _dofUsed[component+3] = true;
   } else {
        if (_rotationsUsed == 3)
            return false;
        _dof[_rotationsUsed].setFunction(_rotationNames[_rotationsUsed], aMotionType, aFunctionNumber, aCoordinateName, axis);
        updateOrder(_rotationNames[_rotationsUsed]);
        _dofUsed[_rotationsUsed] = true;
        _rotationsUsed++;
   }

   return true;
}

//_____________________________________________________________________________
/**
 * Add a DOF that is a constant.
 *
 * @param aName Name of this DOF.
 * @param aAxis Rotation axis (not used for translational DOFs).
 * @param aValue Value of the DOF.
 * @return Whether or not the DOF was added to the joint.
 */
bool SimbodySimmJoint::addConstantDof(const string& aName, const double* aAxis, double aValue)
{
   for (int i=0; i<3; i++) {
      if (aName == _translationNames[i]) {
         _dof[i+3].setConstant(aName, Coordinate::Translational, NULL, aValue);
         updateOrder(aName);
            _dofUsed[i+3] = true;
         return true;
      }
   }

   //TODO get this to work when constant dofs are mixed with function dofs
   for (int i=0; i<3; i++) {
      if (aName == _rotationNames[i]) {
         _dof[i].setConstant(aName, Coordinate::Rotational, aAxis, aValue);
         updateOrder(aName);
            _dofUsed[i] = true;
            _rotationsUsed++;
      }
   }

   return false;
}

//_____________________________________________________________________________
/**
 * Update the string representing the DOF order with another DOF.
 *
 * @param aDofName Name of the DOF to add to the end of the order.
 */
void SimbodySimmJoint::updateOrder(const string& aDofName)
{
   if (aDofName[0] == 't') {
      if (_dofUsed[3] == true || _dofUsed[4] == true || _dofUsed[5] == true)
         return;
      _order.append(" t");
   } else {
      _order.append(" ");
      _order.append(aDofName);
   }
}

void SimbodySimmJoint::makeUniqueAxis(int aDofIndex, double rAxis[]) const
{
    // If this is the first axis being added to the joint, just use the X axis.
    // If this is the second axis being added, use the Y axis unless the first
    // axis is Y (in which case use X). If this is the third axis, cross the first
    // two to get the third.
    if (aDofIndex == 0) {
        rAxis[0] = defaultAxes[0][0];
        rAxis[1] = defaultAxes[0][1];
        rAxis[2] = defaultAxes[0][2];
    } else if (aDofIndex == 1) {
        double firstAxis[3];
        _dof[0].getAxis(firstAxis);
        if (EQUAL_WITHIN_TOLERANCE(firstAxis[1], 1.0, 0.1)) {
            rAxis[0] = 1.0;
            rAxis[1] = 0.0;
            rAxis[2] = 0.0;
        } else {
            rAxis[0] = 0.0;
            rAxis[1] = 1.0;
            rAxis[2] = 0.0;
        }
    } else {
        double firstAxis[3], secondAxis[3];
        _dof[0].getAxis(firstAxis);
        _dof[1].getAxis(secondAxis);
        rAxis[0] = firstAxis[1]*secondAxis[2] - firstAxis[2]*secondAxis[1];
        rAxis[1] = firstAxis[2]*secondAxis[0] - firstAxis[0]*secondAxis[2];
        rAxis[2] = firstAxis[0]*secondAxis[1] - firstAxis[1]*secondAxis[0];
    }
}

//_____________________________________________________________________________
/**
 * Finalize the joint-- make sure all 6 DOFs are initialized.
 */
void SimbodySimmJoint::finalize()
{
   // Initialize the rotations that are not used. The axes do not matter to SIMM because
    // the DOFs are unused, but if you import the SIMM model back into OpenSim it is a
    // problem to have collinear axes. So as you add [the unused 0.0] DOFs, make sure the
    // axes are unique.
   for (int i=0; i<3; i++) {
        if (!_dofUsed[i]) {
            double axis[3];
            makeUniqueAxis(i, axis);
            _dof[i].setConstant(_rotationNames[i], Coordinate::Rotational, axis, 0.0);
            updateOrder(_dof[i].getName());
        }
   }

   // Initialize the translations that are not used.
   for (int i=0; i<3; i++) {
        if (!_dofUsed[i+3]) {
            _dof[i+3].setConstant(_translationNames[i], Coordinate::Translational, NULL, 0.0);
            updateOrder(_dof[i+3].getName());
            _dofUsed[i+3] = true; // not ideal, but needed for updateOrder
        }
   }
}

//_____________________________________________________________________________
/**
 * Write the Joint to a [SIMM joint] file.
 *
 * @param aStream The file to write to.
 */
void SimbodySimmJoint::write(ofstream& aStream)
{
   aStream << "beginjoint " << _name << endl;
   aStream << "segments " << _parentBodyName << " " << _childBodyName << endl;
   aStream << "order" << _order << endl;
   for (int i=0; i<6; i++)
      _dof[i].write(aStream);
   aStream << "endjoint" << endl << endl;
}
