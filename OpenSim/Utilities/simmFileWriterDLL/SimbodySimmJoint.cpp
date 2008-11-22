// SimbodySimmJoint.cpp
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

#include <OpenSim/Common/SimmMacros.h>
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
	_txUsed = false;
	_tyUsed = false;
	_tzUsed = false;
   _rotationsUsed = 0;
   _parentBodyName = "";
   _childBodyName = "";
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
bool SimbodySimmJoint::addFunctionDof(const AbstractTransformAxis& aDof, const string& aCoordinateName,
                                      int aFunctionNumber)
{
   if (aDof.getMotionType() == AbstractTransformAxis::Translational) {
      const double* axis = aDof.getAxisPtr();
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
      _dof[component+3].setFunction(_translationNames[component], aDof.getMotionType(), aFunctionNumber,
                                    aCoordinateName, NULL);
      updateOrder(_translationNames[component]);
      if (component == 0) _txUsed = true;
      if (component == 1) _tyUsed = true;
      if (component == 2) _tzUsed = true;
   } else {
      if (_rotationsUsed == 3)
         return false;
      _dof[_rotationsUsed].setFunction(_rotationNames[_rotationsUsed], aDof.getMotionType(), aFunctionNumber,
                                       aCoordinateName, aDof.getAxisPtr());
      updateOrder(_rotationNames[_rotationsUsed]);
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
	// See if the name matches one of the translation names. This code
	// currently does not check to see if the translation component was already defined.
   for (int i=0; i<3; i++) {
      if (aName == _translationNames[i]) {
         _dof[i+3].setConstant(aName, AbstractTransformAxis::Translational, NULL, aValue);
         updateOrder(aName);
         if (i == 0) _txUsed = true;
         if (i == 1) _tyUsed = true;
         if (i == 2) _tzUsed = true;
         return true;
      }
   }

	// If you make it to here, then the DOF is a rotation. Ignore aName and
	// name the DOF according to its place in the transform order (e.g., the
	// first rotation is always named "r1").
	if (_rotationsUsed == 3 || aAxis == NULL)
		return false;
	_dof[_rotationsUsed].setConstant(_rotationNames[_rotationsUsed], AbstractTransformAxis::Rotational, aAxis, aValue);
	updateOrder(_rotationNames[_rotationsUsed]);
	_rotationsUsed++;

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
      if (_txUsed == true || _tyUsed == true || _tzUsed == true)
         return;
      _order.append(" t");
   } else {
      _order.append(" ");
      _order.append(aDofName);
   }
}

//_____________________________________________________________________________
/**
 * Finalize the joint-- make sure all 6 DOFs are initialized.
 */
void SimbodySimmJoint::finalize()
{
   // Initialize the rotations that are not used.
   for (int i=_rotationsUsed; i<3; i++) {
      _dof[i].setConstant(_rotationNames[i], AbstractTransformAxis::Rotational, defaultAxes[i], 0.0);
      updateOrder(_dof[i].getName());
   }

   // Initialize the translations that are not used.
   if (!_txUsed) {
      _dof[3].setConstant(_translationNames[0], AbstractTransformAxis::Translational, NULL, 0.0);
      updateOrder(_dof[3].getName());
      _txUsed = true; // not ideal, but needed for updateOrder
   }
   if (!_tyUsed) {
      _dof[4].setConstant(_translationNames[1], AbstractTransformAxis::Translational, NULL, 0.0);
      updateOrder(_dof[4].getName());
      _tyUsed = true; // not ideal, but needed for updateOrder
   }
   if (!_tzUsed) {
      _dof[5].setConstant(_translationNames[2], AbstractTransformAxis::Translational, NULL, 0.0);
      updateOrder(_dof[5].getName());
      _tzUsed = true; // not ideal, but needed for updateOrder
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
   for (int i=3; i<6; i++)
      _dof[i].write(aStream);
   for (int i=0; i<3; i++)
      _dof[i].write(aStream);
   aStream << "endjoint" << endl << endl;
}
