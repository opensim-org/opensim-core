#ifndef __UnilateralConstraint_h__
#define __UnilateralConstraint_h__
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  UnilateralConstraint.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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


// INCLUDE
#include <iostream>
#include <string>
#include <math.h>
#include "Constraint.h"
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <SimTKsimbody.h>

namespace OpenSim {

class SimbodyEngine;

//=============================================================================
//=============================================================================
/**
 * An abstract class defining an OpenSim UnilateralConstraint.
 * Specific UnilateralConstraints should be derived from this class. 
 *
 * It is expeced that constraints used to model contact will be unilateral.
 * Furthermore, complex contact constraints can themselves employ several
 * SimTK::Constraints. In this case, disabling methods on Constraint should be
 * overriden and the appropriate logic applied to enabling/disabling individual
 * underlying constraints.  In most cases, the unilateral conditions should be
 * sufficient to determine the states of the internal (underlying) constraints
 * based on the global disabled condition.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API UnilateralConstraint : public Constraint {
OpenSim_DECLARE_ABSTRACT_OBJECT(UnilateralConstraint, Constraint);

//=============================================================================
// DATA
//=============================================================================

protected:
	/** number of constraint equations and thus unilateral conditions to be satisfied */
	int _numConstraintEquations;

//=============================================================================
// METHODS
//=============================================================================
//--------------------------------------------------------------------------
// CONSTRUCTION
//--------------------------------------------------------------------------
public:
	UnilateralConstraint();
	virtual ~UnilateralConstraint();

	virtual int getNumConstraintEquations() {return _numConstraintEquations;};

	// The unilateral conditions for this constraint.
	virtual std::vector<bool> unilateralConditionsSatisfied(const SimTK::State &s)
		{ return std::vector<bool>(_numConstraintEquations, false); };

protected:
	void connectToModel(Model& aModel) OVERRIDE_11;

private:
	void setNull();

//=============================================================================
};	// END of class Constraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __UnilateralConstraint_h__


