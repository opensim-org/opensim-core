#ifndef __UnilateralConstraint_h__
#define __UnilateralConstraint_h__

// UnilateralConstraint.h
// Author: Ajay Seth
/*
 * Copyright (c)  2009, Stanford University. All rights reserved. 
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
#include <OpenSim/Common/PropertyStrArray.h>
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
	UnilateralConstraint(const UnilateralConstraint &aUnilateralConstraint);
	virtual ~UnilateralConstraint();

	UnilateralConstraint& operator=(const UnilateralConstraint &aUnilateralConstraint);
	void copyData(const UnilateralConstraint &aUnilateralConstraint);

	virtual int getNumConstraintEquations() {return _numConstraintEquations;};

	// The unilateral conditions for this constraint.
	virtual std::vector<bool> unilateralConditionsSatisfied(const SimTK::State &s)
		{ return std::vector<bool>(_numConstraintEquations, false); };

protected:
	virtual void setup(Model& aModel);

private:
	void setNull();

//=============================================================================
};	// END of class Constraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __UnilateralConstraint_h__


