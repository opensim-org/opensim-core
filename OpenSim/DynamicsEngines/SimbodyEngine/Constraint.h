#ifndef __Constraint_h__
#define __Constraint_h__

// Constraint.h
// Author: Frank C. Anderson, Ajay Seth
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


// INCLUDE
#include <iostream>
#include <string>
#include <math.h>
#include "osimSimbodyEngineDLL.h"
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Simulation/Model/AbstractConstraint.h>
#include <SimTKsimbody.h>

namespace OpenSim {

class SimbodyEngine;

//=============================================================================
//=============================================================================
/**
 * A parent class for implementing a Simbody Constraint.
 * Specific constraints should be derived from this class. 
 *
 * @author Frank C. Anderson
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMBODYENGINE_API Constraint : public AbstractConstraint  
{
//=============================================================================
// DATA
//=============================================================================

protected:
	/** Flag indicating whether the constraint is disabled or not.  Disabled 
	means that the constraint is not active in subsequent dynamics realizations. */
	PropertyBool _isDisabledProp;

	/** ID for the constraint in Simbody. */
	SimTK::ConstraintIndex _index;

	/** Simbody dynamics engine that contains this Constraint. */
	//SimbodyEngine* _engine;

//=============================================================================
// METHODS
//=============================================================================
//--------------------------------------------------------------------------
// CONSTRUCTION
//--------------------------------------------------------------------------
public:
	Constraint();
	Constraint(const Constraint &aConstraint);
	Constraint(const AbstractConstraint &aConstraint);
	virtual ~Constraint();
	virtual Object* copy() const;

	Constraint& operator=(const Constraint &aConstraint);
	void copyData(const Constraint &aConstraint);
	void copyData(const AbstractConstraint &aConstraint);

	virtual AbstractDynamicsEngine* getDynamicsEngine() const;
	SimbodyEngine* getEngine() const {
		return (SimbodyEngine*)(_dynamicsEngine);
	}

	virtual void setup(AbstractDynamicsEngine* aEngine);
	virtual void initializeState(SimTK::State& completeState);

	virtual void updateFromConstraint(const AbstractConstraint &aConstraint);
	virtual bool getIsDisabled() const;
	virtual bool setIsDisabled(bool isDisabled);
	virtual bool setIsDisabled(bool isDisabled, SimTK::State& theState);

	virtual void calcConstraintForces(SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInParent, 
									  SimTK::Vector& mobilityForces);

	virtual void scale(const ScaleSet& aScaleSet) {};

	OPENSIM_DECLARE_DERIVED(Constraint, AbstractConstraint);

private:
	void setNull();
	void setupProperties();
	void determineType();
	friend class SimbodyEngine;

//=============================================================================
};	// END of class Constraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Constraint_h__


