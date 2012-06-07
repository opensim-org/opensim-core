#ifndef __CoordinateCouplerConstraint_h__
#define __CoordinateCouplerConstraint_h__

// CoordinateCouplerConstraint.h
// Author: Ajay Seth
/*
 * Copyright (c) 2008, Stanford University. All rights reserved. 
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
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/PropertyDblVec.h>
#include <OpenSim/Common/Function.h>
#include "Constraint.h"
#include "Body.h"

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
 /** @file
 * A class implementing a CoordinateCoupler Constraint.  The underlying Constraint 
 * is a Constraint::CoordinateCoupler in Simbody, which relates coordinates
 * of the same or different body(ies) to one another at the position level (holonomic).
 * Relationship between coordinates is a specified by a function that equates 
 * to zero only when the coordinates satisfy the function (constraint).
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API CoordinateCouplerConstraint : public Constraint {
OpenSim_DECLARE_CONCRETE_OBJECT(CoordinateCouplerConstraint, Constraint);

//=============================================================================
// DATA
//=============================================================================
protected:

	/** Constraint function of generalized coordinates (to be specified) used to
	evaluate the constraint errors and their derivatives, and must valid to at
	least 2nd order. Constraint function must evaluate to zero when coordinates
	satisfy constraint */
	PropertyObjPtr<Function> _functionProp;
	Function *&_function;

	/** List of names of the independent coordinates (restricted to 1 for now). */
	PropertyStrArray _independentCoordNamesProp;
	Array<std::string>& _independentCoordNames;

	/** Name of the dependent coordinate. */
	PropertyStr _dependentCoordNameProp;
	std::string& _dependentCoordName;

    // Scale factor for the function.
    PropertyDbl _scaleFactorProp;
    double& _scaleFactor;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	CoordinateCouplerConstraint();
	CoordinateCouplerConstraint(const CoordinateCouplerConstraint &aConstraint);
	virtual ~CoordinateCouplerConstraint();

	CoordinateCouplerConstraint& operator=(const CoordinateCouplerConstraint &aConstraint);
	void copyData(const CoordinateCouplerConstraint &aConstraint);

	// GET AND SET
	void setIndependentCoordinateNames(const Array<std::string> &aCoordNames) { _independentCoordNames = aCoordNames; }
	const Array<std::string>& getIndependentCoordinateNames() const { return _independentCoordNames; }
	void setDependentCoordinateName(const std::string &aCoordName) { _dependentCoordName = aCoordName; }
	const std::string& getDependentCoordinateName() const { return _dependentCoordName; }
	Function& getFunction() const {return *_function; }
	void setFunction(const Function &aFunction) {_function = aFunction.clone();}
	void setFunction(Function *aFunction)  { _function = aFunction; }

	// SCALE
	virtual void scale(const ScaleSet& aScaleSet);

protected:
	void connectToModel(Model& aModel) OVERRIDE_11;
	/**
	 * Create a SimTK::Constraint::CoordinateCooupler which implements this constraint.
	 */
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

private:
	void setNull();
	void setupProperties();
	friend class SimbodyEngine;

//=============================================================================
};	// END of class CoordinateCouplerConstraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __CoordinateCouplerConstraint_h__


