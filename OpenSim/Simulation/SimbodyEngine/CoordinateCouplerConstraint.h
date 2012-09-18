#ifndef __CoordinateCouplerConstraint_h__
#define __CoordinateCouplerConstraint_h__
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  CoordinateCouplerConstraint.h                   *
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


