#ifndef __FunctionThresholdCondition_h__
#define __FunctionThresholdCondition_h__
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  FunctionThresholdCondition.h                   *
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <SimTKsimbody.h>

#include "Condition.h"

namespace OpenSim {

class Model;
class Function;

//=============================================================================
//=============================================================================
/**
 * FunctionThresholdCondition is a concrete implementation of a Condition.
 * A FunctionThresholdCondition returns true if its associate function is above
 * a certain threshold and false otherwise.
 * 
 * Specific FunctionThresholdConditions should be derived from this class. 
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API FunctionThresholdCondition : public Condition {
OpenSim_DECLARE_CONCRETE_OBJECT(FunctionThresholdCondition, Condition);

//=============================================================================
// DATA
//=============================================================================

protected:
	/** Function that Condition evaluates */
	PropertyObjPtr<Function> _functionProp;
	Function *&_function;

	/** Function that Condition evaluates */
	PropertyDbl _thresholdProp;
	double &_threshold;

//=============================================================================
// METHODS
//=============================================================================
//--------------------------------------------------------------------------
// CONSTRUCTION
//--------------------------------------------------------------------------
public:
	FunctionThresholdCondition();
	FunctionThresholdCondition(const FunctionThresholdCondition &aFunctionThresholdCondition);
	virtual ~FunctionThresholdCondition();

	FunctionThresholdCondition& operator=(const FunctionThresholdCondition &aFunctionThresholdCondition);
	void copyData(const FunctionThresholdCondition &aFunctionThresholdCondition);

	// Implement Condition interface. 

	/**
	 *  The defining FunctionThresholdCondition method  
	 */
	bool calcCondition(const SimTK::State& s) const OVERRIDE_11;

private:
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class FunctionThresholdCondition
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __FunctionThresholdCondition_h__


