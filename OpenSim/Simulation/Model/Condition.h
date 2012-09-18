#ifndef __Condition_h__
#define __Condition_h__
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Condition.h                            *
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
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <SimTKsimbody.h>

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * A parent class for implementing an OpenSim Condition. Condition objects
 * are useful for encapulating logic that is commonly used to make decisions.
 * A Condition returns whether or not a particular condition is true or not,
 * based on the current state. 
 * 
 * Specific Conditions should be derived from this class. 
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API Condition : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(Condition, Object);

//=============================================================================
// DATA
//=============================================================================

protected:
	/** Flag indicating whether the Condition is disabled or not.  Disabled 
	means that the Condition is not active when */
	PropertyBool _isDisabledProp;
	bool &_isDisabled;

    Model* _model;

//=============================================================================
// METHODS
//=============================================================================
//--------------------------------------------------------------------------
// CONSTRUCTION
//--------------------------------------------------------------------------
public:
	Condition();
	Condition(const Condition &aCondition);
	virtual ~Condition();

	Condition& operator=(const Condition &aCondition);
	void copyData(const Condition &aCondition);

	virtual void connectConditionToModel(Model& aModel);

	virtual bool isDisabled() const {return _isDisabled; } ;

	virtual void setDisabled(bool isDisabled) {_isDisabled = isDisabled; } ;

	/**
	 *  The defining condition method that subclasses must override 
	 */
	virtual bool calcCondition(const SimTK::State& s) const {return true; };

private:
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class Condition
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Condition_h__


