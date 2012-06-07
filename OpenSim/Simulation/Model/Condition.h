#ifndef __Condition_h__
#define __Condition_h__

// Condition.h
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
	void determineType();

//=============================================================================
};	// END of class Condition
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Condition_h__


