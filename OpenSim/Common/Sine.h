#ifndef __Sine_h__
#define __Sine_h__
/* -------------------------------------------------------------------------- *
 *                              OpenSim:  Sine.h                              *
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


// INCLUDES
#include <string>
#include "Function.h"
#include "FunctionAdapter.h"
#include "PropertyDbl.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for representing a Sine function.
 *
 * This class inherits from Function and so can be used as input to
 * any class requiring a Fuction as input. Implements f(x) = A*sin(omega*x+phase)
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMCOMMON_API Sine : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(Sine, Function);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:

	PropertyDbl _amplitudeProp;
	double &_amplitude;

	PropertyDbl _omegaProp;
	double &_omega;

	PropertyDbl _phaseProp;
	double &_phase;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	Sine() : _amplitude(_amplitudeProp.getValueDbl()), _omega(_omegaProp.getValueDbl()), _phase(_phaseProp.getValueDbl()) { setupProperties();}
	// Convenience Constructor
	Sine(double amplitude, double omega, double phase) : _amplitude(_amplitudeProp.getValueDbl()), _omega(_omegaProp.getValueDbl()), _phase(_phaseProp.getValueDbl()) {
		setupProperties();
		_amplitude = amplitude;  _omega = omega;  _phase = phase; 
	}
	// Copy Constructor
	Sine(const Sine &aFunc): _amplitude(_amplitudeProp.getValueDbl()), _omega(_omegaProp.getValueDbl()), _phase(_phaseProp.getValueDbl()) {
			setupProperties();
			_amplitude = aFunc._amplitude;  _omega = aFunc._omega;  _phase = aFunc._phase; 
	};
	virtual ~Sine() {};

private:
	void setupProperties() {
		_amplitudeProp.setName("amplitude");
		_amplitudeProp.setComment("amplitude of the sinusoidal function");
		_amplitudeProp.setValue(1);
		_propertySet.append(&_amplitudeProp);

		_omegaProp.setName("omega");
		_omegaProp.setComment("the angular frequency (omega) in radians/sec");
		_omegaProp.setValue(1);
		_propertySet.append(&_omegaProp);

		_phaseProp.setName("phase");
		_phaseProp.setComment("the phase shift of the sinusoidal function");
		_phaseProp.setValue(0);
		_propertySet.append(&_phaseProp);
	}

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	Sine& operator=(const Sine &func)
	{
		Function::operator=(func);
		_amplitude = func._amplitude;  _omega = func._omega;  _phase = func._phase; 
		return(*this);
	}
	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
public:

	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------
    virtual double calcValue(const SimTK::Vector& x) const
	{
		return _amplitude*sin(_omega*x[0] + _phase);
	}
	
	double calcDerivative(const std::vector<int>& derivComponents, const SimTK::Vector& x) const
	{
		int n = (int)derivComponents.size();

		return _amplitude*pow(_omega,n)*sin(_omega*x[0] + _phase + n*SimTK::Pi/2);
	}

	SimTK::Function* createSimTKFunction() const {
		return new FunctionAdapter(*this);
	}
   
	int getArgumentSize() const {return 1;}
	int getMaxDerivativeOrder() const {return 10;}

//=============================================================================
};	// END class Sine
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif  // __Sine_h__
