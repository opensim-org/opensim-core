#ifndef __Sine_h__
#define __Sine_h__

// Sine.h
// Author: Ajay Seth
/*
 * Copyright (c)  2011, Stanford University. All rights reserved. 
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
class OSIMCOMMON_API Sine : public Function
{
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
	Sine() : _amplitude(_amplitudeProp.getValueDbl()), _omega(_omegaProp.getValueDbl()), _phase(_omegaProp.getValueDbl()) { setupProperties();}
	// Convenience Constructor
	Sine(double amplitude, double omega, double phase) : _amplitude(amplitude), _omega(omega), _phase(phase) { setupProperties(); }
	// Copy Constructor
	Sine(const Sine &aFunc): _amplitude(_amplitudeProp.getValueDbl()), _omega(_omegaProp.getValueDbl()), _phase(_omegaProp.getValueDbl()) {
			setupProperties();
			_amplitude = aFunc._amplitude;  _omega = aFunc._omega;  _phase = aFunc._phase; 
	};
	virtual ~Sine() {};
	virtual Object* copy() const {
		Sine *func = new Sine(*this);
		return(func);
	};

private:
	void setupProperties() {
		setType("Sine");
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
	void setValue(double aValue);

	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------
    virtual double calcValue(const SimTK::Vector& x) const
	{
		return _amplitude*sin(_omega*x[0] + _phase);
	}
	
	double calcDerivative(const std::vector<int>& derivComponents, const SimTK::Vector& x) const
	{
		int n = derivComponents.size();

		return _amplitude*pow(_omega,n)*sin(_omega*x[0] + _phase + n*SimTK::Pi/2);
	}

	SimTK::Function* createSimTKFunction() const {
		return new FunctionAdapter(*this);
	}
   
	int getArgumentSize() const {return 1;}
	int getMaxDerivativeOrder() const {return 10;}

	OPENSIM_DECLARE_DERIVED(Sine, Function);

//=============================================================================
};	// END class Sine
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif  // __Sine_h__
