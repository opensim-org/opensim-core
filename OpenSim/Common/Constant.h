#ifndef __Constant_h__
#define __Constant_h__

// Constant.h
// Author: Peter Loan, Ajay Seth
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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
#include "PropertyDbl.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for representing a constant value.
 *
 * This class inherits from Function and so can be used as input to
 * any class requiring a Fuction as input.
 *
 * @author Peter Loan, Ajay Seth
 * @version 1.0
 */
class OSIMCOMMON_API Constant : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(Constant, Function);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
	PropertyDbl _valueProp;
	double &_value;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	Constant();
	Constant(double value);
	Constant(const Constant &aSpline);
	virtual ~Constant();

private:
	void setNull();
	void setupProperties();
	void copyData(const Constant &aConstant);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	Constant& operator=(const Constant &aConstant);
#endif

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
public:
	void setValue(double aValue);

	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------
    virtual double calcValue(const SimTK::Vector& xUnused) const
	{
		return _value;
	}
	const double getValue() const { return _value; }
    SimTK::Function* createSimTKFunction() const;
//=============================================================================
};	// END class Constant
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif  // __Constant_h__
