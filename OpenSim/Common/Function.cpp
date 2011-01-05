// Function.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "Function.h"
#include "PropertyDbl.h"


using namespace OpenSim;
using namespace std;
using SimTK::Vector;


//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Function::~Function()
{
    if (_function != NULL)
        delete _function;
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Function::Function() :
	_function(NULL)
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aFunction Function to copy.
 */
Function::Function(const Function &aFunction) :
	Object(aFunction),
	_function(NULL)
{
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to NULL values.
 */
void Function::
setNull()
{
	setType("Function");
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
Function& Function::
operator=(const Function &aFunction)
{
	// BASE CLASS
	Object::operator=(aFunction);

	return(*this);
}


//=============================================================================
// UTILITIES
//=============================================================================
Function* Function::makeFunctionOfType(Function* aFunction, const string& aNewTypeName)
{
	Function* newFunction = NULL;

	if (aFunction != NULL) {
		Object* newObject = Object::newInstanceOfType(aNewTypeName);
		if (newObject) {
			newFunction = Function::safeDownCast(newObject);
			if (newFunction) {
				newFunction->init(aFunction);
				// newFunction's type will usually be written over by aFunction's type,
				// so set it back here.
				newFunction->setType(aNewTypeName);
			}
		}
	}

	return newFunction;
}

//=============================================================================
// EVALUATE
//=============================================================================
/*
double Function::evaluate(int aDerivOrder,double aX,double aY,double aZ) const
{
	SimTK::Vector workX(getArgumentSize(), aX);
	if (aDerivOrder == 0)
	    return calcValue(workX);
	std::vector<int> workDeriv(aDerivOrder);
	for (int i = 0; i < aDerivOrder; ++i)
	    workDeriv[i] = 0;
    return calcDerivative(workDeriv, workX);
}


/**
 * Evaluates total first derivative using the chain rule.
 *
double Function::
evaluateTotalFirstDerivative(double aX,double aDxdt)
{
	return evaluate(1,aX) * aDxdt;
}

/**
 * Evaluates total second derivative using the chain rule.
 *
double Function::
evaluateTotalSecondDerivative(double aX,double aDxdt,double aD2xdt2)
{
	return evaluate(1,aX) * aD2xdt2 + evaluate(2,aX) * aDxdt * aDxdt;
}
*/
double Function::calcValue(const Vector& x) const
{
    if (_function == NULL)
        _function = createSimTKFunction();
    return _function->calcValue(x);
}

double Function::calcDerivative(const std::vector<int>& derivComponents, const Vector& x) const
{
    if (_function == NULL)
        _function = createSimTKFunction();
    return _function->calcDerivative(derivComponents, x);
}

int Function::getArgumentSize() const
{
    if (_function == NULL)
        _function = createSimTKFunction();
    return _function->getArgumentSize();
}

int Function::getMaxDerivativeOrder() const
{
    if (_function == NULL)
        _function = createSimTKFunction();
    return _function->getMaxDerivativeOrder();
}

void Function::resetFunction()
{
    if (_function != NULL)
        delete _function;
    _function = NULL;
}
