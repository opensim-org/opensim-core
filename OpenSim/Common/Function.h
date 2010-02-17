#ifndef _Function_h_
#define _Function_h_
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
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include "osimCommonDLL.h"
#include "Object.h"
#include "PropertyDbl.h"
#include "SimTKmath.h"


//=============================================================================
//=============================================================================
/**
 * An abstract class for representing a function.
 *
 * A function is a relation between independent variables and a dependent
 * value such that for any particular set of independent variables there is
 * only one unique dependent value.  Values of the function and its derivatives
 * are obtained by calling the evaluate() method.  The curve may or may not
 * be finite or diferentiable; the evaluate method returns values between
 * -SimTK::Infinity and SimTK::Infinity, or it returns SimTK::NaN
 * (not a number) if the curve is not defined.
 * Currently, functions of up to 3 variables (x,y,z) are supported.
 *
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMCOMMON_API Function : public Object
{
//=============================================================================
// DATA
//=============================================================================
protected:
    // The SimTK::Function object implementing this function.
    mutable SimTK::Function* _function;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	Function();
	Function(const Function &aFunction);
	virtual ~Function();
	virtual Object* copy() const = 0;
	virtual void init(Function* aFunction) { }

private:
	void setNull();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	Function& operator=(const Function &aFunction);
#endif
	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
public:
	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	static Function* makeFunctionOfType(Function* aFunction, const std::string& aNewTypeName);

	//--------------------------------------------------------------------------
	// EVALUATE
	//--------------------------------------------------------------------------
    /**
     * Calculate the value of this function at a particular point.
     * 
     * @param x     the Vector of input arguments.  Its size must equal the value returned by getArgumentSize().
     */
    virtual double calcValue(const SimTK::Vector& x) const;
    /**
     * Calculate a partial derivative of this function at a particular point.  Which derivative to take is specified
     * by listing the input components with which to take it.  For example, if derivComponents=={0}, that indicates
     * a first derivative with respective to component 0.  If derivComponents=={0, 0, 0}, that indicates a third
     * derivative with respective to component 0.  If derivComponents=={4, 7}, that indicates a partial second derivative with
     * respect to components 4 and 7.
     * 
     * @param derivComponents  the input components with respect to which the derivative should be taken.  Its size must be
     *                         less than or equal to the value returned by getMaxDerivativeOrder().
     * @param x                the Vector of input arguments.  Its size must equal the value returned by getArgumentSize().
     */
    virtual double calcDerivative(const std::vector<int>& derivComponents, const SimTK::Vector& x) const;
    /**
     * Get the number of components expected in the input vector.
     */
    virtual int getArgumentSize() const;
    /**
     * Get the maximum derivative order this Function object can calculate.
     */
    virtual int getMaxDerivativeOrder() const;
    virtual SimTK::Function* createSimTKFunction() const = 0;

	OPENSIM_DECLARE_DERIVED(Function, Object);
protected:
    /**
     * This should be called whenever this object has been modified.  It clears the internal SimTK::Function object
     * used to evaluate it.
     */
    void resetFunction();

//=============================================================================
};	// END class Function

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __Function_h__
