#ifndef _State_Function_h_
#define _State_Function_h_
// StateFunction.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2010, Stanford University. All rights reserved. 
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

/*  
 * Author: Jack Middleton
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
 * An abstract class for representing a function that takes the current State as an arguement.
 *
 *
 * @author Jack Middleton 
 */
namespace OpenSim { 

class OSIMCOMMON_API StateFunction : public Object
{
//=============================================================================
// DATA
//=============================================================================
protected:
    // The SimTK::StateFunction object implementing this class.

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	StateFunction(){};
	virtual ~StateFunction() { };
	virtual Object* copy() const = 0;

private:

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
public:
	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	//--------------------------------------------------------------------------
	// EVALUATE
	//--------------------------------------------------------------------------
    /**
     * Calculate the value of this function given the current state of the system.
     * 
     * @param s     reference to a SimTK State 
     */
    virtual double calcValue(const SimTK::State& s) const = 0;

	OPENSIM_DECLARE_DERIVED(StateFunction, Object);

//=============================================================================
};	// END class StateFunction

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __State_Function_h__
