#ifndef __FunctionThresholdCondition_h__
#define __FunctionThresholdCondition_h__

// FunctionThresholdCondition.h
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
class OSIMSIMULATION_API FunctionThresholdCondition : public Condition  
{
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
	virtual Object* copy() const;

	FunctionThresholdCondition& operator=(const FunctionThresholdCondition &aFunctionThresholdCondition);
	void copyData(const FunctionThresholdCondition &aFunctionThresholdCondition);

	// Perform and setup that is necessary
	virtual void setup(Model& aModel);

	/**
	 *  The defining FunctionThresholdCondition method  
	 */
	virtual bool calcCondition(const SimTK::State& s) const;

private:
	void setNull();
	void setupProperties();
	void determineType();

//=============================================================================
};	// END of class FunctionThresholdCondition
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __FunctionThresholdCondition_h__


