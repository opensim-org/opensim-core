#ifndef __StepFunction_h__
#define __StepFunction_h__

// StepFunction.h
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


// INCLUDES
#include <string>
#include "Function.h"
#include "PropertyDbl.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for representing a StepFunction.
 *
 *	  		{	start_value,	t <= start_time 
 * f(t) =   {	S-plolynomial(t), start_time < t < end_time
 *			{   end_value,		t >= end_time
 *
 * This class inherits from Function and so can be used as input to
 * any class requiring a Fuction as input.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMCOMMON_API StepFunction : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(StepFunction, Function);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
	PropertyDbl _startTimeProp;
	double &_startTime;

	PropertyDbl _endTimeProp;
	double &_endTime;

	PropertyDbl _startValueProp;
	double &_startValue;

	PropertyDbl _endValueProp;
	double &_endValue;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	StepFunction();
	StepFunction(double startTime, double endTime, double startValue=0.0, double endValue=1.0);
	StepFunction(const StepFunction &aSpline);
	virtual ~StepFunction();

private:
	void setNull();
	void setupProperties();
	void copyData(const StepFunction &aStepFunction);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	StepFunction& operator=(const StepFunction &aStepFunction);
#endif

	//--------------------------------------------------------------------------
	// SET AND GET Coefficients
	//--------------------------------------------------------------------------
public:
	/** Set step transition start time */
	void setStartTime(double time)
		{ _startTime = time; };
	/** Get step transition time */
	double getStartTime() const
		{ return _startTime; };

	/** Set step transition end time */
	void setEndTime(double time)
		{ _endTime = time; };
	/** Get step transition time */
	double getEndTime() const
		{ return _endTime; };

	/** Set start value before step */
	void setStartValue(double start)
		{ _startValue = start; };
	/** Get start value before step */
	double getStartValue() const
		{ return _startValue; };

	/** Set end value before step */
	void setEndValue(double end)
		{ _endValue = end; };
	/** Get end value before step */
	double getEndValue() const
		{ return _endValue; };

	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------
    virtual SimTK::Function* createSimTKFunction() const;

//=============================================================================
};	// END class StepFunction
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif  // __StepFunction_h__
