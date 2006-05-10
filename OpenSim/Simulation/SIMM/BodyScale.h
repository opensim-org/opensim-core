#ifndef _BodyScale_h_
#define _BodyScale_h_

// BodyScale.h
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


// INCLUDE
#include <iostream>
#include <math.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/Array.h>
#include <OpenSim/Tools/PropertyStrArray.h>
#include <OpenSim/Tools/XMLDocument.h>

//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing how
 * to scale a body segment.
 *
 * @author Peter Loan
 * @version 1.0
 */
namespace OpenSim { 

class RDSIMULATION_API BodyScale : public Object  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	PropertyStrArray _axisNamesProp;
	Array<std::string>& _axisNames;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	BodyScale();
	BodyScale(DOMElement *aElement);
	BodyScale(const BodyScale &aBodyScale);
	virtual ~BodyScale();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;

#ifndef SWIG
	BodyScale& operator=(const BodyScale &aBodyScale);
#endif
	void copyData(const BodyScale &aBodyScale);

	const Array<std::string>& getAxisNames() const { return _axisNames; }

	void peteTest() const;

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class BodyScale

}; //namespace
//=============================================================================
//=============================================================================

#endif // __BodyScale_h__


