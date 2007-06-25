#ifndef __BodyScale_h__
#define __BodyScale_h__

// BodyScale.h
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/PropertyStrArray.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing how
 * to scale a body segment.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API BodyScale : public Object  
{

//=============================================================================
// DATA
//=============================================================================
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
	BodyScale(const BodyScale &aBodyScale);
	virtual ~BodyScale();
	virtual Object* copy() const;

	BodyScale& operator=(const BodyScale &aBodyScale);
	void copyData(const BodyScale &aBodyScale);

	const Array<std::string>& getAxisNames() const { return _axisNames; }

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class BodyScale
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __BodyScale_h__


