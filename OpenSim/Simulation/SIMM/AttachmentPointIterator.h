#ifndef __AttachmentPointIterator_h__
#define __AttachmentPointIterator_h__

// AttachmentPointIterator.h
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
#include <string>
#include <OpenSim/Simulation/rdSimulationDLL.h>

namespace OpenSim {

class SimmMusclePoint;

//=============================================================================
//=============================================================================
/**
 * A base class that specifies the interface for an [actuator] attachment point iterator.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API AttachmentPointIterator
{
//=============================================================================
// DATA
//=============================================================================
protected:
	int _counter;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	AttachmentPointIterator();
	virtual ~AttachmentPointIterator();

	virtual bool finished() const = 0;
	virtual SimmMusclePoint* getCurrent() = 0;
	virtual SimmMusclePoint* next() = 0;
	virtual void reset() = 0;
	virtual void end() = 0;

//=============================================================================
};	// END of class AttachmentPointIterator
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AttachmentPointIterator_h__


