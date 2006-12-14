#ifndef __SimmMusclePointSet_h__
#define __SimmMusclePointSet_h__

// SimmMusclePointSet.h
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

#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/Set.h>
#include "SimmMusclePoint.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of muscle points.
 *
 * @authors Peter Loan
 * @version 1.0
 */

class RDSIMULATION_API SimmMusclePointSet :	public Set<SimmMusclePoint>
{
private:
	void setNull();
public:
	SimmMusclePointSet();
	SimmMusclePointSet(const SimmMusclePointSet& aSimmMusclePointSet);
	~SimmMusclePointSet(void);
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	SimmMusclePointSet& operator=(const SimmMusclePointSet &aSimmMusclePointSet);
#endif
//=============================================================================
};	// END of class SimmMusclePointSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmMusclePointSet_h__
