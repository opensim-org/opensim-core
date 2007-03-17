#ifndef __SimmMeasurementSet_h__
#define __SimmMeasurementSet_h__

// SimmMeasurementSet.h
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
#include "SimmMeasurement.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of measurements.
 *
 * @authors Peter Loan
 * @version 1.0
 */

class RDSIMULATION_API SimmMeasurementSet :	public Set<SimmMeasurement>
{
private:
	void setNull();
public:
	SimmMeasurementSet();
	SimmMeasurementSet(const SimmMeasurementSet& aSimmMeasurementSet);
	~SimmMeasurementSet(void);
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	SimmMeasurementSet& operator=(const SimmMeasurementSet &aSimmMeasurementSet);
#endif
//=============================================================================
};	// END of class SimmMeasurementSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmMeasurementSet_h__
