#ifndef __SdfastSpeedSet_h__
#define __SdfastSpeedSet_h__

// SdfastSpeedSet.h
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

#include "osimSdfastEngineDLL.h"
#include <OpenSim/Common/Set.h>
#include "SdfastSpeed.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of speeds.
 *
 * @authors Peter Loan
 * @version 1.0
 */

class OSIMSDFASTENGINE_API SdfastSpeedSet :	public Set<SdfastSpeed>
{
private:
	void setNull();
public:
	SdfastSpeedSet();
	SdfastSpeedSet(const SdfastSpeedSet& aSdfastSpeedSet);
	~SdfastSpeedSet(void);
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	SdfastSpeedSet& operator=(const SdfastSpeedSet &aSdfastSpeedSet);
#endif
//=============================================================================
};	// END of class SdfastSpeedSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SdfastSpeedSet_h__
