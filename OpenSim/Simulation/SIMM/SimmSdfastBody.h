#ifndef _SimmSdfastBody_h_
#define _SimmSdfastBody_h_

// SimmSdfastBody.h
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
#include <string>
#include <OpenSim/Simulation/rdSimulationDLL.h>

namespace OpenSim { 

class SimmBody;

//=============================================================================
//=============================================================================
/**
 * SimmSdfastBody holds the data corresponding to each SD/FAST body.
 * The number of these bodies does not necessarily match the number of
 * SimmBodys (if there are closed loops, then there will be more
 * SD/FAST bodies than SimmBodys).
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API SimmSdfastBody
{

//=============================================================================
// DATA
//=============================================================================
public:
	std::string _name;
	double _mass;
	double _massCenter[3];
	double _inertia[3][3];
	double _bodyToJoint[3];
	double _inboardToJoint[3];
	SimmBody* _SimmBody;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmSdfastBody(void);
	SimmSdfastBody(const SimmSdfastBody &aSdfastBody);
	~SimmSdfastBody(void);
	SimmSdfastBody* copy(void) const;

#ifndef SWIG
	SimmSdfastBody& operator=(const SimmSdfastBody &aSdfastBody);
#endif
   void SimmSdfastBody::copyData(const SimmSdfastBody &aSdfastBody);

//=============================================================================
};	// END of class SimmSdfastBody

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimmSdfastBody_h__


