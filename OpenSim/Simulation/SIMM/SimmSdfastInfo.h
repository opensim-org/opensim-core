#ifndef _SimmSdfastInfo_h_
#define _SimmSdfastInfo_h_

// SimmSdfastInfo.h
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
#include <OpenSim/Tools/Array.h>
#include <OpenSim/Tools/ArrayPtrs.h>
#include "SimmSdfastBody.h"
#include "SimmJoint.h"

//=============================================================================
//=============================================================================
/**
 * This class holds information that a SimmKinematicsEngine needs when
 * saving dynamics files.
 *
 * @author Peter Loan
 * @version 1.0
 */
namespace OpenSim { 

class RDSIMULATION_API SimmSdfastInfo
{

//=============================================================================
// DATA
//=============================================================================
public:
	/* _bodies[] holds the array of bodies that are written to the
	 * SD/FAST input file when saving dynamics. The size of this array
	 * is numJoints + 1, so it does not necessarily correspond exactly
	 * to the body array in the model (if there are closed loops, then
	 * there will be more SD/FAST bodies than Simm bodies).
	 * _jointOrder[] holds an array of pointers to the SimmJoints
	 * that are in the SD/FAST dynamics model, in the order that they are
	 * defined in the SD/FAST input file.
	 */
	ArrayPtrs<SimmSdfastBody> _bodies;
	Array<SimmJoint*> _jointOrder;
	int _numQs;
	int _numConstraints;
	int _numRestraintFunctions;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmSdfastInfo();
	virtual ~SimmSdfastInfo();

//=============================================================================
};	// END of class SimmSdfastInfo

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimmSdfastInfo_h__


