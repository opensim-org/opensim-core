#ifndef _SimmStep_h_
#define _SimmStep_h_

// SimmStep.h
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
#include <math.h>
#include <vector>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/Transform.h>
#include <OpenSim/Tools/Storage.h>

namespace OpenSim { 

class SimmJoint;

//=============================================================================
//=============================================================================
/**
 * A class implementing one step in a SimmPath; that is, a joint and direction
 * from a body to an adjacent body.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API SimmStep
{
public:
	typedef enum
	{
		forward,
		inverse
	} Direction;

//=============================================================================
// DATA
//=============================================================================
private:
	SimmJoint* _joint;
	Direction _direction;
	Transform  _transformCache;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmStep();
	SimmStep(SimmJoint* aJoint, Direction aDirection);
	virtual ~SimmStep();

	Direction getDirection() const { return _direction; }
	SimmJoint* getJoint() const { return _joint; }
	Transform& getJointTransform();
	void peteTest() const;

private:
	void calcTransforms();

//=============================================================================
};	// END of class SimmStep

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimmStep_h__


