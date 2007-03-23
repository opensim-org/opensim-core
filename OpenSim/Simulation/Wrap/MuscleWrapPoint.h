#ifndef __MuscleWrapPoint_h__
#define __MuscleWrapPoint_h__

// MuscleWrapPoint.h
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
#include <math.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/SimmPoint.h>
#include <OpenSim/Simulation/Model/MusclePoint.h>

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class AbstractCoordinate;
class Model;
class AbstractMuscle;
class AbstractDynamicsEngine;
class AbstractWrapObject;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM muscle via point, which is a muscle point that
 * is active only for a specified range of a coordinate.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API MuscleWrapPoint : public MusclePoint  
{

//=============================================================================
// DATA
//=============================================================================
private:
	Array<SimmPoint> _wrapPath; // points defining muscle path on surface of wrap object
   double _wrapPathLength; // length of _wrapPath

	AbstractWrapObject* _wrapObject; // the wrap object this point is on

protected:

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	MuscleWrapPoint();
	MuscleWrapPoint(const MuscleWrapPoint &aPoint);
	virtual ~MuscleWrapPoint();
	virtual Object* copy() const;

#ifndef SWIG
	MuscleWrapPoint& operator=(const MuscleWrapPoint &aPoint);
#endif
   void copyData(const MuscleWrapPoint &aPoint);
	virtual void setup(Model* aModel, AbstractMuscle* aMuscle);

	Array<SimmPoint>& getWrapPath() { return _wrapPath; }
	double getWrapLength() const { return _wrapPathLength; }
	void setWrapLength(double aLength) { _wrapPathLength = aLength; }
	virtual AbstractWrapObject* getWrapObject() const { return _wrapObject; }
	void setWrapObject(AbstractWrapObject* aWrapObject) { _wrapObject = aWrapObject; }

	virtual void peteTest() const;

	OPENSIM_DECLARE_DERIVED(MuscleWrapPoint, MusclePoint);

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class MuscleWrapPoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MuscleWrapPoint_h__


