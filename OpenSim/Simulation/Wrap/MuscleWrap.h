#ifndef __MuscleWrap_h__
#define __MuscleWrap_h__

// MuscleWrap.h
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyIntArray.h>
#include "MuscleWrapPoint.h"
#include "WrapResult.h"

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class AbstractWrapObject;
class AbstractDynamicsEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing an instance of muscle wrapping. That is, it is owned
 * by a particular muscle, and contains parameters for wrapping that muscle
 * over a particular wrap object.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API MuscleWrap : public Object
{

//=============================================================================
// DATA
//=============================================================================
public:

	enum WrapMethod
	{
		hybrid,
		midpoint,
		axial
	};

protected:
	PropertyStr _wrapObjectNameProp;
   std::string &_wrapObjectName;

	PropertyStr _methodNameProp;   // currently used only for ellipsoid wrapping
	std::string& _methodName;
	WrapMethod _method;

   PropertyIntArray _rangeProp;
   Array<int> &_range;

	AbstractWrapObject* _wrapObject;

	WrapResult _previousWrap;  // results from previous wrapping

   MuscleWrapPoint _wrapPoints[2]; // the two muscle points created when the muscle wraps

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	MuscleWrap();
	MuscleWrap(const MuscleWrap& aMuscleWrap);
	virtual ~MuscleWrap();
	virtual Object* copy() const;
#ifndef SWIG
	MuscleWrap& operator=(const MuscleWrap& aMuscleWrap);
#endif
   void copyData(const MuscleWrap& aMuscleWrap);

	virtual void setup(AbstractDynamicsEngine* aEngine);

	int getStartPoint() const { return _range[0]; }
	int getEndPoint() const { return _range[1]; }
	const std::string& getWrapObjectName() const { return _wrapObjectName; }
	AbstractWrapObject* getWrapObject() const { return _wrapObject; }
	MuscleWrapPoint& getWrapPoint(int aIndex);
	WrapMethod getMethod() const { return _method; }
	const std::string& getMethodName() const { return _methodName; }

	const WrapResult& getPreviousWrap() const { return _previousWrap; }
	void setPreviousWrap(const WrapResult& aWrapResult);
	void resetPreviousWrap();

	virtual void peteTest() const;

protected:
	void setupProperties();

private:
	void setNull();
//=============================================================================
};	// END of class MuscleWrap
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MuscleWrap_h__


