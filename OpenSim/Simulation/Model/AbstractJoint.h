#ifndef __AbstractJoint_h__
#define __AbstractJoint_h__

// AbstractJoint.h
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
#include <OpenSim/Common/Transform.h>
#include <OpenSim/Common/ScaleSet.h>

namespace OpenSim {

class DofSet;
class AbstractBody;
class AbstractCoordinate;
class AbstractDynamicsEngine;

//=============================================================================
//=============================================================================
/**
 * A base class that specifies the interface for a joint.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API AbstractJoint : public Object  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	AbstractDynamicsEngine* _dynamicsEngine;

	bool _transformsValid;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	AbstractJoint();
	AbstractJoint(const AbstractJoint &aJoint);
	virtual ~AbstractJoint();
	virtual Object* copy() const = 0;

#ifndef SWIG
	AbstractJoint& operator=(const AbstractJoint &aJoint);
#endif
   void copyData(const AbstractJoint &aJoint);

	virtual AbstractDynamicsEngine* getDynamicsEngine() { return _dynamicsEngine; }
   virtual void setup(AbstractDynamicsEngine* aEngine);

	virtual void invalidate() { _transformsValid = false; }

	virtual DofSet* getDofSet() const { return NULL; }
	virtual AbstractBody* getChildBody() const = 0;
	virtual AbstractBody* getParentBody() const = 0;
	virtual const Transform& getForwardTransform() = 0;
	virtual const Transform& getInverseTransform() = 0;
	virtual bool isCoordinateUsed(AbstractCoordinate* aCoordinate) const = 0;
	virtual bool hasXYZAxes() const = 0;
	virtual void scale(const ScaleSet& aScaleSet) = 0;

	virtual void peteTest() { }

private:
	void setNull();

//=============================================================================
};	// END of class AbstractJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractJoint_h__


