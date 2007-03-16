#ifndef __SimmJoint_h__
#define __SimmJoint_h__

// SimmJoint.h
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
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyStrArray.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/Transform.h>
#include <OpenSim/Tools/ScaleSet.h>
#include "AbstractJoint.h"
#include "AbstractCoordinate.h"
#include "AbstractBody.h"
#include "DofSet.h"

namespace OpenSim {

class AbstractDynamicsEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM joint.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API SimmJoint : public AbstractJoint  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyStrArray _bodiesProp;
	Array<std::string>& _bodies;

	PropertyObj _dofSetProp;
	DofSet &_dofSet;

   AbstractBody *_childBody;
   AbstractBody *_parentBody;

	Transform _forwardTransform;
	Transform _inverseTransform;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmJoint();
	SimmJoint(const SimmJoint &aJoint);
	virtual ~SimmJoint();
	virtual Object* copy() const;

   virtual void setup(AbstractDynamicsEngine* aEngine);

#ifndef SWIG
   SimmJoint& operator=(const SimmJoint &aJoint);
#endif
   void copyData(const SimmJoint &aJoint);

	virtual DofSet* getDofSet() const { return &_dofSet; }
	virtual AbstractBody* getChildBody() const { return _childBody; }
	virtual AbstractBody* getParentBody() const { return _parentBody; }
	virtual const Transform& getForwardTransform();
	virtual const Transform& getInverseTransform();
	virtual bool isCoordinateUsed(AbstractCoordinate* aCoordinate) const;
	virtual bool hasXYZAxes() const;
	virtual void scale(const ScaleSet& aScaleSet);

	virtual void peteTest();

private:
	void setNull();
	void setupProperties();
	void calcTransforms();

//=============================================================================
};	// END of class SimmJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmJoint_h__


