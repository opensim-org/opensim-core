#ifndef __AbstractJoint_h__
#define __AbstractJoint_h__

// AbstractJoint.h
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

	OPENSIM_DECLARE_DERIVED(AbstractJoint, Object);

private:
	void setNull();

//=============================================================================
};	// END of class AbstractJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractJoint_h__


