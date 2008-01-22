#ifndef __SimmJoint_h__
#define __SimmJoint_h__

// SimmJoint.h
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
#include "osimSimmKinematicsEngineDLL.h"
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Transform.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/AbstractJoint.h>
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <OpenSim/Simulation/Model/DofSet.h>

namespace OpenSim {

class AbstractDynamicsEngine;
class SimmPath;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM joint.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMMKINEMATICSENGINE_API SimmJoint : public AbstractJoint  
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

	Array<SimmPath*> _pathList; // list of paths that use this joint

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
	virtual void invalidate();
	virtual bool isCoordinateUsed(AbstractCoordinate* aCoordinate) const;
	virtual bool hasXYZAxes() const;
	virtual void scale(const ScaleSet& aScaleSet);
	virtual void scale(const Array<double> &aScaleFactors);
   void addPathToList(SimmPath* aPath) { _pathList.append(aPath); }
	void clearPathList() { _pathList.setSize(0); }

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


