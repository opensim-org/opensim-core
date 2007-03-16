#ifndef __SdfastJoint_h__
#define __SdfastJoint_h__

// SdfastJoint.h
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
#include <string>
#include "SdfastEngineDLL.h"
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyStrArray.h>
#include <OpenSim/Tools/Transform.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Tools/ScaleSet.h>
#include <OpenSim/Simulation/SIMM/AbstractJoint.h>
#include <OpenSim/Simulation/SDFast/sdfast.h>
#include "SdfastBody.h"
#include "SdfastCoordinate.h"

namespace OpenSim {

class DofSet;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM joint.
 *
 * @author Peter Loan
 * @version 1.0
 */
class SDFAST_ENGINE_API SdfastJoint : public AbstractJoint  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Names of bodies that this joint connects. */
	PropertyStrArray _bodiesProp;
	Array<std::string>& _bodies;

	/** Location of the joint in the parent body specified in the parent
	reference frame. */
	PropertyDblArray _locationInParentProp;
	Array<double> &_locationInParent;

	/** Location of the joint in the child body specified in the child
	reference frame.  For SIMM models, this vector is always the zero vector
	(i.e., the body reference frame coincides with the joint).  */
	PropertyDblArray _locationInChildProp;
	Array<double> &_locationInChild;

	/** Index of this joint in the SD/FAST code. */
	PropertyInt _indexProp;
	int &_index;

	/** SD/FAST name of the joint type. */
	PropertyStr _SdfastTypeNameProp;
	std::string& _SdfastTypeName;

	/** Child body. */
   SdfastBody *_childBody;

	/** Parent body. */
   SdfastBody *_parentBody;

	/** Forward transform. */
	Transform _forwardTransform;

	/** Inverse transforms. */
	Transform _inverseTransform;

	/** Sdfast engine that contains this joint. */
	SdfastEngine* _SdfastEngine;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SdfastJoint();
	SdfastJoint(DOMElement *aElement);
	SdfastJoint(const SdfastJoint &aJoint);
	virtual ~SdfastJoint();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;

	void setup(AbstractDynamicsEngine* aEngine);

	SdfastJoint& operator=(const SdfastJoint &aJoint);
	void copyData(const SdfastJoint &aJoint);

	virtual DofSet* getDofSet() const { return NULL; }
	virtual SdfastBody* getChildBody() const { return _childBody; }
	virtual SdfastBody* getParentBody() const { return _parentBody; }
	virtual void setLocationInParent(const double aLocation[3]);
	virtual void getLocationInParent(double rLocation[3]) const;
	virtual void setLocationInChild(const double aLocation[3]);
	virtual void getLocationInChild(double rLocation[3]) const;
	virtual const Transform& getForwardTransform();
	virtual const Transform& getInverseTransform();
	virtual bool isCoordinateUsed(AbstractCoordinate* aCoordinate) const { return false; }
	virtual bool hasXYZAxes() const;
	virtual void scale(const ScaleSet& aScaleSet);

	void setSdfastIndex(int aIndex) { _index = aIndex; }
	int getSdfastIndex() const { return _index; }
	void setParentBodyName(const std::string& aName);
	void setChildBodyName(const std::string& aName);
	void setSdfastType(const char* aName);
	bool isTreeJoint() const;

	virtual void peteTest();

private:
	void setNull();
	void setupProperties();
	void calcTransforms();
	void updateSdfast();

//=============================================================================
};	// END of class SdfastJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SdfastJoint_h__


