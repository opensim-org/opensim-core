#ifndef __SdfastJoint_h__
#define __SdfastJoint_h__

// SdfastJoint.h
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
#include <string>
#include "osimSdfastEngineDLL.h"
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/Transform.h>
#include <OpenSim/Simulation/Model/AbstractJoint.h>
#include "SdfastBody.h"

namespace OpenSim {

class ScaleSet;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM joint.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSDFASTENGINE_API SdfastJoint : public AbstractJoint  
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
	SdfastJoint(const SdfastJoint &aJoint);
	virtual ~SdfastJoint();
	virtual Object* copy() const;

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


