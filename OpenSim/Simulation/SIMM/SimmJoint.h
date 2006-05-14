#ifndef _SimmJoint_h_
#define _SimmJoint_h_

// SimmJoint.h
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
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyObjArray.h>
#include <OpenSim/Tools/PropertyStrArray.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/ArrayPtrs.h>
#include <OpenSim/Tools/Transform.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Tools/ScaleSet.h>
#include "dp.h"
#include "SimmCoordinate.h"
#include "SimmRotationDof.h"
#include "SimmTranslationDof.h"
#include "SimmBody.h"
#include "SimmStep.h"
#include "SimmSdfastBody.h"

namespace OpenSim { 

class SimmKinematicsEngine;
class SimmModel;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM joint.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API SimmJoint : public Object  
{

//=============================================================================
// DATA
//=============================================================================
public:
#ifndef SWIG
	typedef struct
	{
		bool used;
		std::string name;
		dpJointType type;
		int index;
		SimmStep::Direction direction;
		std::string inbname;
		std::string outbname;
		bool closesLoop;
	} sdfastJointInfo;

	sdfastJointInfo _sdfastInfo;
#endif
protected:
	PropertyStrArray _bodiesProp;
	Array<std::string>& _bodies;

	PropertyObjArray _dofsProp;
	ArrayPtrs<SimmDof>& _dofs;

   SimmBody *_childBody;
   SimmBody *_parentBody;

	Transform _forwaTransform;
	Transform _inverseTransform;

	bool _transformsValid;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmJoint();
	SimmJoint(DOMElement *aElement);
	SimmJoint(const SimmJoint &aJoint);
	virtual ~SimmJoint();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;

   void setup(SimmKinematicsEngine* aEngine);

#ifndef SWIG
	SimmJoint& operator=(const SimmJoint &aJoint);
#endif
   void copyData(const SimmJoint &aJoint);

	void invalidate() { _transformsValid = false; }
	Array<std::string>& getBodyNames() const { return _bodies; }
	ArrayPtrs<SimmDof>& getDofs() const { return _dofs; }
	SimmBody* getChildBody() const { return _childBody; }
	SimmBody* getParentBody() const { return _parentBody; }
	const Transform& getForwaTransform();
	const Transform& getInverseTransform();
	bool isCoordinateUsed(SimmCoordinate* aCoordinate) const;
	void identifyDpType(SimmModel* aModel);
	void makeSdfastJoint(std::ofstream& out, ArrayPtrs<SimmSdfastBody>& sdfastBodies, int* dofCount, int* constrainedCount, bool writeFile);
	void scale(const ScaleSet& aScaleSet);

	void writeSIMM(std::ofstream& out, int& aFunctionIndex) const;

	void peteTest();

private:
	void setNull();
	void setupProperties();
	void calcTransforms();
	bool isSdfastCompatible(void);
   bool hasXYZAxes(void);
	SimmRotationDof* findNthFunctionRotation(int n) const;
	SimmTranslationDof* findNthFunctionTranslation(int n) const;
	SimmTranslationDof* getTranslationDof(int axis) const;
	SimmTranslationDof* findMatchingTranslationDof(SimmRotationDof* rotDof);
	void makeSdfastWeld(std::ofstream& out, int* dofCount, int* constrainedCount, bool writeFile);
	void makeSdfastPin(std::ofstream& out, int* dofCount, int* constrainedCount, bool writeFile);
	void makeSdfastSlider(std::ofstream& out, int* dofCount, int* constrainedCount, bool writeFile);
	void makeSdfastPlanar(std::ofstream& out, int* dofCount, int* constrainedCount, bool writeFile);
	void makeSdfastUniversal(std::ofstream& out, int* dofCount, int* constrainedCount, bool writeFile);
	void makeSdfastCylindrical(std::ofstream& out, int* dofCount, int* constrainedCount, bool writeFile);
	void makeSdfastGimbal(std::ofstream& out, int* dofCount, int* constrainedCount, bool writeFile);
	void makeSdfastBushing(std::ofstream& out, int* dofCount, int* constrainedCount, bool writeFile);

//=============================================================================
};	// END of class SimmJoint
//=============================================================================
//=============================================================================

typedef OpenSim::Array<OpenSim::SimmJoint*> SimmJointList;

}; //namespace

#endif // __SimmJoint_h__


