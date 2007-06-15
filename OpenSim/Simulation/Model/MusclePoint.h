#ifndef __MusclePoint_h__
#define __MusclePoint_h__

// MusclePoint.h
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
#include <OpenSim/Common/Geometry.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/Storage.h>

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class AbstractBody;
class Model;
class AbstractMuscle;
class AbstractDynamicsEngine;
class AbstractWrapObject;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM muscle point.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API MusclePoint : public Object  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
   PropertyDblArray _attachmentProp;
   Array<double> &_attachment;

	PropertyStr _bodyNameProp;
   std::string &_bodyName;

	// Support for Display
	PropertyObj _displayerProp;
	VisibleObject &_displayer;

	/* const*/ AbstractBody *_body; // Not const anymore since the body's displayer is not const

	AbstractMuscle* _muscle; // the muscle that owns this attachment point

	/** A temporary kluge until the default mechanism is working */
	static Geometry *_defaultGeometry;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	MusclePoint();
	MusclePoint(const MusclePoint &aPoint);
	virtual ~MusclePoint();
	virtual Object* copy() const;

#ifndef SWIG
	MusclePoint& operator=(const MusclePoint &aPoint);
#endif
   void copyData(const MusclePoint &aPoint);

	Array<double>& getAttachment() const { return _attachment; }
	void setAttachment(double aAttachment[3]);
   void setAttachment(int aCoordIndex, double aAttachment);
	const AbstractBody* getBody() const { return _body; }
	void setBody(AbstractBody& aBody);
	const std::string& getBodyName() const { return _bodyName; }
	void scale(Array<double>& aScaleFactors);
	const AbstractMuscle* getMuscle() const { return _muscle; }

	virtual bool isActive() const { return true; }
	virtual AbstractWrapObject* getWrapObject() const { return NULL; }
	virtual void setup(Model* aModel, AbstractMuscle* aMuscle);

	// Visible Object Support
	virtual VisibleObject* getDisplayer() const { return &_displayer; };
	virtual void updateGeometry();

	virtual void peteTest() const;

	OPENSIM_DECLARE_DERIVED(MusclePoint, Object);
protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class MusclePoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MusclePoint_h__


