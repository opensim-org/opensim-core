#ifndef __SimmMusclePoint_h__
#define __SimmMusclePoint_h__

// SimmMusclePoint.h
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
#include <OpenSim/Tools/Array.h>
#include <OpenSim/Tools/Geometry.h>
#include <OpenSim/Tools/VisibleObject.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/Storage.h>

namespace OpenSim {

class AbstractBody;
class AbstractModel;
class AbstractSimmMuscle;
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
class RDSIMULATION_API SimmMusclePoint : public Object  
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

	/** A temporary kluge until the default mechanism is working */
	static Geometry *_defaultGeometry;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmMusclePoint();
	SimmMusclePoint(const SimmMusclePoint &aPoint);
	virtual ~SimmMusclePoint();
	virtual Object* copy() const;

#ifndef SWIG
	SimmMusclePoint& operator=(const SimmMusclePoint &aPoint);
#endif
   void copyData(const SimmMusclePoint &aPoint);

	Array<double>& getAttachment() const { return _attachment; }
	void setAttachment(double aAttachment[3]);
	const AbstractBody* getBody() const { return _body; }
	void setBody(AbstractBody& aBody);
	std::string& getBodyName() const { return _bodyName; }
	void scale(Array<double>& aScaleFactors);

	virtual bool isActive() const { return true; }
	virtual AbstractWrapObject* getWrapObject() const { return NULL; }
	virtual void setup(AbstractModel* aModel, AbstractSimmMuscle* aMuscle);

	// Visible Object Support
	virtual VisibleObject* getDisplayer() { return &_displayer; };
	virtual void updateGeometry();

	virtual void peteTest() const;

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmMusclePoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmMusclePoint_h__


