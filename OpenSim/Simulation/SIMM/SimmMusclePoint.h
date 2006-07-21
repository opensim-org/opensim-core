#ifndef _SimmMusclePoint_h_
#define _SimmMusclePoint_h_

// SimmMusclePoint.h
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
#include <OpenSim/Tools/Array.h>
#include <OpenSim/Tools/Geometry.h>
#include <OpenSim/Tools/VisibleObject.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/XMLDocument.h>
#include "SimmBody.h"

namespace OpenSim { 

class SimmModel;
class SimmKinematicsEngine;

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
	PropertyObj		_displayerProp;
	VisibleObject	&_displayer;

	/* const*/ SimmBody *_body; // Not const anymore since the body'd displayer is not const

	static Geometry *_defaultGeometry;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmMusclePoint();
	SimmMusclePoint(DOMElement *aElement);
	SimmMusclePoint(const SimmMusclePoint &aPoint);
	virtual ~SimmMusclePoint();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;

#ifndef SWIG
	SimmMusclePoint& operator=(const SimmMusclePoint &aPoint);
#endif
   void SimmMusclePoint::copyData(const SimmMusclePoint &aPoint);

	Array<double>& getAttachment() const { return _attachment; }
	const SimmBody* getBody() const { return _body; }
	std::string& getBodyName() const { return _bodyName; }
	void scale(Array<double>& aScaleFactors);

	virtual void writeSIMM(std::ofstream& out) const;
	virtual void setup(SimmModel* model, SimmKinematicsEngine* ke);
	// Visible Object Support
	virtual VisibleObject* getDisplayer() { return &_displayer; };
	virtual void peteTest() const;

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmMusclePoint

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimmMusclePoint_h__


