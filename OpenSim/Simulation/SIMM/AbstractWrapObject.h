#ifndef __AbstractWrapObject_h__
#define __AbstractWrapObject_h__

// AbstractWrapObject.h
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
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/VisibleObject.h>
#include <OpenSim/Tools/PropertyBool.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/PropertyStr.h>

namespace OpenSim {

class VisibleObject;
class AbstractBody;
class AbstractDynamicsEngine;

//=============================================================================
//=============================================================================
/**
 * An abstract class that specifies the interface for a muscle wrapping
 * object.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API AbstractWrapObject : public Object
{

//=============================================================================
// DATA
//=============================================================================
public:
	enum WrapQuadrant
	{
		allQuadrants,
		negativeX,
		positiveX,
		negativeY,
		positiveY,
		negativeZ,
		positiveZ
	};

protected:

	PropertyDblArray _xyzBodyRotationProp;
	Array<double>& _xyzBodyRotation;

	PropertyDblArray _translationProp;
	Array<double>& _translation;

	PropertyBool _activeProp;
	bool& _active;

	PropertyStr _quadrantNameProp;
	std::string& _quadrantName;
	WrapQuadrant _quadrant;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	AbstractWrapObject();
	AbstractWrapObject(DOMElement* aElement);
	AbstractWrapObject(const AbstractWrapObject& aWrapObject);
	virtual ~AbstractWrapObject();
	virtual Object* copy() const = 0;
	virtual Object* copy(DOMElement* aElement) const = 0;
#ifndef SWIG
	AbstractWrapObject& operator=(const AbstractWrapObject& aWrapObject);
#endif
   void copyData(const AbstractWrapObject& aWrapObject);

	virtual void scale(Array<double>& aScaleFactors) = 0;
	virtual void setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody);

	const double* getXYZBodyRotation() const { return &_xyzBodyRotation[0]; }
	const double* getTranslation() const { return &_translation[0]; }
	bool getActive() const { return _active; }
	bool getActiveUseDefault() const { return _activeProp.getUseDefault(); }
	const std::string* getQuadrantName() const { return &_quadrantName; }
	bool getQuadrantNameUseDefault() const { return _quadrantNameProp.getUseDefault(); }

	virtual VisibleObject* getDisplayer() { return NULL; }
	virtual void peteTest() const;

protected:
	void setupProperties();

private:
	void setNull();
//=============================================================================
};	// END of class AbstractWrapObject
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractWrapObject_h__


