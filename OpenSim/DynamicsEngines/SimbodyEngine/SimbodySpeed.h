#ifndef __SimbodySpeed_h__
#define __SimbodySpeed_h__

// SimbodySpeed.h
// Author: Frank C. Anderson
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
#include "osimSimbodyEngineDLL.h"
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Simulation/Model/AbstractSpeed.h>

namespace OpenSim {

class SimbodyEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing an SD/FAST speed.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMBODYENGINE_API SimbodySpeed : public AbstractSpeed  
{
//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyDbl _defaultValueProp;
	double &_defaultValue;

	/** Index of this speed in the list of Us (state vector index =
	_numQs + this index). */
	PropertyInt _indexProp;
	int &_index;

	/** Name of coordinate that this speed corresponds to (if any). */
	PropertyStr _coordinateNameProp;
	std::string &_coordinateName;

	/** Simbody coordinate that this speed corresponds to (if any). */
	AbstractCoordinate *_coordinate;

	/** Simbody engine that contains this speed. */
	SimbodyEngine *_SimbodyEngine;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimbodySpeed();
	SimbodySpeed(const SimbodySpeed &aSpeed);
	SimbodySpeed(const AbstractSpeed &aSpeed);
	virtual ~SimbodySpeed();
	virtual Object* copy() const;

	SimbodySpeed& operator=(const SimbodySpeed &aSpeed);
	void copyData(const SimbodySpeed &aSpeed);
	void copyData(const AbstractSpeed &aSpeed);

	void setup(AbstractDynamicsEngine* aEngine);

	virtual AbstractCoordinate* getCoordinate() const { return _coordinate; }
	virtual bool setCoordinate(AbstractCoordinate *aCoordinate);
	virtual bool setCoordinateName(const std::string& aCoordName);

	virtual double getDefaultValue() const { return _defaultValue; }
	virtual bool setDefaultValue(double aDefaultValue);
	virtual bool getDefaultValueUseDefault() const { return _defaultValueProp.getUseDefault(); }
	virtual bool getValueUseDefault() const { return true; }

	virtual double getValue() const;
	virtual bool setValue(double aValue);
	virtual double getAcceleration() const;

	void setSimbodyIndex(int aIndex) { _index = aIndex; }
	int getSimbodyIndex() const { return _index; }

	virtual void peteTest() const;

private:
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class SimbodySpeed
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimbodySpeed_h__


