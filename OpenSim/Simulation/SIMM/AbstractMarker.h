#ifndef __AbstractMarker_h__
#define __AbstractMarker_h__

// AbstractMarker.h
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
#include <OpenSim/Tools/Object.h>

namespace OpenSim {

class VisibleObject;
class AbstractBody;
class AbstractDynamicsEngine;

//=============================================================================
//=============================================================================
/**
 * An abstract class that specifies the interface for a marker (a passive
 * motion capture marker that is fixed to a body segment).
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API AbstractMarker : public Object
{

//=============================================================================
// DATA
//=============================================================================

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	AbstractMarker();
	AbstractMarker(const AbstractMarker &aMarker);
	virtual ~AbstractMarker();
	virtual Object* copy() const = 0;
#ifndef SWIG
	AbstractMarker& operator=(const AbstractMarker &aMarker);
#endif
	virtual void updateFromMarker(const AbstractMarker &aMarker) = 0;
	virtual void getOffset(double *rOffset) const = 0;
	virtual const double* getOffset() const = 0;
	virtual bool setOffset(Array<double>& aOffset) = 0;
	virtual bool setOffset(const double aPoint[3]) = 0;
	virtual bool getOffsetUseDefault() const = 0;
	virtual bool getFixed() const = 0;
	virtual bool setFixed(bool aFixed) = 0;
	virtual bool getFixedUseDefault() const = 0;
	virtual const std::string* getBodyName() const = 0;
	virtual bool setBodyName(const std::string& aName) = 0;
	virtual bool getBodyNameUseDefault() const = 0;
	virtual bool setBodyNameUseDefault(bool aValue) = 0;
	virtual AbstractBody* getBody() const = 0;
	virtual void setBody(AbstractBody* aBody) = 0;
	virtual void scale(const Array<double>& aScaleFactors) = 0;
	virtual void setup(AbstractDynamicsEngine *aEngine) = 0;
	virtual void removeSelfFromDisplay() = 0;
	virtual VisibleObject* getDisplayer() { return NULL; }
	virtual void updateGeometry() = 0;
	virtual void peteTest() const { }

private:
	void setNull();
//=============================================================================
};	// END of class AbstractMarker
//=============================================================================
//=============================================================================

//typedef RDSIMULATION_API Set<AbstractMarker> MarkerSet;

} // end of namespace OpenSim

#endif // __AbstractMarker_h__


