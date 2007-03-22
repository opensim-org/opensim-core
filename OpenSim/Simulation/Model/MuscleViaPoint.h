#ifndef __MuscleViaPoint_h__
#define __MuscleViaPoint_h__

// MuscleViaPoint.h
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
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/MusclePoint.h>

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class AbstractCoordinate;
class Model;
class AbstractMuscle;
class AbstractDynamicsEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM muscle via point, which is a muscle point that
 * is active only for a specified range of a coordinate.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API MuscleViaPoint : public MusclePoint  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
   PropertyDblArray _rangeProp;
   Array<double> &_range;

	PropertyStr _coordinateNameProp;
   std::string &_coordinateName;

	const AbstractCoordinate* _coordinate;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	MuscleViaPoint();
	MuscleViaPoint(const MuscleViaPoint &aPoint);
	virtual ~MuscleViaPoint();
	virtual Object* copy() const;

#ifndef SWIG
	MuscleViaPoint& operator=(const MuscleViaPoint &aPoint);
#endif
   void copyData(const MuscleViaPoint &aPoint);

	static MuscleViaPoint* safeDownCast(Object* aObject) { return dynamic_cast<MuscleViaPoint*>(aObject); }

	Array<double>& getRange() const { return _range; }
	const AbstractCoordinate* getCoordinate() const { return _coordinate; }
	const std::string& getCoordinateName() const { return _coordinateName; }

	virtual bool isActive() const;
	virtual void setup(Model* aModel, AbstractMuscle* aMuscle);

	virtual void peteTest() const;

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class MuscleViaPoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MuscleViaPoint_h__


