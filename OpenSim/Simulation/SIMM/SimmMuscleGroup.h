#ifndef __SimmMuscleGroup_h__
#define __SimmMuscleGroup_h__

// SimmMuscleGroup.h
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
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/XMLDocument.h>

namespace OpenSim {

class AbstractModel;
class AbstractActuator;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM muscle group. Muscle groups are used in SIMM
 * to make muscle menus in the GUI more user-friendly.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API SimmMuscleGroup : public Object  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	Array<AbstractActuator*> _muscles;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmMuscleGroup();
	SimmMuscleGroup(const SimmMuscleGroup &aGroup);
	SimmMuscleGroup(DOMElement *aElement);
	virtual ~SimmMuscleGroup();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;

#ifndef SWIG
	SimmMuscleGroup& operator=(const SimmMuscleGroup &aGroup);
#endif
   void copyData(const SimmMuscleGroup &aGroup);

   void setup(AbstractModel* aModel);

	bool contains(const std::string& aName) const;

	virtual void peteTest() const;

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmMuscleGroup
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmMuscleGroup_h__


