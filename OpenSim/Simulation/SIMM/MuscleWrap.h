#ifndef __MuscleWrap_h__
#define __MuscleWrap_h__

// MuscleWrap.h
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
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyIntArray.h>
#include "SimmMuscleWrapPoint.h"

namespace OpenSim {

class AbstractWrapObject;
class AbstractDynamicsEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing an instance of muscle wrapping. That is, it is owned
 * by a particular muscle, and contains parameters for wrapping that muscle
 * over a particular wrap object.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API MuscleWrap : public Object
{

//=============================================================================
// DATA
//=============================================================================

	PropertyStr _wrapObjectNameProp;
   std::string &_wrapObjectName;

	PropertyStr _algorithmNameProp;   /* currently used only for ellipsoid wrapping */
   std::string &_algorithmName;

   PropertyIntArray _rangeProp;
   Array<int> &_range;

	AbstractWrapObject* _wrapObject;
   double _c[3];                       /* previous c point */
   double _r1[3];                      /* previous r1 (tangent) point */
   double _r2[3];                      /* previous r2 (tangent) point */
   SimmMuscleWrapPoint _wrapPoints[2]; /* the two muscle points created when the muscle wraps */

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	MuscleWrap();
	MuscleWrap(DOMElement* aElement);
	MuscleWrap(const MuscleWrap& aMuscleWrap);
	virtual ~MuscleWrap();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement* aElement) const;
#ifndef SWIG
	MuscleWrap& operator=(const MuscleWrap& aMuscleWrap);
#endif
   void copyData(const MuscleWrap& aMuscleWrap);

	virtual void setup(AbstractDynamicsEngine* aEngine);

	virtual void peteTest() const;

protected:
	void setupProperties();

private:
	void setNull();
//=============================================================================
};	// END of class MuscleWrap
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MuscleWrap_h__


