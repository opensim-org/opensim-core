#ifndef __MuscleWrap_h__
#define __MuscleWrap_h__

// MuscleWrap.h
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


// INCLUDE
#include <iostream>
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyIntArray.h>
#include "MuscleWrapPoint.h"
#include "WrapResult.h"

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class AbstractWrapObject;
class AbstractDynamicsEngine;
class AbstractMuscle;

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
class OSIMSIMULATION_API MuscleWrap : public Object
{

//=============================================================================
// DATA
//=============================================================================
public:

	enum WrapMethod
	{
		hybrid,
		midpoint,
		axial
	};

protected:
	PropertyStr _wrapObjectNameProp;
   std::string &_wrapObjectName;

	PropertyStr _methodNameProp;   // currently used only for ellipsoid wrapping
	std::string& _methodName;
	WrapMethod _method;

   PropertyIntArray _rangeProp;
   Array<int> &_range;

	AbstractWrapObject* _wrapObject;
	AbstractMuscle* _muscle;

	WrapResult _previousWrap;  // results from previous wrapping

   MuscleWrapPoint _wrapPoints[2]; // the two muscle points created when the muscle wraps

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	MuscleWrap();
	MuscleWrap(const MuscleWrap& aMuscleWrap);
	virtual ~MuscleWrap();
	virtual Object* copy() const;
#ifndef SWIG
	MuscleWrap& operator=(const MuscleWrap& aMuscleWrap);
#endif
   void copyData(const MuscleWrap& aMuscleWrap);

	virtual void setup(AbstractDynamicsEngine* aEngine, AbstractMuscle* aMuscle);

	int getStartPoint() const { return _range[0]; }
	void setStartPoint(int aIndex);
	int getEndPoint() const { return _range[1]; }
	void setEndPoint(int aIndex);
	const std::string& getWrapObjectName() const { return _wrapObjectName; }
	AbstractWrapObject* getWrapObject() const { return _wrapObject; }
	void setWrapObject(AbstractWrapObject& aWrapObject);
	MuscleWrapPoint& getWrapPoint(int aIndex);
	WrapMethod getMethod() const { return _method; }
	void setMethod(WrapMethod aMethod);
	const std::string& getMethodName() const { return _methodName; }
	AbstractMuscle* getMuscle() const { return _muscle; }

	const WrapResult& getPreviousWrap() const { return _previousWrap; }
	void setPreviousWrap(const WrapResult& aWrapResult);
	void resetPreviousWrap();

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


