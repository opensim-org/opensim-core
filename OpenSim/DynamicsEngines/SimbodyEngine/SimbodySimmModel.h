#ifndef __SimbodySimmModel_h__
#define __SimbodySimmModel_h__
// SimbodySimmModel.h
// Authors: Peter Loan
/*
 * Copyright (c)  2008, Stanford University. All rights reserved. 
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

// INCLUDES
#include <iostream>
#include <string>
#include "osimSimbodyEngineDLL.h"
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Function.h>
#include "SimbodySimmBody.h"
#include "SimbodySimmJoint.h"
#include "SimbodySimmGencoord.h"
#include "SimbodySimmFunction.h"
#include <SimTKsimbody.h>

#ifdef SWIG
	#ifdef OSIMSIMBODYENGINE_API
		#undef OSIMSIMBODYENGINE_API
		#define OSIMSIMBODYENGINE_API
	#endif
#endif

namespace OpenSim {

class SimbodyEngine;
class AbstractCoordinate;
class AbstractJoint;
class AbstractBody;
class Coordinate;
class Joint;
class MarkerSet;
class Function;

//=============================================================================
//=============================================================================
/**
 * A class to hold a SIMM model representing a Simbody model.
 *
 * @authors Peter Loan
 * @version 1.0
 */
class OSIMSIMBODYENGINE_API SimbodySimmModel : public Object
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Pointer to the SimbodyEngine that this object was created from. */
	const SimbodyEngine* _engine;

	/** bodies */
	Array<SimbodySimmBody*> _simmBody;

	/** joints */
	Array<SimbodySimmJoint*> _simmJoint;

	/** gencoords */
	Array<SimbodySimmGencoord*> _simmGencoord;

	/** functions */
	Array<SimbodySimmFunction*> _simmFunction;
   int _maxFunctionUserNumber;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~SimbodySimmModel();
	SimbodySimmModel();
	SimbodySimmModel(const SimbodyEngine& aEngine);
	SimbodySimmModel(const SimbodySimmModel& aModel);
	virtual Object* copy() const;
#ifndef SWIG
	SimbodySimmModel& operator=(const SimbodySimmModel &aModel);
#endif

private:
	void setNull();
	void copyData(const SimbodySimmModel &aEngine);
	void setup(const SimbodyEngine& aEngine);

public:
   bool writeJointFile(const std::string& aFileName) const;
   const std::string& getGravityLabel(const SimTK::Vec3& aGravity) const;
   Function* isDependent(const AbstractCoordinate* aCoordinate, const AbstractCoordinate** rIndependentCoordinate) const;
   void convertBody(const OpenSim::Body& aBody, const MarkerSet* aMarkerSet);
   bool isJointNeeded(SimTK::Vec3& aLocation, SimTK::Vec3& aOrientation);
   void makeSimmJoint(const std::string& aName, const std::string& aParentName, const std::string& aChildName,
                      SimTK::Vec3& aLocation, SimTK::Vec3& aOrientation);
   void addExtraJoints(const OpenSim::Joint& aJoint, std::string& rParentName, std::string& rChildName);
   void addBody(const OpenSim::Body& aBody);
   void addGencoord(const AbstractCoordinate* aCoordinate);
   int addFunction(const Function* aFunction, AbstractTransformAxis::MotionType aXType,
                   AbstractTransformAxis::MotionType aYType);
   void writeWrapObjects(AbstractBody& aBody, std::ofstream& aStream) const;

//=============================================================================
};	// END of class SimbodySimmModel
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimbodySimmModel_h__


