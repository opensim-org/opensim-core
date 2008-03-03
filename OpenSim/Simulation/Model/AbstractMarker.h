#ifndef __AbstractMarker_h__
#define __AbstractMarker_h__

// AbstractMarker.h
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
#include "SimTKcommon.h"

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
class OSIMSIMULATION_API AbstractMarker : public Object
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
	virtual void getOffset(SimTK::Vec3& rOffset) const = 0;
	virtual const SimTK::Vec3& getOffset() const = 0;
	virtual bool setOffset(const SimTK::Vec3& aOffset) = 0;
	//virtual bool setOffset(const double aPoint[3]) = 0;
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
	virtual void scale(const SimTK::Vec3& aScaleFactors) = 0;
	virtual void setup(AbstractDynamicsEngine *aEngine) = 0;
	virtual void removeSelfFromDisplay() = 0;
	virtual VisibleObject* getDisplayer() const { return NULL; }
	virtual void updateGeometry() = 0;

	OPENSIM_DECLARE_DERIVED(AbstractMarker, Object);

private:
	void setNull();
//=============================================================================
};	// END of class AbstractMarker
//=============================================================================
//=============================================================================

//typedef OSIMSIMULATION_API Set<AbstractMarker> MarkerSet;

} // end of namespace OpenSim

#endif // __AbstractMarker_h__


