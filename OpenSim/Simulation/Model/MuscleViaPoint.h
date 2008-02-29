#ifndef __MuscleViaPoint_h__
#define __MuscleViaPoint_h__

// MuscleViaPoint.h
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
	virtual void init(const MusclePoint& aPoint);

	Array<double>& getRange() const { return _range; }
	const AbstractCoordinate* getCoordinate() const { return _coordinate; }
	void setCoordinate(AbstractCoordinate& aCoordinate);
	const std::string& getCoordinateName() const { return _coordinateName; }
	void setRangeMin(double aMin);
	void setRangeMax(double aMax);

	virtual bool isActive() const;
	virtual void setup(Model* aModel, AbstractMuscle* aMuscle);

	OPENSIM_DECLARE_DERIVED(MuscleViaPoint, MusclePoint);
private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class MuscleViaPoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MuscleViaPoint_h__


