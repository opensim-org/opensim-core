#ifndef __MovingMusclePoint_h__
#define __MovingMusclePoint_h__

// MovingMusclePoint.h
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
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Function.h>
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
 * A class implementing a moving muscle point, which is a muscle point that
 * moves in a body's reference frame as a function of a coordinate.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API MovingMusclePoint : public MusclePoint  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	PropertyObjPtr<Function> _xAttachmentProp;
	Function* &_xAttachment;

	PropertyStr _xCoordinateNameProp;
   std::string &_xCoordinateName;

	const AbstractCoordinate* _xCoordinate;

	PropertyObjPtr<Function> _yAttachmentProp;
	Function* &_yAttachment;

	PropertyStr _yCoordinateNameProp;
   std::string &_yCoordinateName;

	const AbstractCoordinate* _yCoordinate;

	PropertyObjPtr<Function> _zAttachmentProp;
	Function* &_zAttachment;

	PropertyStr _zCoordinateNameProp;
   std::string &_zCoordinateName;

	const AbstractCoordinate* _zCoordinate;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	MovingMusclePoint();
	MovingMusclePoint(const MovingMusclePoint &aPoint);
	virtual ~MovingMusclePoint();
	virtual Object* copy() const;

#ifndef SWIG
	MovingMusclePoint& operator=(const MovingMusclePoint &aPoint);
#endif
   void copyData(const MovingMusclePoint &aPoint);

	const AbstractCoordinate* getXCoordinate() const { return _xCoordinate; }
	const AbstractCoordinate* getYCoordinate() const { return _yCoordinate; }
	const AbstractCoordinate* getZCoordinate() const { return _zCoordinate; }
	void setXCoordinate(AbstractCoordinate& aCoordinate);
	void setYCoordinate(AbstractCoordinate& aCoordinate);
	void setZCoordinate(AbstractCoordinate& aCoordinate);
	const std::string& getXCoordinateName() const { return _xCoordinateName; }
	const std::string& getYCoordinateName() const { return _yCoordinateName; }
	const std::string& getZCoordinateName() const { return _zCoordinateName; }
	
	virtual	void scale(Array<double>& aScaleFactors);
	virtual bool isActive() const { return true; }
	virtual void setup(Model* aModel, AbstractMuscle* aMuscle);
	virtual void update();
	virtual void getVelocity(double aVelocity[3]);


	OPENSIM_DECLARE_DERIVED(MovingMusclePoint, MusclePoint);
private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class MovingMusclePoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MovingMusclePoint_h__


