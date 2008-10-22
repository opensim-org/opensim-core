#ifndef __Speed_h__
#define __Speed_h__

// Speed.h
// Author: Frank C. Anderson
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
#include "osimSimbodyEngineDLL.h"
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Simulation/Model/AbstractSpeed.h>
#include <SimTKsimbody.h>

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
class OSIMSIMBODYENGINE_API Speed : public AbstractSpeed  
{
//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyDbl _defaultValueProp;
	double &_defaultValue;

	/** Name of coordinate that this speed corresponds to (if any). */
	PropertyStr _coordinateNameProp;
	std::string &_coordinateName;

	/** Simbody coordinate that this speed corresponds to (if any). */
	AbstractCoordinate *_coordinate;

	/** ID of the body which this speed serves.  */
	SimTK::MobilizedBodyIndex _bodyIndex;

	/** Mobility index for this speed. */
	int _mobilityIndex;

	/** Simbody engine that contains this speed. */
	//SimbodyEngine *_engine;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Speed();
	Speed(const Speed &aSpeed);
	Speed(const AbstractSpeed &aSpeed);
	virtual ~Speed();
	virtual Object* copy() const;

	Speed& operator=(const Speed &aSpeed);
	void copyData(const Speed &aSpeed);
	void copyData(const AbstractSpeed &aSpeed);

	void setup(AbstractDynamicsEngine* aEngine);
	SimbodyEngine* getEngine() const {
		return (SimbodyEngine*)(_dynamicsEngine);
	}

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

private:
	void setNull();
	void setupProperties();
	friend class SimbodyEngine;
	friend class Joint;

//=============================================================================
};	// END of class Speed
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Speed_h__


