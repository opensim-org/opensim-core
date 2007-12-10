#ifndef __SdfastSpeed_h__
#define __SdfastSpeed_h__

// SdfastSpeed.h
// Author: Peter Loan, Frank C. Anderson
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
#include <string>
#include "osimSdfastEngineDLL.h"
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Simulation/Model/AbstractSpeed.h>

namespace OpenSim {

class SdfastEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing an SD/FAST speed.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSDFASTENGINE_API SdfastSpeed : public AbstractSpeed  
{
//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyDbl _defaultValueProp;
	double &_defaultValue;

	/** Index of this speed in the list of Us (state vector index =
	_numQs + this index). */
	PropertyInt _indexProp;
	int &_index;

	/** Name of coordinate that this speed corresponds to (if any). */
	PropertyStr _coordinateNameProp;
	std::string &_coordinateName;

	/** Sdfast coordinate that this speed corresponds to (if any). */
	AbstractCoordinate *_coordinate;

	/** Sdfast engine that contains this speed. */
	SdfastEngine *_SdfastEngine;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SdfastSpeed();
	SdfastSpeed(const SdfastSpeed &aSpeed);
	SdfastSpeed(const AbstractSpeed &aSpeed);
	virtual ~SdfastSpeed();
	virtual Object* copy() const;

	SdfastSpeed& operator=(const SdfastSpeed &aSpeed);
	void copyData(const SdfastSpeed &aSpeed);
	void copyData(const AbstractSpeed &aSpeed);

	void setup(AbstractDynamicsEngine* aEngine);

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

	void setSdfastIndex(int aIndex) { _index = aIndex; }
	int getSdfastIndex() const { return _index; }

private:
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class SdfastSpeed
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SdfastSpeed_h__


