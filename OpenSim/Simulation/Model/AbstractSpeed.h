#ifndef __AbstractSpeed_h__
#define __AbstractSpeed_h__

// AbstractSpeed.h
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
#include <iostream>
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>

namespace OpenSim {

class AbstractDynamicsEngine;
class AbstractCoordinate;

//=============================================================================
//=============================================================================
/**
 * An abstract class that specifies the interface for a generalied speed.
 * Speeds usually have a one-to-one correspondence with coordinates, but
 * they don't have to.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API AbstractSpeed : public Object
{

//=============================================================================
// DATA
//=============================================================================
protected:
	AbstractDynamicsEngine* _dynamicsEngine;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	AbstractSpeed();
	AbstractSpeed(const AbstractSpeed &aSpeed);
	virtual ~AbstractSpeed();
	virtual Object* copy() const = 0;

	AbstractSpeed& operator=(const AbstractSpeed &aSpeed);
   void copyData(const AbstractSpeed &aSpeed);
   virtual void setup(AbstractDynamicsEngine* aEngine);

	virtual AbstractCoordinate* getCoordinate() const = 0;
	virtual bool setCoordinate(AbstractCoordinate *aCoordinate) = 0;
	virtual bool setCoordinateName(const std::string& aCoordName) = 0;

	virtual double getDefaultValue() const = 0;
	virtual bool setDefaultValue(double aDefaultValue) = 0;
	virtual bool getDefaultValueUseDefault() const = 0;
	virtual bool getValueUseDefault() const = 0;

	virtual bool setValue(double aValue) = 0;
	virtual double getValue() const = 0;
	virtual double getAcceleration() const = 0;

	static std::string getSpeedName(const std::string &aCoordinateName);
	static std::string getCoordinateName(const std::string &aSpeedName);

private:
	void setNull();
//=============================================================================
};	// END of class AbstractSpeed
//=============================================================================
//=============================================================================

//typedef OSIMSIMULATION_API Set<AbstractSpeed> MarkerSet;

} // end of namespace OpenSim

#endif // __AbstractSpeed_h__


