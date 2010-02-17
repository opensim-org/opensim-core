// PassiveJointTorque.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS:  Frank C. Anderson, Kate Holzbaur
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University, All rights reserved. 
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/PropertyDbl.h>
#include "PassiveJointTorque.h"
#include <OpenSim/Simulation/Model/Muscle.h> 
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>

using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
PassiveJointTorque::~PassiveJointTorque()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PassiveJointTorque::
PassiveJointTorque(string aQName) :
	CoordinateActuator(aQName),
	_slope1(_propSlope1.getValueDbl()),
	_limit1(_propLimit1.getValueDbl()),
	_slope2(_propSlope2.getValueDbl()),
	_limit2(_propLimit2.getValueDbl()),
	_offset(_propOffset.getValueDbl()),
	_damping(_propDamping.getValueDbl())
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aActuator Actuator to be copied.
 */
PassiveJointTorque::
PassiveJointTorque(const PassiveJointTorque &aActuator) :
	CoordinateActuator(aActuator),
	_slope1(_propSlope1.getValueDbl()),
	_limit1(_propLimit1.getValueDbl()),
	_slope2(_propSlope2.getValueDbl()),
	_limit2(_propLimit2.getValueDbl()),
	_offset(_propOffset.getValueDbl()),
	_damping(_propDamping.getValueDbl())
{
	setNull();
	copyData(aActuator);
}
//_____________________________________________________________________________
/**
 * Copy this actuator and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this actuator.
 */
Object* PassiveJointTorque::
copy() const
{
	Actuator *act = new PassiveJointTorque(*this);
	return(act);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void PassiveJointTorque::
setNull()
{
	setType("PassiveJointTorque");
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Set up the serializable member variables.  This involves generating
 * properties and connecting local variables to those properties.
 */
void PassiveJointTorque::
setupProperties()
{
	_propSlope1.setComment("Slope of the passive restraining torque for the positive end of the ROM of the generalized coordinate (>0).");
	_propSlope1.setName("slope1");
	_propSlope1.setValue(20);
	_propertySet.append( &_propSlope1 );
	
	_propLimit1.setComment("The positive (or most positive) end of the ROM for the generalized coordinate in degrees.");
	_propLimit1.setName("limit1");
	_propLimit1.setValue(180.0);
	_propertySet.append( &_propLimit1 );
	
	
	_propSlope2.setComment("Slope of the passive restraining torque for the negative end of the ROM of the generalized coordinate (>0).");
	_propSlope2.setName("slope2");
	_propSlope2.setValue(20);
	_propertySet.append( &_propSlope2 );
	
	
	_propLimit2.setComment("The negative (or least positive) end of the ROM for the generalized coordinate in degrees.");
	_propLimit2.setName("limit2");
	_propLimit2.setValue(-180.0);
	_propertySet.append( &_propLimit2 );
	
	_propOffset.setComment("Parameter used to tune the location in the ROM where the 0 restraining moment occurs.");
	_propOffset.setName("offset");
	_propOffset.setValue(0.000);
	_propertySet.append( &_propOffset );
	
	_propDamping.setComment("Damping cooefficient for the generalized coordinate.");
	_propDamping.setName("damping");
	_propDamping.setValue(0.001);
	_propertySet.append( &_propDamping );
}

//_____________________________________________________________________________
/**
 * Copy the member data of the specified actuator.
 */
void PassiveJointTorque::
copyData(const PassiveJointTorque &aActuator)
{
	_slope1 = aActuator._slope1;
	_limit1 = aActuator._limit1;
	_slope2 = aActuator._slope2;
	_limit2 = aActuator._limit2;
	_offset = aActuator._offset;
	_damping = aActuator._damping;
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return  Reference to the altered object.
 */
PassiveJointTorque& PassiveJointTorque::
operator=(const PassiveJointTorque &aActuator)
{
	// BASE CLASS
	CoordinateActuator::operator =(aActuator);

	// DATA
	copyData(aActuator);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// Param1
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the parameters.
 *

 */
void PassiveJointTorque::
setSlope1(double aSlope1)
{
	_slope1 = aSlope1;

}
void PassiveJointTorque::
setLimit1(double aLimit1)
{
	_limit1 = aLimit1;

}
void PassiveJointTorque::
setSlope2(double aSlope2)
{
	_slope2 = aSlope2;

}
void PassiveJointTorque::
setLimit2(double aLimit2)
{
	_limit2 = aLimit2;

}
void PassiveJointTorque::
setOffset(double aOffset)
{
	_offset = aOffset;

}
void PassiveJointTorque::
setDamping(double aDamping)
{
	_damping = aDamping;

}

//_____________________________________________________________________________
/**
 * Get the parameters.
 *
 */
double PassiveJointTorque::
getSlope1() const
{
	return(_slope1);
}

double PassiveJointTorque::
getLimit1() const
{
	return(_limit1);
}
double PassiveJointTorque::
getSlope2() const
{
	return(_slope2);
}
double PassiveJointTorque::
getLimit2() const
{
	return(_limit2);
}
double PassiveJointTorque::
getOffset() const
{
	return(_offset);
}
double PassiveJointTorque::
getDamping() const
{
	return(_damping);
}
//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 */
double PassiveJointTorque::
computeActuation( const SimTK::State& s)
{
	if(_model==NULL) return 0;

	// SPEED
	if(_coord) {
		// FORCE
		double newq1 = _coord->getValue(s) - _limit1 * SimTK_DEGREE_TO_RADIAN;
		double newq2 = _coord->getValue(s) - _limit2 * SimTK_DEGREE_TO_RADIAN;
		double dampingForce = -_damping * _coord->getSpeedValue(s);
		double elasticForce = _offset - exp(_slope1 * newq1) + exp(-_slope2 * newq2);
		double totalForce = dampingForce + elasticForce;
		return( totalForce);
	} else 
        return 0;
}


