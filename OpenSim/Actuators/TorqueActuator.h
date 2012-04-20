#ifndef _TorqueActuator_h_
#define _TorqueActuator_h_
// TorqueActuator.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2009, Stanford University. All rights reserved. 
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

/*
 * Author: Ajay Seth, Matt DeMers
 */


#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDblVec.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include "SimTKsimbody.h"


//=============================================================================
//=============================================================================
/**
 * A class that implements a torque actuator acting on a body.
 * This actuator has no states; the control is simply the force to
 * be applied to the model.
 *
 * @author Ajay Seth, Matt DeMers
 * @version 2.0
 */
namespace OpenSim { 

class Body;
class Model;

class OSIMACTUATORS_API TorqueActuator : public Actuator {
OpenSim_DECLARE_CONCRETE_OBJECT(TorqueActuator, Actuator);

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Corresponding Body to which the torque actuator is applied. */
    Body *_bodyA;

	/** Corresponding Body to which the equal and opposite torque is applied. */
    Body *_bodyB;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	TorqueActuator( std::string aBodyNameA="", std::string abodyNameB="");
	TorqueActuator( const TorqueActuator &aTorqueActuator);
	virtual ~TorqueActuator();

	void copyData(const TorqueActuator &aTorqueActuator);
private:
	void setNull();
	void setupProperties();
	

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	TorqueActuator& operator=(const TorqueActuator &aGenForce);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// GENERALIZED Body
	void setBodyA(Body* aBody);
	void setBodyB(Body* aBody);
	Body* getBodyA() const;
	Body* getBodyB() const;
	//Torque Axis
	void setAxis(SimTK::Vec3 anAxis) { setPropertyValue("axis", anAxis); } ;
	SimTK::Vec3 getAxis() const { return getPropertyValue<SimTK::Vec3>("axis"); };
	void setTorqueIsGlobal(bool aBool) {setPropertyValue("torque_is_global", aBool); };
	bool getTorqueIsGlobal() { return getPropertyValue<bool>("torque_is_global"); };

	// OPTIMAL FORCE
	void setOptimalForce(double aOptimalForce);
	double getOptimalForce() const;
	// STRESS
#ifndef SWIG
	double getStress( const SimTK::State& s ) const;

	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------
	virtual void computeForce( const SimTK::State& state, 
							   SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							   SimTK::Vector& mobilityForces) const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual double  computeActuation( const SimTK::State& s) const;

#endif
	//--------------------------------------------------------------------------
	// CHECK
	//--------------------------------------------------------------------------
	virtual bool check() const;

	// Setup method to initialize Body reference
	void setup(Model& aModel);

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);

//=============================================================================
};	// END of class TorqueActuator

}; //namespace
//=============================================================================
//=============================================================================

#endif // __TorqueActuator_h__


