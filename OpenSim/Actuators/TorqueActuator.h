#ifndef OPENSIM_TORQUE_ACTUATOR_H_
#define OPENSIM_TORQUE_ACTUATOR_H_
// TorqueActuator.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2009-12, Stanford University. All rights reserved. 
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

#include "Simbody.h"

namespace OpenSim { 

class Body;
class Model;

//==============================================================================
//                           TORQUE ACTUATOR
//==============================================================================
/**
 * A class that implements a torque actuator acting on a body.
 * This actuator has no states; the control is simply the force to
 * be applied to the model.
 *
 * @author Ajay Seth, Matt DeMers
 */
class OSIMACTUATORS_API TorqueActuator : public Actuator {
OpenSim_DECLARE_CONCRETE_OBJECT(TorqueActuator, Actuator);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_OPTIONAL_PROPERTY(bodyA, std::string,
		"Name of Body to which the torque actuator is applied.");
	OpenSim_DECLARE_OPTIONAL_PROPERTY(bodyB, std::string,
		"Name of Body to which the equal and opposite torque is applied.");
    /** The default is torque_is_global=true. **/
    OpenSim_DECLARE_PROPERTY(torque_is_global, bool, 
        "Interpret axis in Ground frame if true; otherwise, body A's frame.");
    /** The default direction for the axis is z (0,0,1). **/
	OpenSim_DECLARE_PROPERTY(axis, SimTK::Vec3,
        "Fixed direction about which torque is applied, in Ground or body A "
        "frame depending on 'torque_is_global' property.");
    /** The default for optimal force is 1. **/
	OpenSim_DECLARE_PROPERTY(optimal_force, double,
        "The maximum torque produced by this actuator when fully activated.");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
	/** Default constructor leaves body names unspecified. **/
    TorqueActuator();

    /** Construct with body names given. An empty name ("") is treated as
    though it were unspecified. **/
    TorqueActuator(const std::string& bodyNameA, 
                   const std::string& bodyNameB);

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.
	
    /** Set the 'axis' property to the supplied value; frame is interpreted
    according to the 'torque_is_global' property. **/
	void setAxis(const SimTK::Vec3& axis) 
    {   set_axis(axis); }
    /** Return the current value of the 'axis' property. **/
	const SimTK::Vec3& getAxis() const 
    {   return get_axis(); }

    /** Set the 'torque_is_global' property that determines how to interpret
    the 'axis' vector; if not global (Ground frame) it is in body A's frame. **/
	void setTorqueIsGlobal(bool isGlobal) 
    {   set_torque_is_global(isGlobal); }
    /** Return the current value of the 'torque_is_global' property. **/
	bool getTorqueIsGlobal() const
    {   return get_torque_is_global(); }

	/** Set the 'optimal_force' property. **/
	void setOptimalForce(double optimalForce)
    {   set_optimal_force(optimalForce); }
    /** Get the current value of the 'optimal_force' property. **/
	double getOptimalForce() const OVERRIDE_11 // Part of Actuator interface.
    {   return get_optimal_force(); }

//==============================================================================
// PRIVATE
//==============================================================================
private:
	void constructProperties();

	// Set the bodies to which this actuator applies; setting these pointers
    // also sets the corresponding body name properties.
	void setBodyA(Body* bodyp);
	void setBodyB(Body* bodyp);
    Body* getBodyA() const {return _bodyA;}
    Body* getBodyB() const {return _bodyB;}

	//--------------------------------------------------------------------------
	// Implement Force interface
	//--------------------------------------------------------------------------
	void computeForce(const SimTK::State& state, 
                      SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                      SimTK::Vector& mobilityForces) const OVERRIDE_11;

	//--------------------------------------------------------------------------
	// Implement Actuator interface (also see getOptimalForce() above)
	//--------------------------------------------------------------------------
	double computeActuation(const SimTK::State& s) const OVERRIDE_11;
	// Return the stress, defined as abs(force/optimal_force).
	double getStress(const SimTK::State& state) const OVERRIDE_11; 

	//--------------------------------------------------------------------------
	// Implement ModelComponent interface
	//--------------------------------------------------------------------------
	// Setup method initializes Body reference pointers to match the names.
	void setup(Model& model) OVERRIDE_11;

	//--------------------------------------------------------------------------
	// Implement Object interface.
	//--------------------------------------------------------------------------
	void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1)
        OVERRIDE_11;

//==============================================================================
// DATA
//==============================================================================
    // Note: reference pointers are automatically set to null on construction 
    // and also on copy construction and copy assignment.

	// Corresponding Body to which the torque actuator is applied.
    SimTK::ReferencePtr<Body> _bodyA;

	// Corresponding Body to which the equal and opposite torque is applied.
    SimTK::ReferencePtr<Body> _bodyB;

//==============================================================================
};	// END of class TorqueActuator

}; //namespace
//==============================================================================
//==============================================================================

#endif // OPENSIM_TORQUE_ACTUATOR_H_


