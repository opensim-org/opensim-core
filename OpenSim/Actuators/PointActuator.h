#ifndef OPENSIM_POINT_ACTUATOR_H_
#define OPENSIM_POINT_ACTUATOR_H_
// PointActuator.h
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
 * Author: Ajay Seth
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

//=============================================================================
//                              POINT ACTUATOR
//=============================================================================
/**
 * A class that implements a point actuator acting on the model.
 * This actuator has no states; the control is simply the force to
 * be applied to the model.
 *
 * @author Ajay Seth
 */
class OSIMACTUATORS_API PointActuator : public Actuator {
OpenSim_DECLARE_CONCRETE_OBJECT(PointActuator, Actuator);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with this class. **/
    /**@{**/
	OpenSim_DECLARE_OPTIONAL_PROPERTY(body, std::string, 
        "Name of Body to which this actuator is applied.");
	OpenSim_DECLARE_PROPERTY(point, SimTK::Vec3,
		"Location of application point; in body frame unless "
        "point_is_global=true");
    /** The default is point_is_global=false. **/
	OpenSim_DECLARE_PROPERTY(point_is_global, bool,
		"Interpret point in Ground frame if true; otherwise, body frame.");
	OpenSim_DECLARE_PROPERTY(direction, SimTK::Vec3,
		"Force application direction; in body frame unless "
        "force_is_global=true.");
    /** The default is force_is_global=false. **/
	OpenSim_DECLARE_PROPERTY(force_is_global, bool,
		"Interpret direction in Ground frame if true; otherwise, body frame.");
	OpenSim_DECLARE_PROPERTY(optimal_force, double,
		"The maximum force produced by this actuator when fully activated.");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Default constructor or construct with body name given. An empty 
    name ("") is treated as though it were unspecified. **/
	PointActuator(const std::string& bodyName="");

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

	/** Set the 'optimal_force' property. **/
	void setOptimalForce(double aOptimalForce);
    /** Get the current value of the 'optimal_force' property. **/
	double getOptimalForce() const OVERRIDE_11; // Part of Actuator interface.

private:
	void setNull();
	void constructProperties();

	// Set the body to which this actuator applies; setting this pointer
    // also sets the corresponding body name property.
	void setBody(Body* aBody);
	Body* getBody() const;

	//--------------------------------------------------------------------------
	// Implement Force interface
	//--------------------------------------------------------------------------
	void computeForce(const SimTK::State& state, 
					  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
					  SimTK::Vector& mobilityForces) const OVERRIDE_11;

	//--------------------------------------------------------------------------
	// Implement Actuator interface (also see getOptimalForce() above)
	//--------------------------------------------------------------------------
	double computeActuation( const SimTK::State& s) const OVERRIDE_11;
	double getStress( const SimTK::State& s ) const OVERRIDE_11;

    //--------------------------------------------------------------------------
	// Implement ModelComponent interface
	//--------------------------------------------------------------------------
	// Setup method to initialize Body reference
	void connectToModel(Model& model) OVERRIDE_11;
    
    //--------------------------------------------------------------------------
	// Implement Object interface.
	//--------------------------------------------------------------------------
	void updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber=-1)
        OVERRIDE_11;

	// Corresponding Body to which the point actuator is applied.
    SimTK::ReferencePtr<Body> _body;

//=============================================================================
};	// END of class PointActuator

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_POINT_ACTUATOR_H_


