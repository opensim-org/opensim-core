#ifndef OPENSIM_HUNT_CROSSLEY_FORCE_H_
#define OPENSIM_HUNT_CROSSLEY_FORCE_H_
// HuntCrossleyForce.h
// Author: Peter Eastman
/*
 * Copyright (c)  2009-12 Stanford University. All rights reserved. 
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
#include "OpenSim/Common/Property.h"
#include "OpenSim/Common/Set.h"

#include "Force.h"

namespace OpenSim {

class Model;
//==============================================================================
//                         HUNT CROSSLEY FORCE
//==============================================================================
/** This force subclass implements a Hunt-Crossley contact model. It uses Hertz
contact theory to model the interactions between a set of ContactSpheres and 
ContactHalfSpaces.

@author Peter Eastman **/
class OSIMSIMULATION_API HuntCrossleyForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(HuntCrossleyForce, Force);
public:
    class ContactParameters;
    class ContactParametersSet;

//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. Others
    are inherited from the superclass. **/
    /**@{**/
	OpenSim_DECLARE_PROPERTY(contact_parameters, 
        HuntCrossleyForce::ContactParametersSet,
		"Material properties.");
	OpenSim_DECLARE_PROPERTY(transition_velocity, double,
		"Slip velocity (creep) at which peak static friction occurs.");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    HuntCrossleyForce();
    explicit HuntCrossleyForce(ContactParameters* params);

    ContactParametersSet& updContactParametersSet();
    const ContactParametersSet& getContactParametersSet();

    /** Takes over ownership of the passed-in object. **/
    void addContactParameters(ContactParameters* params);
    /**
     * Get the transition velocity for switching between static and dynamic friction.
     */
    double getTransitionVelocity() const;
    /**
     * Set the transition velocity for switching between static and dynamic friction.
     */
    void setTransitionVelocity(double velocity);

	//-----------------------------------------------------------------------------
	// Reporting
	//-----------------------------------------------------------------------------
	/** 
	 * Provide name(s) of the quantities (column labels) of the force value(s) to be reported
	 */
	virtual OpenSim::Array<std::string> getRecordLabels() const ;
	/**
	*  Provide the value(s) to be reported that correspond to the labels
	*/
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const ;

protected:

	/**
	 * Create a SimTK::Force which implements this Force.
	 */
	void createSystem(SimTK::MultibodySystem& system) const;


private:
    // INITIALIZATION
	void constructProperties();

//==============================================================================
};	// END of class HuntCrossleyForce
//==============================================================================
//==============================================================================


//==============================================================================
//                 HUNT CROSSLEY FORCE :: CONTACT PARAMETERS
//==============================================================================
class OSIMSIMULATION_API HuntCrossleyForce::ContactParameters : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(HuntCrossleyForce::ContactParameters, Object);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. Others
    are inherited from the superclass. **/
    /**@{**/
	OpenSim_DECLARE_LIST_PROPERTY(geometry, std::string,
		"Names of geometry objects affected by these parameters.");
	OpenSim_DECLARE_PROPERTY(stiffness, double,
		"");
	OpenSim_DECLARE_PROPERTY(dissipation, double,
		"");
	OpenSim_DECLARE_PROPERTY(static_friction, double,
		"");
	OpenSim_DECLARE_PROPERTY(dynamic_friction, double,
		"");
	OpenSim_DECLARE_PROPERTY(viscous_friction, double,
		"");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    ContactParameters();
    ContactParameters(double stiffness, double dissipation, 
                      double staticFriction, double dynamicFriction, 
                      double viscousFriction);

    const Property<std::string>& getGeometry() const;
    Property<std::string>& updGeometry();
    void addGeometry(const std::string& name);
    double getStiffness() const;
    void setStiffness(double stiffness);
    double getDissipation() const;
    void setDissipation(double dissipation);
    double getStaticFriction() const;
    void setStaticFriction(double friction);
    double getDynamicFriction() const;
    void setDynamicFriction(double friction);
    double getViscousFriction() const;
    void setViscousFriction(double friction);

private:
    void constructProperties();
};


//==============================================================================
//                 HUNT CROSSLEY FORCE :: CONTACT PARAMETERS SET
//==============================================================================
class OSIMSIMULATION_API HuntCrossleyForce::ContactParametersSet 
:   public Set<HuntCrossleyForce::ContactParameters> {
OpenSim_DECLARE_CONCRETE_OBJECT(HuntCrossleyForce::ContactParametersSet, 
                                Set<HuntCrossleyForce::ContactParameters>);

public:
	ContactParametersSet();

private:
	void setNull();
};

} // end of namespace OpenSim

#endif // OPENSIM_HUNT_CROSSLEY_FORCE_H_


