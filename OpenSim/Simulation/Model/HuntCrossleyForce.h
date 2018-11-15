#ifndef OPENSIM_HUNT_CROSSLEY_FORCE_H_
#define OPENSIM_HUNT_CROSSLEY_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  HuntCrossleyForce.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Eastman                                                   *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
// INCLUDE
#include "Force.h"
#include "OpenSim/Common/Set.h"

namespace OpenSim {

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
    OpenSim_DECLARE_PROPERTY(contact_parameters, 
        HuntCrossleyForce::ContactParametersSet,
        "Material properties.");
    OpenSim_DECLARE_PROPERTY(transition_velocity, double,
        "Slip velocity (creep) at which peak static friction occurs.");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    HuntCrossleyForce();
#ifndef SWIG
    /** The force takes ownership of the passed-in params. */
    explicit HuntCrossleyForce(ContactParameters* params);
#endif
    ContactParametersSet& updContactParametersSet();
    const ContactParametersSet& getContactParametersSet();

    /** Takes over ownership of the passed-in object. **/
    void addContactParameters(ContactParameters* params);
    /**
     * Get the transition velocity for switching between static and dynamic friction.
     */
    double getTransitionVelocity() const;
    /**
     * %Set the transition velocity for switching between static and dynamic friction.
     */
    void setTransitionVelocity(double velocity);
    
    /**
     * Access to ContactParameters. Methods assume size 1 of ContactParametersSet and add one ContactParameter if needed
     */
    double getStiffness() ;
    void setStiffness(double stiffness) ;
    double getDissipation() ;
    void setDissipation(double dissipation);
    double getStaticFriction() ;
    void setStaticFriction(double friction);
    double getDynamicFriction() ;
    void setDynamicFriction(double friction);
    double getViscousFriction() ;
    void setViscousFriction(double friction);
    void addGeometry(const std::string& name);


    //-----------------------------------------------------------------------------
    // Reporting
    //-----------------------------------------------------------------------------
    /** 
     * Provide name(s) of the quantities (column labels) of the force value(s) to be reported
     */
    OpenSim::Array<std::string> getRecordLabels() const override ;
    /**
    *  Provide the value(s) to be reported that correspond to the labels
    */
    OpenSim::Array<double> getRecordValues(const SimTK::State& state) const override ;

protected:

    /**
     * Create a SimTK::Force which implements this Force.
     */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;


private:
    // INITIALIZATION
    void constructProperties();

//==============================================================================
};  // END of class HuntCrossleyForce
//==============================================================================
//==============================================================================
#ifndef SWIG
//==============================================================================
//                 HUNT CROSSLEY FORCE :: CONTACT PARAMETERS
//==============================================================================
class OSIMSIMULATION_API HuntCrossleyForce::ContactParameters : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(HuntCrossleyForce::ContactParameters, Object);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
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

#endif

} // end of namespace OpenSim

#endif // OPENSIM_HUNT_CROSSLEY_FORCE_H_


