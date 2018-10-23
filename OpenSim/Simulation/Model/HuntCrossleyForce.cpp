/* -------------------------------------------------------------------------- *
 *                      OpenSim:  HuntCrossleyForce.cpp                       *
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

#include "HuntCrossleyForce.h"
#include "ContactGeometry.h"
#include "Model.h"

#include "simbody/internal/HuntCrossleyForce.h"

namespace OpenSim {

//==============================================================================
//                          HUNT CROSSLEY FORCE
//==============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

// Default constructor.
HuntCrossleyForce::HuntCrossleyForce()
{
    constructProperties();
}

// Take over ownership of supplied object.
HuntCrossleyForce::HuntCrossleyForce(ContactParameters* params)
{
    constructProperties();
    addContactParameters(params);
}


void HuntCrossleyForce::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    const ContactParametersSet& contactParametersSet = 
        get_contact_parameters();
    const double& transitionVelocity = get_transition_velocity();

    SimTK::GeneralContactSubsystem& contacts = system.updContactSubsystem();
    SimTK::ContactSetIndex set = contacts.createContactSet();
    SimTK::HuntCrossleyForce force(_model->updForceSubsystem(), contacts, set);
    force.setTransitionVelocity(transitionVelocity);
    for (int i = 0; i < contactParametersSet.getSize(); ++i)
    {
        ContactParameters& params = contactParametersSet.get(i);
        for (int j = 0; j < params.getGeometry().size(); ++j) {
            // TODO: Dependency of HuntCrossleyForce on ContactGeometry
            // should be handled by Sockets.
            const ContactGeometry* contactGeom = nullptr;
            if (getModel().hasComponent<ContactGeometry>(params.getGeometry()[j]))
                contactGeom = &getModel().getComponent<ContactGeometry>(
                    params.getGeometry()[j]);
            else
                contactGeom = &getModel().getComponent<ContactGeometry>(
                    "./contactgeometryset/" + params.getGeometry()[j]);

            const ContactGeometry& geom = *contactGeom;
            // B: base Frame (Body or Ground)
            // F: PhysicalFrame that this ContactGeometry is connected to
            // P: the frame defined (relative to F) by the location and
            //    orientation properties.
            const auto& X_BF = geom.getFrame().findTransformInBaseFrame();
            const auto& X_FP = geom.getTransform();
            const auto X_BP = X_BF * X_FP;
            contacts.addBody(set, geom.getFrame().getMobilizedBody(),
                    geom.createSimTKContactGeometry(), X_BP);
            force.setBodyParameters(
                    SimTK::ContactSurfaceIndex(contacts.getNumBodies(set)-1),
                    params.getStiffness(), params.getDissipation(),
                    params.getStaticFriction(), params.getDynamicFriction(),
                    params.getViscousFriction());
        }
    }

    // Beyond the const Component get the index so we can access the
    // SimTK::Force later.
    HuntCrossleyForce* mutableThis = const_cast<HuntCrossleyForce *>(this);
    mutableThis->_index = force.getForceIndex();
}

void HuntCrossleyForce::constructProperties()
{
    constructProperty_contact_parameters(ContactParametersSet());
    constructProperty_transition_velocity(0.01);
}

HuntCrossleyForce::ContactParametersSet& HuntCrossleyForce::
updContactParametersSet()
{
    return upd_contact_parameters();
}

const HuntCrossleyForce::ContactParametersSet& HuntCrossleyForce::
getContactParametersSet()
{
    return get_contact_parameters();
}

void HuntCrossleyForce::
addContactParameters(HuntCrossleyForce::ContactParameters* params)
{
    updContactParametersSet().adoptAndAppend(params);
}

double HuntCrossleyForce::getTransitionVelocity() const
{
    return get_transition_velocity();
}

void HuntCrossleyForce::setTransitionVelocity(double velocity)
{
    set_transition_velocity(velocity);
}
/**
 * The following set of functions are introduced for convenience to get/set values in HuntCrossleyForce::ContactParameters
 * and for access in Matlab without exposing HuntCrossleyForce::ContactParameters. pending refactoring contact forces
 */
double HuntCrossleyForce::getStiffness()  { 
    if (get_contact_parameters().getSize()==0)
            updContactParametersSet().adoptAndAppend(
                    new HuntCrossleyForce::ContactParameters());
    return get_contact_parameters().get(0).getStiffness(); 
};
void HuntCrossleyForce::setStiffness(double stiffness) {
    if (get_contact_parameters().getSize()==0)
        updContactParametersSet().adoptAndAppend(
                new HuntCrossleyForce::ContactParameters());
    upd_contact_parameters()[0].setStiffness(stiffness); 
};
double HuntCrossleyForce::getDissipation()   { 
    if (get_contact_parameters().getSize()==0)
            updContactParametersSet().adoptAndAppend(
                    new HuntCrossleyForce::ContactParameters());
    return get_contact_parameters().get(0).getDissipation(); 
};
void HuntCrossleyForce::setDissipation(double dissipation) {
        if (get_contact_parameters().getSize()==0)
            updContactParametersSet().adoptAndAppend(
                    new HuntCrossleyForce::ContactParameters());
        upd_contact_parameters()[0].setDissipation(dissipation); 
};
double HuntCrossleyForce::getStaticFriction()  { 
    if (get_contact_parameters().getSize()==0)
            updContactParametersSet().adoptAndAppend(
                    new HuntCrossleyForce::ContactParameters());
    return get_contact_parameters().get(0).getStaticFriction(); 
};
void HuntCrossleyForce::setStaticFriction(double friction) {
    if (get_contact_parameters().getSize()==0)
        updContactParametersSet().adoptAndAppend(
                new HuntCrossleyForce::ContactParameters());
    upd_contact_parameters()[0].setStaticFriction(friction); 
};
double HuntCrossleyForce::getDynamicFriction()   { 
    if (get_contact_parameters().getSize()==0)
            updContactParametersSet().adoptAndAppend(
                    new HuntCrossleyForce::ContactParameters());
    return get_contact_parameters().get(0).getDynamicFriction(); 
};
void HuntCrossleyForce::setDynamicFriction(double friction) {
    if (get_contact_parameters().getSize()==0)
        updContactParametersSet().adoptAndAppend(
                new HuntCrossleyForce::ContactParameters());
    upd_contact_parameters()[0].setDynamicFriction(friction); 
};
double HuntCrossleyForce::getViscousFriction()   { 
    if (get_contact_parameters().getSize()==0)
        updContactParametersSet().adoptAndAppend(
                new HuntCrossleyForce::ContactParameters());
    return get_contact_parameters().get(0).getViscousFriction(); 
};
void HuntCrossleyForce::setViscousFriction(double friction) {
    if (get_contact_parameters().getSize()==0)
        updContactParametersSet().adoptAndAppend(
                new HuntCrossleyForce::ContactParameters());
    upd_contact_parameters()[0].setViscousFriction(friction); 
};

void HuntCrossleyForce::addGeometry(const std::string& name)
{
    if (get_contact_parameters().getSize()==0)
        updContactParametersSet().adoptAndAppend(
                new HuntCrossleyForce::ContactParameters());
    upd_contact_parameters()[0].addGeometry(name);
}
//==============================================================================
//                HUNT CROSSLEY FORCE :: CONTACT PARAMETERS
//==============================================================================

// Default constructor.
HuntCrossleyForce::ContactParameters::ContactParameters()
{
    constructProperties();
}

// Constructor specifying material properties.
HuntCrossleyForce::ContactParameters::ContactParameters
   (double stiffness, double dissipation, double staticFriction, 
    double dynamicFriction, double viscousFriction)
{
    constructProperties();
    set_stiffness(stiffness);
    set_dissipation(dissipation);
    set_static_friction(staticFriction);
    set_dynamic_friction(dynamicFriction);
    set_viscous_friction(viscousFriction);
}


void HuntCrossleyForce::ContactParameters::constructProperties()
{
    constructProperty_geometry(); // a list of strings
    constructProperty_stiffness(0.0);
    constructProperty_dissipation(0.0);
    constructProperty_static_friction(0.0);
    constructProperty_dynamic_friction(0.0);
    constructProperty_viscous_friction(0.0);
}

const Property<std::string>& HuntCrossleyForce::ContactParameters::getGeometry() const
{
    return getProperty_geometry();
}

Property<std::string>& HuntCrossleyForce::ContactParameters::updGeometry()
{
    return updProperty_geometry();
}

void HuntCrossleyForce::ContactParameters::addGeometry(const std::string& name)
{
    updGeometry().appendValue(name);
}

double HuntCrossleyForce::ContactParameters::getStiffness() const
{
    return get_stiffness();
}

void HuntCrossleyForce::ContactParameters::setStiffness(double stiffness)
{
    set_stiffness(stiffness);
}

double HuntCrossleyForce::ContactParameters::getDissipation() const
{
    return get_dissipation();
}

void HuntCrossleyForce::ContactParameters::setDissipation(double dissipation)
{
    set_dissipation(dissipation);
}

double HuntCrossleyForce::ContactParameters::getStaticFriction() const
{
    return get_static_friction();
}

void HuntCrossleyForce::ContactParameters::setStaticFriction(double friction)
{
    set_static_friction(friction);
}

double HuntCrossleyForce::ContactParameters::getDynamicFriction() const
{
    return get_dynamic_friction();
}

void HuntCrossleyForce::ContactParameters::setDynamicFriction(double friction)
{
    set_dynamic_friction(friction);
}

double HuntCrossleyForce::ContactParameters::getViscousFriction() const
{
    return get_viscous_friction();
}

void HuntCrossleyForce::ContactParameters::setViscousFriction(double friction)
{
    set_viscous_friction(friction);
}

void HuntCrossleyForce::ContactParametersSet::setNull()
{
    setAuthors("Peter Eastman");
}

HuntCrossleyForce::ContactParametersSet::ContactParametersSet()
{
    setNull();
}


//=============================================================================
// Reporting
//=============================================================================
/* Provide names of the quantities (column labels) of the force value(s) 
 * to be reported. */
OpenSim::Array<std::string> HuntCrossleyForce::getRecordLabels() const 
{
    OpenSim::Array<std::string> labels("");

    const ContactParametersSet& contactParametersSet = 
        get_contact_parameters();

    for (int i = 0; i < contactParametersSet.getSize(); ++i)
    {
        ContactParameters& params = contactParametersSet.get(i);
        for (int j = 0; j < params.getGeometry().size(); ++j)
        {
            // TODO: Dependency of HuntCrossleyForce on ContactGeometry
            // should be handled by Sockets.
            const ContactGeometry* contactGeom = nullptr;
            if (getModel().hasComponent<ContactGeometry>(params.getGeometry()[j]))
                contactGeom = &getModel().getComponent<ContactGeometry>(
                    params.getGeometry()[j]);
            else
                contactGeom = &getModel().getComponent<ContactGeometry>(
                    "./contactgeometryset/" + params.getGeometry()[j]);

            const ContactGeometry& geom = *contactGeom;
            std::string frameName = geom.getFrame().getName();
            labels.append(getName()+"."+frameName+".force.X");
            labels.append(getName()+"."+frameName+".force.Y");
            labels.append(getName()+"."+frameName+".force.Z");
            labels.append(getName()+"."+frameName+".torque.X");
            labels.append(getName()+"."+frameName+".torque.Y");
            labels.append(getName()+"."+frameName+".torque.Z");
        }
    }

    return labels;
}
/**
 * Provide the value(s) to be reported that correspond to the labels
 */
OpenSim::Array<double> HuntCrossleyForce::
getRecordValues(const SimTK::State& state) const 
{
    OpenSim::Array<double> values(1);

    const ContactParametersSet& contactParametersSet =
        get_contact_parameters();

    const auto& forceSubsys = _model->getForceSubsystem();
    const SimTK::Force& abstractForce = forceSubsys.getForce(_index);
    const auto& simtkForce = (SimTK::HuntCrossleyForce &)(abstractForce);

    SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
    SimTK::Vector_<SimTK::Vec3> particleForces(0);
    SimTK::Vector mobilityForces(0);

    //get the net force added to the system contributed by the Spring
    simtkForce.calcForceContribution(state, bodyForces, particleForces, 
                                     mobilityForces);

    for (int i = 0; i < contactParametersSet.getSize(); ++i)
    {
        ContactParameters& params = contactParametersSet.get(i);
        for (int j = 0; j < params.getGeometry().size(); ++j)
        {
            // TODO: Dependency of HuntCrossleyForce on ContactGeometry
            // should be handled by Sockets.
            const ContactGeometry* contactGeom = nullptr;
            if (getModel().hasComponent<ContactGeometry>(params.getGeometry()[j]))
                contactGeom = &getModel().getComponent<ContactGeometry>(
                    params.getGeometry()[j]);
            else
                contactGeom = &getModel().getComponent<ContactGeometry>(
                    "./contactgeometryset/" + params.getGeometry()[j]);

            const ContactGeometry& geom = *contactGeom;
    
            const auto& mbi = geom.getFrame().getMobilizedBodyIndex();
            const auto& thisBodyForce = bodyForces(mbi);
            SimTK::Vec3 forces = thisBodyForce[1];
            SimTK::Vec3 torques = thisBodyForce[0];

            values.append(3, &forces[0]);
            values.append(3, &torques[0]);
        }
    }

    return values;
}

}// end of namespace OpenSim
