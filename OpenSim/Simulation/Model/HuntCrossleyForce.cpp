// HuntCrossleyForce.cpp
// Author: Peter Eastman
/*
 * Copyright (c) 2009 Stanford University. All rights reserved. 
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

#include "HuntCrossleyForce.h"
#include "ContactGeometry.h"
#include "ContactGeometrySet.h"
#include "Model.h"
#include <OpenSim/Simulation/Model/BodySet.h>

namespace OpenSim {

HuntCrossleyForce::HuntCrossleyForce() : Force()
{
	setupProperties();
}

HuntCrossleyForce::HuntCrossleyForce(const HuntCrossleyForce& copy) : Force(copy)
{
	setupProperties();
    copyData(copy);
}

HuntCrossleyForce::HuntCrossleyForce(ContactParameters* params) : Force()
{
	setupProperties();
    addContactParameters(params);
}


void HuntCrossleyForce::createSystem(SimTK::MultibodySystem& system) const
{
    const ContactParametersSet &contactParametersSet = getPropertyValue<ContactParametersSet>("contact_parameters");
	const double &transitionVelocity = getPropertyValue<double>("transition_velocity");

    SimTK::GeneralContactSubsystem& contacts = system.updContactSubsystem();
    SimTK::SimbodyMatterSubsystem& matter = system.updMatterSubsystem();
    SimTK::ContactSetIndex set = contacts.createContactSet();
    SimTK::HuntCrossleyForce force(_model->updForceSubsystem(), contacts, set);
    force.setTransitionVelocity(transitionVelocity);
    for (int i = 0; i < contactParametersSet.getSize(); ++i)
    {
        ContactParameters& params = contactParametersSet.get(i);
        for (int j = 0; j < params.getGeometry().size(); ++j)
        {
	        if (!_model->updContactGeometrySet().contains(params.getGeometry()[j]))
            {
                std::string errorMessage = "Invalid ContactGeometry (" + params.getGeometry()[j] + ") specified in HuntCrossleyForce" + getName();
		        throw (Exception(errorMessage.c_str()));
	        }
	        ContactGeometry& geom = _model->updContactGeometrySet().get(params.getGeometry()[j]);
            contacts.addBody(set, matter.updMobilizedBody(SimTK::MobilizedBodyIndex(geom.getBody().getIndex())), geom.createSimTKContactGeometry(), geom.getTransform());
            force.setBodyParameters(SimTK::ContactSurfaceIndex(contacts.getNumBodies(set)-1), params.getStiffness(), params.getDissipation(),
                params.getStaticFriction(), params.getDynamicFriction(), params.getViscousFriction());
        }
    }

	// Beyond the const Component get the index so we can access the SimTK::Force later
	HuntCrossleyForce* mutableThis = const_cast<HuntCrossleyForce *>(this);
	mutableThis->_index = force.getForceIndex();
}

void HuntCrossleyForce::setupProperties()
{
	addProperty<ContactParametersSet>("contact_parameters",
		"",
		ContactParametersSet());
	addProperty<double>("transition_velocity",
		"",
		0.01);
}

void HuntCrossleyForce::copyData(const HuntCrossleyForce& copy)
{
    setPropertyValue("contact_parameters", copy.getPropertyValue<ContactParametersSet>("contact_parameters"));
	setPropertyValue("transition_velocity", copy.getPropertyValue<double>("transition_velocity"));
}


HuntCrossleyForce::ContactParametersSet& HuntCrossleyForce::updContactParametersSet()
{
    return updPropertyValue<ContactParametersSet>("contact_parameters");
}

const HuntCrossleyForce::ContactParametersSet& HuntCrossleyForce::getContactParametersSet()
{
    return getPropertyValue<ContactParametersSet>("contact_parameters");
}

void HuntCrossleyForce::addContactParameters(HuntCrossleyForce::ContactParameters* params)
{
    updPropertyValue<ContactParametersSet>("contact_parameters").append(params);
}

double HuntCrossleyForce::getTransitionVelocity() const
{
    return getPropertyValue<double>("transition_velocity");
}

void HuntCrossleyForce::setTransitionVelocity(double velocity)
{
    setPropertyValue("transition_velocity", velocity);
}

HuntCrossleyForce::ContactParameters::ContactParameters()
{
    setupProperties();
}

HuntCrossleyForce::ContactParameters::ContactParameters(const ContactParameters& copy) :
	Object(copy)
{
    setupProperties();
    copyData(copy);
}

HuntCrossleyForce::ContactParameters::ContactParameters(double stiffness, double dissipation, double staticFriction, double dynamicFriction, double viscousFriction)
{
    setupProperties();
	setPropertyValue("stiffness", stiffness);
	setPropertyValue("dissipation", dissipation);
	setPropertyValue("static_friction", staticFriction);
	setPropertyValue("dynamic_friction", dynamicFriction);
	setPropertyValue("viscous_friction", viscousFriction);
}

void HuntCrossleyForce::ContactParameters::copyData(const ContactParameters& copy)
{
	setPropertyValue("geometry", copy.getProperty<std::string>("geometry"));

	setPropertyValue("stiffness", copy.getPropertyValue<double>("stiffness"));
	setPropertyValue("dissipation", copy.getPropertyValue<double>("dissipation"));
	setPropertyValue("static_friction", copy.getPropertyValue<double>("static_friction"));
	setPropertyValue("dynamic_friction", copy.getPropertyValue<double>("dynamic_friction"));
	setPropertyValue("viscous_friction", copy.getPropertyValue<double>("viscous_friction"));
}


void HuntCrossleyForce::ContactParameters::setupProperties()
{
	addListProperty<std::string>("geometry",
		"Names of geometry objects affected by these parameters.");

	addProperty<double>("stiffness",
		"",
		0.0);
	addProperty<double>("dissipation",
		"",
		0.0);
	addProperty<double>("static_friction",
		"",
		0.0);
	addProperty<double>("dynamic_friction",
		"",
		0.0);
	addProperty<double>("viscous_friction",
		"",
		0.0);
}

const Property<std::string>& HuntCrossleyForce::ContactParameters::getGeometry() const
{
    return getProperty<std::string>("geometry");
}

Property<std::string>& HuntCrossleyForce::ContactParameters::updGeometry()
{
    return updProperty<std::string>("geometry");
}

void HuntCrossleyForce::ContactParameters::addGeometry(const std::string& name)
{
    updGeometry().appendValue(name);
}

double HuntCrossleyForce::ContactParameters::getStiffness() const
{
    return getPropertyValue<double>("stiffness");
}

void HuntCrossleyForce::ContactParameters::setStiffness(double stiffness)
{
    setPropertyValue("stiffness", stiffness);
}

double HuntCrossleyForce::ContactParameters::getDissipation() const
{
    return getPropertyValue<double>("dissipation");
}

void HuntCrossleyForce::ContactParameters::setDissipation(double dissipation)
{
    setPropertyValue("dissipation", dissipation);
}

double HuntCrossleyForce::ContactParameters::getStaticFriction() const
{
    return getPropertyValue<double>("static_friction");
}

void HuntCrossleyForce::ContactParameters::setStaticFriction(double friction)
{
    setPropertyValue("static_friction", friction);
}

double HuntCrossleyForce::ContactParameters::getDynamicFriction() const
{
    return getPropertyValue<double>("dynamic_friction");
}

void HuntCrossleyForce::ContactParameters::setDynamicFriction(double friction)
{
    setPropertyValue("dynamic_friction", friction);
}

double HuntCrossleyForce::ContactParameters::getViscousFriction() const
{
    return getPropertyValue<double>("viscous_friction");
}

void HuntCrossleyForce::ContactParameters::setViscousFriction(double friction)
{
    setPropertyValue("viscous_friction", friction);
}

void HuntCrossleyForce::ContactParametersSet::setNull()
{
}

HuntCrossleyForce::ContactParametersSet::ContactParametersSet()
{
    setNull();
}

HuntCrossleyForce::ContactParametersSet::ContactParametersSet(const ContactParametersSet& copy) :
    Set<ContactParameters>(copy)
{
    setNull();
    *this = copy;
}

HuntCrossleyForce::ContactParametersSet::~ContactParametersSet(void)
{
}

HuntCrossleyForce::ContactParametersSet& HuntCrossleyForce::ContactParametersSet::operator=(const ContactParametersSet& copy)
{
	Set<ContactParameters>::operator=(copy);
	return (*this);
}

//=============================================================================
// Reporting
//=============================================================================
/** 
 * Provide names of the quantities (column labels) of the force value(s) reported
 * 
 */
OpenSim::Array<std::string> HuntCrossleyForce::getRecordLabels() const 
{
	OpenSim::Array<std::string> labels("");

	const ContactParametersSet &contactParametersSet = getPropertyValue<ContactParametersSet>("contact_parameters");

	for (int i = 0; i < contactParametersSet.getSize(); ++i)
    {
        ContactParameters& params = contactParametersSet.get(i);
        for (int j = 0; j < params.getGeometry().size(); ++j)
        {
			ContactGeometry& geom = _model->updContactGeometrySet().get(params.getGeometry()[j]);
			std::string bodyName = geom.getBodyName();
			labels.append(getName()+"."+bodyName+".force.X");
			labels.append(getName()+"."+bodyName+".force.Y");
			labels.append(getName()+"."+bodyName+".force.Z");
			labels.append(getName()+"."+bodyName+".torque.X");
			labels.append(getName()+"."+bodyName+".torque.Y");
			labels.append(getName()+"."+bodyName+".torque.Z");
		}
	}

	return labels;
}
/**
 * Provide the value(s) to be reported that correspond to the labels
 */
OpenSim::Array<double> HuntCrossleyForce::getRecordValues(const SimTK::State& state) const 
{
	OpenSim::Array<double> values(1);

	const ContactParametersSet &contactParametersSet = getPropertyValue<ContactParametersSet>("contact_parameters");

	const SimTK::HuntCrossleyForce &simtkForce = (SimTK::HuntCrossleyForce &)(_model->getForceSubsystem().getForce(_index));

	SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
	SimTK::Vector_<SimTK::Vec3> particleForces(0);
	SimTK::Vector mobilityForces(0);

	//get the net force added to the system contributed by the Spring
	simtkForce.calcForceContribution(state, bodyForces, particleForces, mobilityForces);

	for (int i = 0; i < contactParametersSet.getSize(); ++i)
    {
        ContactParameters& params = contactParametersSet.get(i);
        for (int j = 0; j < params.getGeometry().size(); ++j)
        {
			ContactGeometry& geom = _model->updContactGeometrySet().get(params.getGeometry()[j]);
			std::string bodyName = geom.getBodyName();
	
			SimTK::Vec3 forces = bodyForces(_model->getBodySet().get(bodyName).getIndex())[1];
			SimTK::Vec3 torques = bodyForces(_model->getBodySet().get(bodyName).getIndex())[0];

			values.append(3, &forces[0]);
			values.append(3, &torques[0]);
		}
	}

	return values;
}

}// end of namespace OpenSim
