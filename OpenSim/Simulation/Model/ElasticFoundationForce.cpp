// ElasticFoundationForce.cpp
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

#include "ElasticFoundationForce.h"
#include "ContactGeometry.h"
#include "ContactGeometrySet.h"
#include "ContactMesh.h"
#include "Model.h"
#include <OpenSim/Simulation/Model/BodySet.h>

namespace OpenSim {

//==============================================================================
//                         ELASTIC FOUNDATION FORCE
//==============================================================================

// Default constructor.
ElasticFoundationForce::ElasticFoundationForce()
{
	constructProperties();
}

// Construct with supplied contact parameters.
ElasticFoundationForce::ElasticFoundationForce(ContactParameters* params)
{
	constructProperties();
    addContactParameters(params);
}

void ElasticFoundationForce::createSystem(SimTK::MultibodySystem& system) const
{
    Super::createSystem(system);

    const ContactParametersSet& contactParametersSet = 
        get_contact_parameters();
	const double& transitionVelocity = get_transition_velocity();

    SimTK::GeneralContactSubsystem& contacts = system.updContactSubsystem();
    SimTK::SimbodyMatterSubsystem& matter = system.updMatterSubsystem();
    SimTK::ContactSetIndex set = contacts.createContactSet();
    SimTK::ElasticFoundationForce force(_model->updForceSubsystem(), contacts, set);
    force.setTransitionVelocity(transitionVelocity);
    for (int i = 0; i < contactParametersSet.getSize(); ++i)
    {
        ContactParameters& params = contactParametersSet.get(i);
        for (int j = 0; j < params.getGeometry().size(); ++j)
        {
	        if (!_model->updContactGeometrySet().contains(params.getGeometry()[j]))
            {
                std::string errorMessage = "Invalid ContactGeometry (" + params.getGeometry()[j] + ") specified in ElasticFoundationForce" + getName();
		        throw (Exception(errorMessage.c_str()));
	        }
	        ContactGeometry& geom = _model->updContactGeometrySet().get(params.getGeometry()[j]);
            contacts.addBody(set, matter.updMobilizedBody(SimTK::MobilizedBodyIndex(geom.getBody().getIndex())), geom.createSimTKContactGeometry(), geom.getTransform());
            if (dynamic_cast<ContactMesh*>(&geom) != NULL)
                force.setBodyParameters(SimTK::ContactSurfaceIndex(contacts.getNumBodies(set)-1), 
                    params.getStiffness(), params.getDissipation(),
                    params.getStaticFriction(), params.getDynamicFriction(), params.getViscousFriction());
        }
    }

	// Beyond the const Component get the index so we can access the SimTK::Force later
	ElasticFoundationForce* mutableThis = const_cast<ElasticFoundationForce *>(this);
	mutableThis->_index = force.getForceIndex();
}

void ElasticFoundationForce::constructProperties()
{
	constructProperty_contact_parameters(ContactParametersSet());
	constructProperty_transition_velocity(0.01);
}


ElasticFoundationForce::ContactParametersSet& ElasticFoundationForce::updContactParametersSet()
{
    return upd_contact_parameters();
}

const ElasticFoundationForce::ContactParametersSet& ElasticFoundationForce::getContactParametersSet()
{
    return get_contact_parameters();
}

void ElasticFoundationForce::addContactParameters(ElasticFoundationForce::ContactParameters* params)
{
    updContactParametersSet().append(params);
}

double ElasticFoundationForce::getTransitionVelocity() const
{
    return get_transition_velocity();
}

void ElasticFoundationForce::setTransitionVelocity(double velocity)
{
    set_transition_velocity(velocity);
}



//==============================================================================
//               ELASTIC FOUNDATION FORCE :: CONTACT PARAMETERS
//==============================================================================

// Default constructor.
ElasticFoundationForce::ContactParameters::ContactParameters()
{
    constructProperties();
}

// Constructor specifying material properties.
ElasticFoundationForce::ContactParameters::ContactParameters
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


void ElasticFoundationForce::ContactParameters::constructProperties()
{
	constructProperty_geometry(); // a list of strings
	constructProperty_stiffness(0.0);
	constructProperty_dissipation(0.0);
	constructProperty_static_friction(0.0);
	constructProperty_dynamic_friction(0.0);
	constructProperty_viscous_friction(0.0);
}

const Property<std::string>& ElasticFoundationForce::ContactParameters::getGeometry() const
{
    return getProperty_geometry();
}

Property<std::string>& ElasticFoundationForce::ContactParameters::updGeometry()
{
    return updProperty_geometry();
}

void ElasticFoundationForce::ContactParameters::addGeometry(const std::string& name)
{
    updGeometry().appendValue(name);
}

double ElasticFoundationForce::ContactParameters::getStiffness() const
{
    return get_stiffness();
}

void ElasticFoundationForce::ContactParameters::setStiffness(double stiffness)
{
    set_stiffness(stiffness);
}

double ElasticFoundationForce::ContactParameters::getDissipation() const
{
    return get_dissipation();
}

void ElasticFoundationForce::ContactParameters::setDissipation(double dissipation)
{
    set_dissipation(dissipation);
}

double ElasticFoundationForce::ContactParameters::getStaticFriction() const
{
    return get_static_friction();
}

void ElasticFoundationForce::ContactParameters::setStaticFriction(double friction)
{
    set_static_friction(friction);
}

double ElasticFoundationForce::ContactParameters::getDynamicFriction() const
{
    return get_dynamic_friction();
}

void ElasticFoundationForce::ContactParameters::setDynamicFriction(double friction)
{
    set_dynamic_friction(friction);
}

double ElasticFoundationForce::ContactParameters::getViscousFriction() const
{
    return get_viscous_friction();
}

void ElasticFoundationForce::ContactParameters::setViscousFriction(double friction)
{
    set_viscous_friction(friction);
}

void ElasticFoundationForce::ContactParametersSet::setNull()
{
    // no data members
}

ElasticFoundationForce::ContactParametersSet::ContactParametersSet()
{
    setNull();
}


//=============================================================================
// Reporting
//=============================================================================
/** 
 * Provide names of the quantities (column labels) of the force value(s) reported
 * 
 */
OpenSim::Array<std::string> ElasticFoundationForce::getRecordLabels() const 
{
	OpenSim::Array<std::string> labels("");

	const ContactParametersSet& contactParametersSet = 
        get_contact_parameters();

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
OpenSim::Array<double> ElasticFoundationForce::getRecordValues(const SimTK::State& state) const 
{
	OpenSim::Array<double> values(1);

	const ContactParametersSet& contactParametersSet = 
        get_contact_parameters();

	const SimTK::ElasticFoundationForce& simtkForce = 
        (SimTK::ElasticFoundationForce &)(_model->getForceSubsystem().getForce(_index));

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


} // end of namespace OpenSim
