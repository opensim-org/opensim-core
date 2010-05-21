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

ElasticFoundationForce::ElasticFoundationForce() :
	_contactParametersSetProp(PropertyObj("", ContactParametersSet())),
    _contactParametersSet((ContactParametersSet&)_contactParametersSetProp.getValueObj()),
    _transitionVelocityProp(PropertyDbl("", 0.01)),
    _transitionVelocity(_transitionVelocityProp.getValueDbl())
{
	setupProperties();
}

ElasticFoundationForce::ElasticFoundationForce(const ElasticFoundationForce& copy) : Force(copy),
	_contactParametersSetProp(PropertyObj("", ContactParametersSet())),
    _contactParametersSet((ContactParametersSet&)_contactParametersSetProp.getValueObj()),
    _transitionVelocityProp(PropertyDbl("", 0.01)),
    _transitionVelocity(_transitionVelocityProp.getValueDbl())
{
	setupProperties();
    copyData(copy);
}

ElasticFoundationForce::ElasticFoundationForce(ContactParameters* params) :
	_contactParametersSetProp(PropertyObj("", ContactParametersSet())),
    _contactParametersSet((ContactParametersSet&)_contactParametersSetProp.getValueObj()),
    _transitionVelocityProp(PropertyDbl("", 0.01)),
    _transitionVelocity(_transitionVelocityProp.getValueDbl())
{
	setupProperties();
    addContactParameters(params);
}

void ElasticFoundationForce::createSystem(SimTK::MultibodySystem& system) const
{
    SimTK::GeneralContactSubsystem& contacts = system.updContactSubsystem();
    SimTK::SimbodyMatterSubsystem& matter = system.updMatterSubsystem();
    SimTK::ContactSetIndex set = contacts.createContactSet();
    SimTK::ElasticFoundationForce force(_model->updUserForceSubsystem(), contacts, set);
    force.setTransitionVelocity(_transitionVelocity);
    for (int i = 0; i < _contactParametersSet.getSize(); ++i)
    {
        ContactParameters& params = _contactParametersSet.get(i);
        for (int j = 0; j < params.getGeometry().getSize(); ++j)
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

void ElasticFoundationForce::setupProperties()
{
    setType("ElasticFoundationForce");
	_contactParametersSetProp.setName("contact_parameters");
	_propertySet.append(&_contactParametersSetProp);
    _transitionVelocityProp.setName("transition_velocity");
	_propertySet.append(&_transitionVelocityProp);
}

void ElasticFoundationForce::copyData(const ElasticFoundationForce& copy)
{
    _contactParametersSet = copy._contactParametersSet;
	_transitionVelocity = copy._transitionVelocity;
}

//_____________________________________________________________________________
/**
 * Copy this force and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this Force.
 */
Object* ElasticFoundationForce::copy() const
{
	ElasticFoundationForce *force = new ElasticFoundationForce(*this);
	return(force);
}

ElasticFoundationForce::ContactParametersSet& ElasticFoundationForce::updContactParametersSet()
{
    return _contactParametersSet;
}

const ElasticFoundationForce::ContactParametersSet& ElasticFoundationForce::getContactParametersSet()
{
    return _contactParametersSet;
}

void ElasticFoundationForce::addContactParameters(ElasticFoundationForce::ContactParameters* params)
{
    _contactParametersSet.append(params);
}

double ElasticFoundationForce::getTransitionVelocity() const
{
    return _transitionVelocity;
}

void ElasticFoundationForce::setTransitionVelocity(double velocity)
{
    _transitionVelocity = velocity;
}

ElasticFoundationForce::ContactParameters::ContactParameters() :
    _geometry(_geometryProp.getValueStrArray()),
    _stiffness(_stiffnessProp.getValueDbl()),
    _dissipation(_dissipationProp.getValueDbl()),
    _staticFriction(_staticFrictionProp.getValueDbl()),
    _dynamicFriction(_dynamicFrictionProp.getValueDbl()),
    _viscousFriction(_viscousFrictionProp.getValueDbl())
{
    setupProperties();
}

ElasticFoundationForce::ContactParameters::ContactParameters(const ContactParameters& copy) :
	Object(copy),
    _geometry(_geometryProp.getValueStrArray()),
    _stiffness(_stiffnessProp.getValueDbl()),
    _dissipation(_dissipationProp.getValueDbl()),
    _staticFriction(_staticFrictionProp.getValueDbl()),
    _dynamicFriction(_dynamicFrictionProp.getValueDbl()),
    _viscousFriction(_viscousFrictionProp.getValueDbl())
{
    setupProperties();
    copyData(copy);
}

ElasticFoundationForce::ContactParameters::ContactParameters(double stiffness, double dissipation, double staticFriction, double dynamicFriction, double viscousFriction) :
    _geometry(_geometryProp.getValueStrArray()),
    _stiffness(_stiffnessProp.getValueDbl()),
    _dissipation(_dissipationProp.getValueDbl()),
    _staticFriction(_staticFrictionProp.getValueDbl()),
    _dynamicFriction(_dynamicFrictionProp.getValueDbl()),
    _viscousFriction(_viscousFrictionProp.getValueDbl())
{
    setupProperties();
    _stiffness = stiffness;
    _dissipation = dissipation;
    _staticFriction = staticFriction;
    _dynamicFriction = dynamicFriction;
    _viscousFriction = viscousFriction;
}

void ElasticFoundationForce::ContactParameters::copyData(const ContactParameters& copy)
{
    _geometry = copy._geometry;
    _stiffness = copy._stiffness;
    _dissipation = copy._dissipation;
    _staticFriction = copy._staticFriction;
    _dynamicFriction = copy._dynamicFriction;
    _viscousFriction = copy._viscousFriction;
}

Object* ElasticFoundationForce::ContactParameters::copy() const
{
	ContactParameters *cps = new ContactParameters(*this);
	return(cps);
}

void ElasticFoundationForce::ContactParameters::setupProperties()
{
    setType("ElasticFoundationForce::ContactParameters");
	_geometryProp.setName("geometry");
	_propertySet.append(&_geometryProp);
	_stiffnessProp.setName("stiffness");
	_propertySet.append(&_stiffnessProp);
	_dissipationProp.setName("dissipation");
	_propertySet.append(&_dissipationProp);
	_staticFrictionProp.setName("static_friction");
	_propertySet.append(&_staticFrictionProp);
	_dynamicFrictionProp.setName("dynamic_friction");
	_propertySet.append(&_dynamicFrictionProp);
	_viscousFrictionProp.setName("viscous_friction");
	_propertySet.append(&_viscousFrictionProp);
}

const Array<std::string>& ElasticFoundationForce::ContactParameters::getGeometry() const
{
    return _geometry;
}

Array<std::string>& ElasticFoundationForce::ContactParameters::updGeometry()
{
    return _geometry;
}

void ElasticFoundationForce::ContactParameters::addGeometry(const std::string& name)
{
    _geometry.append(name);
}

double ElasticFoundationForce::ContactParameters::getStiffness() const
{
    return _stiffness;
}

void ElasticFoundationForce::ContactParameters::setStiffness(double stiffness)
{
    _stiffness = stiffness;
}

double ElasticFoundationForce::ContactParameters::getDissipation() const
{
    return _dissipation;
}

void ElasticFoundationForce::ContactParameters::setDissipation(double dissipation)
{
    _dissipation = dissipation;
}

double ElasticFoundationForce::ContactParameters::getStaticFriction() const
{
    return _staticFriction;
}

void ElasticFoundationForce::ContactParameters::setStaticFriction(double friction)
{
    _staticFriction = friction;
}

double ElasticFoundationForce::ContactParameters::getDynamicFriction() const
{
    return _dynamicFriction;
}

void ElasticFoundationForce::ContactParameters::setDynamicFriction(double friction)
{
    _dynamicFriction = friction;
}

double ElasticFoundationForce::ContactParameters::getViscousFriction() const
{
    return _viscousFriction;
}

void ElasticFoundationForce::ContactParameters::setViscousFriction(double friction)
{
    _viscousFriction = friction;
}

void ElasticFoundationForce::ContactParametersSet::setNull()
{
	setType("ElasticFoundationForce::ContactParametersSet");
}

ElasticFoundationForce::ContactParametersSet::ContactParametersSet()
{
    setNull();
}

ElasticFoundationForce::ContactParametersSet::ContactParametersSet(const ContactParametersSet& copy) :
    Set<ContactParameters>(copy)
{
    setNull();
    *this = copy;
}

ElasticFoundationForce::ContactParametersSet::~ContactParametersSet(void)
{
}

ElasticFoundationForce::ContactParametersSet& ElasticFoundationForce::ContactParametersSet::operator=(const ContactParametersSet& copy)
{
	Set<ContactParameters>::operator=(copy);
	return (*this);
}

Object* ElasticFoundationForce::ContactParametersSet::copy() const
{
	ContactParametersSet *cpsSet = new ContactParametersSet(*this);
	return(cpsSet);
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


	for (int i = 0; i < _contactParametersSet.getSize(); ++i)
    {
        ContactParameters& params = _contactParametersSet.get(i);
        for (int j = 0; j < params.getGeometry().getSize(); ++j)
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

	const SimTK::ElasticFoundationForce &simtkForce = (SimTK::ElasticFoundationForce &)(_model->getUserForceSubsystem().getForce(_index));

	SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
	SimTK::Vector_<SimTK::Vec3> particleForces(0);
	SimTK::Vector mobilityForces(0);

	//get the net force added to the system contributed by the Spring
	simtkForce.calcForceContribution(state, bodyForces, particleForces, mobilityForces);

	for (int i = 0; i < _contactParametersSet.getSize(); ++i)
    {
        ContactParameters& params = _contactParametersSet.get(i);
        for (int j = 0; j < params.getGeometry().getSize(); ++j)
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
