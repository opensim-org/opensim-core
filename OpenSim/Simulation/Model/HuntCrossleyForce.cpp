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

namespace OpenSim {

HuntCrossleyForce::HuntCrossleyForce() : Force(),
	_contactParametersSetProp(PropertyObj("", ContactParametersSet())),
    _contactParametersSet((ContactParametersSet&)_contactParametersSetProp.getValueObj()),
    _transitionVelocityProp(PropertyDbl("", 0.01)),
    _transitionVelocity(_transitionVelocityProp.getValueDbl())
{
	setupProperties();
}

HuntCrossleyForce::HuntCrossleyForce(const HuntCrossleyForce& copy) : Force(copy),
	_contactParametersSetProp(PropertyObj("", ContactParametersSet())),
    _contactParametersSet((ContactParametersSet&)_contactParametersSetProp.getValueObj()),
    _transitionVelocityProp(PropertyDbl("", 0.01)),
    _transitionVelocity(_transitionVelocityProp.getValueDbl())
{
	setupProperties();
    copyData(copy);
}

HuntCrossleyForce::HuntCrossleyForce(ContactParameters* params) : Force(),
	_contactParametersSetProp(PropertyObj("", ContactParametersSet())),
    _contactParametersSet((ContactParametersSet&)_contactParametersSetProp.getValueObj()),
    _transitionVelocityProp(PropertyDbl("", 0.01)),
    _transitionVelocity(_transitionVelocityProp.getValueDbl())
{
	setupProperties();
    addContactParameters(params);
}


void HuntCrossleyForce::createSystem(SimTK::MultibodySystem& system) const
{
    SimTK::GeneralContactSubsystem& contacts = system.updContactSubsystem();
    SimTK::SimbodyMatterSubsystem& matter = system.updMatterSubsystem();
    SimTK::ContactSetIndex set = contacts.createContactSet();
    SimTK::HuntCrossleyForce force(_model->updUserForceSubsystem(), contacts, set);
    force.setTransitionVelocity(_transitionVelocity);
    for (int i = 0; i < _contactParametersSet.getSize(); ++i)
    {
        ContactParameters& params = _contactParametersSet.get(i);
        for (int j = 0; j < params.getGeometry().getSize(); ++j)
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
    setType("HuntCrossleyForce");
	_contactParametersSetProp.setName("contact_parameters");
	_propertySet.append(&_contactParametersSetProp);
    _transitionVelocityProp.setName("transition_velocity");
	_propertySet.append(&_transitionVelocityProp);
}

void HuntCrossleyForce::copyData(const HuntCrossleyForce& copy)
{
    _contactParametersSet = copy._contactParametersSet;
}

//_____________________________________________________________________________
/**
 * Copy this force and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this Force.
 */
Object* HuntCrossleyForce::copy() const
{
	HuntCrossleyForce *force = new HuntCrossleyForce(*this);
	return(force);
}

HuntCrossleyForce::ContactParametersSet& HuntCrossleyForce::updContactParametersSet()
{
    return _contactParametersSet;
}

const HuntCrossleyForce::ContactParametersSet& HuntCrossleyForce::getContactParametersSet()
{
    return _contactParametersSet;
}

void HuntCrossleyForce::addContactParameters(HuntCrossleyForce::ContactParameters* params)
{
    _contactParametersSet.append(params);
}

double HuntCrossleyForce::getTransitionVelocity() const
{
    return _transitionVelocity;
}

void HuntCrossleyForce::setTransitionVelocity(double velocity)
{
    _transitionVelocity = velocity;
}

HuntCrossleyForce::ContactParameters::ContactParameters() :
    _geometry(_geometryProp.getValueStrArray()),
    _stiffness(_stiffnessProp.getValueDbl()),
    _dissipation(_dissipationProp.getValueDbl()),
    _staticFriction(_staticFrictionProp.getValueDbl()),
    _dynamicFriction(_dynamicFrictionProp.getValueDbl()),
    _viscousFriction(_viscousFrictionProp.getValueDbl())
{
    setupProperties();
}

HuntCrossleyForce::ContactParameters::ContactParameters(const ContactParameters& copy) :
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

HuntCrossleyForce::ContactParameters::ContactParameters(double stiffness, double dissipation, double staticFriction, double dynamicFriction, double viscousFriction) :
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

void HuntCrossleyForce::ContactParameters::copyData(const ContactParameters& copy)
{
    _geometry = copy._geometry;
    _stiffness = copy._stiffness;
    _dissipation = copy._dissipation;
    _staticFriction = copy._staticFriction;
    _dynamicFriction = copy._dynamicFriction;
    _viscousFriction = copy._viscousFriction;
}

Object* HuntCrossleyForce::ContactParameters::copy() const
{
	ContactParameters *cps = new ContactParameters(*this);
	return(cps);
}


void HuntCrossleyForce::ContactParameters::setupProperties()
{
    setType("HuntCrossleyForce::ContactParameters");
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

const Array<std::string>& HuntCrossleyForce::ContactParameters::getGeometry() const
{
    return _geometry;
}

Array<std::string>& HuntCrossleyForce::ContactParameters::updGeometry()
{
    return _geometry;
}

void HuntCrossleyForce::ContactParameters::addGeometry(const std::string& name)
{
    _geometry.append(name);
}

double HuntCrossleyForce::ContactParameters::getStiffness() const
{
    return _stiffness;
}

void HuntCrossleyForce::ContactParameters::setStiffness(double stiffness)
{
    _stiffness = stiffness;
}

double HuntCrossleyForce::ContactParameters::getDissipation() const
{
    return _dissipation;
}

void HuntCrossleyForce::ContactParameters::setDissipation(double dissipation)
{
    _dissipation = dissipation;
}

double HuntCrossleyForce::ContactParameters::getStaticFriction() const
{
    return _staticFriction;
}

void HuntCrossleyForce::ContactParameters::setStaticFriction(double friction)
{
    _staticFriction = friction;
}

double HuntCrossleyForce::ContactParameters::getDynamicFriction() const
{
    return _dynamicFriction;
}

void HuntCrossleyForce::ContactParameters::setDynamicFriction(double friction)
{
    _dynamicFriction = friction;
}

double HuntCrossleyForce::ContactParameters::getViscousFriction() const
{
    return _viscousFriction;
}

void HuntCrossleyForce::ContactParameters::setViscousFriction(double friction)
{
    _viscousFriction = friction;
}

void HuntCrossleyForce::ContactParametersSet::setNull()
{
	setType("HuntCrossleyForce::ContactParametersSet");
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

Object* HuntCrossleyForce::ContactParametersSet::copy() const
{
	ContactParametersSet *cpsSet = new ContactParametersSet(*this);
	return(cpsSet);
}

}// end of namespace OpenSim
