/* -------------------------------------------------------------------------- *
 *                      OpenSim:  PointToPointSpring.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "PointToPointSpring.h"
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;
using namespace std;


//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================

//_____________________________________________________________________________
//Default constructor.
PointToPointSpring::PointToPointSpring()
{
	setNull();
    constructInfrastructure();
}
//_____________________________________________________________________________
// Convenience constructor for API users.
PointToPointSpring::PointToPointSpring(const Body& body1, SimTK::Vec3 point1, 
                                       const Body& body2, SimTK::Vec3 point2,
                                       double stiffness, double restlength )
{
	setNull();
    constructInfrastructure();

    // Set properties to the passed-in values.
	setBody1(body1);
	setBody2(body2);

	setPoint1(point1);
	setPoint2(point2);

	setStiffness(stiffness);
	setRestlength(restlength);
}

void PointToPointSpring::constructConnectors()
{
    constructConnector<Body>("body1");
    constructConnector<Body>("body2");
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this force to their null values.
 */
void PointToPointSpring::setNull()
{
	setAuthors("Ajay Seth"); 
}


//_____________________________________________________________________________
/**
 * Construct properties and initialize to their default values.
 */
void PointToPointSpring::constructProperties()
{
	const SimTK::Vec3 bodyOrigin(0.0, 0.0, 0.0);
	constructProperty_point1(bodyOrigin);
	constructProperty_point2(bodyOrigin);

	constructProperty_stiffness(1.0);
	constructProperty_rest_length(0.0);
}

void PointToPointSpring::setBody1(const Body& body)
{
    updConnector<Body>("body1").connect(body);
}

void PointToPointSpring::setBody2(const Body& body)
{
    updConnector<Body>("body2").connect(body);
}

const Body& PointToPointSpring::getBody1() const
{
    return getConnector<Body>("body1").getConnectee();
}

const Body& PointToPointSpring::getBody2() const
{
    return getConnector<Body>("body2").getConnectee();
}


//_____________________________________________________________________________
/**
 * Get the visible object used to represent the spring.
 */
VisibleObject* PointToPointSpring::getDisplayer() const
{ 
	return const_cast<VisibleObject*>(&_displayer); 
}

void PointToPointSpring::updateDisplayer(const SimTK::State& s)
{
	SimTK::Vec3 globalLocation1, globalLocation2;
	const OpenSim::Body& body1 = getBody1();
	const OpenSim::Body& body2 = getBody2();
	_model->getSimbodyEngine().transformPosition(s, body1, getPoint1(), 
                                                 globalLocation1);
	_model->getSimbodyEngine().transformPosition(s, body2, getPoint2(),
                                                 globalLocation2);

	if (_displayer.countGeometry()==0){
		Geometry *g = new LineGeometry();
		g->setFixed(false);
		_displayer.addGeometry(g);
	}
	((LineGeometry *)_displayer.getGeometry(0))->
        setPoints(globalLocation1, globalLocation2);
}

void PointToPointSpring::updateGeometry(const SimTK::State& s)
{
	if (_displayer.countGeometry()==0){
		Geometry *g = new LineGeometry();
		g->setFixed(false);
		_displayer.addGeometry(g);
	}
	updateDisplayer(s);
}

//=============================================================================
// Connect this force element to the rest of the model.
//=============================================================================
void PointToPointSpring::connectToModel(Model& model)
{
	Super::connectToModel(model);	// Let base class connect first.

	if(getName() == "")
		setName("pointToPointSpring");
}

//=============================================================================
// Create the underlying system component(s)
//=============================================================================
void PointToPointSpring::addToSystem(SimTK::MultibodySystem& system) const
{
	Super::addToSystem(system);    // Base class first.

	const Body& body1 = getBody1();
	const Body& body2 = getBody2();

	// Get underlying mobilized bodies
	SimTK::MobilizedBody b1 = _model->getMatterSubsystem()
                                        .getMobilizedBody(body1.getMobilizedBodyIndex());
	SimTK::MobilizedBody b2 = _model->getMatterSubsystem()
                                        .getMobilizedBody(body2.getMobilizedBodyIndex());

    // Now create a Simbody Force::TwoPointLinearSpring
    SimTK::Force::TwoPointLinearSpring simtkSpring
       (_model->updForceSubsystem(), b1, getPoint1(), b2, getPoint2(), 
        getStiffness(), getRestlength());
    
    // Beyond the const Component get the index so we can access the 
    // SimTK::Force later.
	PointToPointSpring* mutableThis = const_cast<PointToPointSpring *>(this);
	mutableThis->_index = simtkSpring.getForceIndex();
}



//=============================================================================
// Reporting
//=============================================================================
// Provide names of the quantities (column labels) of the force value(s) 
// reported.
OpenSim::Array<std::string> PointToPointSpring::getRecordLabels() const 
{
	const string& body1Name = getBody1().getName();
	const string& body2Name = getBody2().getName();

	OpenSim::Array<std::string> labels("");
	labels.append(getName()+"."+body1Name+".force.X");
	labels.append(getName()+"."+body1Name+".force.Y");
	labels.append(getName()+"."+body1Name+".force.Z");
	labels.append(getName()+"."+body1Name+".point.X");
	labels.append(getName()+"."+body1Name+".point.Y");
	labels.append(getName()+"."+body1Name+".point.Z");
	labels.append(getName()+"."+body2Name+".force.X");
	labels.append(getName()+"."+body2Name+".force.Y");
	labels.append(getName()+"."+body2Name+".force.Z");
	labels.append(getName()+"."+body2Name+".point.X");
	labels.append(getName()+"."+body2Name+".point.Y");
	labels.append(getName()+"."+body2Name+".point.Z");

	return labels;
}

// Provide the value(s) to be reported that correspond to the labels.
OpenSim::Array<double> PointToPointSpring::
getRecordValues(const SimTK::State& state) const 
{
	OpenSim::Array<double> values(1);

	const SimTK::Force::TwoPointLinearSpring& 
        simtkSpring = SimTK::Force::TwoPointLinearSpring::downcast
                                (_model->getForceSubsystem().getForce(_index));

	SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
	SimTK::Vector_<SimTK::Vec3> particleForces(0);
	SimTK::Vector mobilityForces(0);

	const Body& body1 = getBody1();
	const Body& body2 = getBody2();

	//get the net force added to the system contributed by the Spring
	simtkSpring.calcForceContribution(state, bodyForces, particleForces, 
                                      mobilityForces);
	SimTK::Vec3 forces = bodyForces(body1.getMobilizedBodyIndex())[1];
	values.append(3, &forces[0]);

	SimTK::Vec3 gpoint(0);
	_model->getSimbodyEngine().getPosition(state, body1, getPoint1(), gpoint);
	values.append(3, &gpoint[0]);

	forces = bodyForces(body2.getMobilizedBodyIndex())[1];
	values.append(3, &forces[0]);

	_model->getSimbodyEngine().getPosition(state, body2, getPoint2(), gpoint);
	values.append(3, &gpoint[0]);

	return values;
}

