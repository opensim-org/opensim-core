// PointToPointSpring.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2009, Stanford University. All rights reserved. 
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
    constructProperties();
}
//_____________________________________________________________________________
// Convenience constructor for API users.
PointToPointSpring::PointToPointSpring(string body1Name, SimTK::Vec3 point1, 
									   string body2Name, SimTK::Vec3 point2, 
                                       double stiffness, double restlength )
{
	setNull();
    constructProperties();

    // Set properties to the passed-in values.
	setBody1Name(body1Name);
	setBody2Name(body2Name);

	setPoint1(point1);
	setPoint2(point2);

	setStiffness(stiffness);
	setRestlength(restlength);
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
}


//_____________________________________________________________________________
/**
 * Construct properties and initialize to their default values.
 */
void PointToPointSpring::constructProperties()
{
	constructProperty_body1();
	constructProperty_body2();

	const SimTK::Vec3 bodyOrigin(0.0, 0.0, 0.0);
	constructProperty_point1(bodyOrigin);
	constructProperty_point2(bodyOrigin);

	constructProperty_stiffness(1.0);
	constructProperty_rest_length(0.0);
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
	const OpenSim::Body& body1 = _model->getBodySet().get(getBody1Name());
	const OpenSim::Body& body2 = _model->getBodySet().get(getBody2Name());
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

	string errorMessage;

	const string& body1Name = getBody1Name();
	const string& body2Name = getBody2Name();

	// Look up the two bodies being connected together by name in the model
	if (!model.updBodySet().contains(body1Name)) {
		errorMessage = "Invalid body1 (" + body1Name + ") specified for Spring "
                        + getName();
		throw (Exception(errorMessage));
	}
	if (!model.updBodySet().contains(body2Name)) {
		errorMessage = "Invalid body2 (" + body2Name + ") specified for Spring "
                        + getName();
		throw (Exception(errorMessage));
	}

	if(getName() == "")
		setName("spring");
}

//=============================================================================
// Create the underlying system component(s)
//=============================================================================
void PointToPointSpring::addToSystem(SimTK::MultibodySystem& system) const
{
	Super::addToSystem(system);    // Base class first.

	Body& body1 = _model->updBodySet().get(getBody1Name());
	Body& body2 = _model->updBodySet().get(getBody2Name());

	// Get underlying mobilized bodies
	SimTK::MobilizedBody b1 = _model->getMatterSubsystem()
                                        .getMobilizedBody(body1.getIndex());
	SimTK::MobilizedBody b2 = _model->getMatterSubsystem()
                                        .getMobilizedBody(body2.getIndex());

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
	const string& body1Name = getBody1Name();
	const string& body2Name = getBody2Name();

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

	const Body& body1 = _model->getBodySet().get(getBody1Name());
	const Body& body2 = _model->getBodySet().get(getBody2Name());

	//get the net force added to the system contributed by the Spring
	simtkSpring.calcForceContribution(state, bodyForces, particleForces, 
                                      mobilityForces);
	SimTK::Vec3 forces = bodyForces(body1.getIndex())[1];
	values.append(3, &forces[0]);

	SimTK::Vec3 gpoint(0);
	_model->getSimbodyEngine().getPosition(state, body1, getPoint1(), gpoint);
	values.append(3, &gpoint[0]);

	forces = bodyForces(body2.getIndex())[1];
	values.append(3, &forces[0]);

	_model->getSimbodyEngine().getPosition(state, body2, getPoint2(), gpoint);
	values.append(3, &gpoint[0]);

	return values;
}

