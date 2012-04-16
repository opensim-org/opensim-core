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
/**
 * Destructor.
 */
PointToPointSpring::~PointToPointSpring()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 *
 */
PointToPointSpring::PointToPointSpring() : 	Force()
{
	setNull();
	if (_displayer.countGeometry()==0){
		Geometry *g = new LineGeometry();
		g->setFixed(false);
		_displayer.addGeometry(g);
	}
}
//_____________________________________________________________________________
/**
 * Convenience constructor.
 * @param body1Name name of the first body to which the spring is attached
 * @param point1 location where spring is attached on body1
 * @param body2Name name of the second body to which the spring is attached
 * @param point2 location where spring is attached on body2
 */
PointToPointSpring::PointToPointSpring(string body1Name, SimTK::Vec3 point1, 
									   string body2Name, SimTK::Vec3 point2, double stiffness, double restlength ) :
	Force()
{
	// NULL
	setNull();

	// MEMBER VARIABLES
	setPropertyValue("body1", body1Name);
	setPropertyValue("body2", body2Name);

	setPropertyValue("point1", point1);
	setPropertyValue("point2", point2);

	setPropertyValue("stiffness", stiffness);
	setPropertyValue("rest_length", restlength);
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForce force to be copied.
 */
PointToPointSpring::PointToPointSpring(const PointToPointSpring &aForce) :
	Force(aForce)
{
	setNull();
	copyData(aForce);
		if (_displayer.countGeometry()==0){
		Geometry *g = new LineGeometry();
		g->setFixed(false);
		_displayer.addGeometry(g);
	}

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
	setupProperties();
}


//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PointToPointSpring::setupProperties()
{
	addProperty<string>("body1",
		"Name of Body to which 1 end of the spring is attached.",
		"");
	addProperty<string>("body2",
		"Name of Body to which the 2nd end of the spring is attached.",
		"");
	SimTK::Vec3 pointZero(0.0, 0.0, 0.0);
	addProperty<SimTK::Vec3>("point1",
		"",
		pointZero);
	addProperty<SimTK::Vec3>("point2",
		"",
		pointZero);
	addProperty<double>("stiffness",
		"",
		1.0);
	addProperty<double>("rest_length",
		"",
		0.0);
}

//_____________________________________________________________________________
/**
 * Copy the member data of the specified force.
 * @param aPointToPointSpring PointToPointSpring providing the data to be copied
 */
void PointToPointSpring::copyData(const PointToPointSpring &aPointToPointSpring)
{
	// MEMBER VARIABLES
	setPropertyValue("body1", aPointToPointSpring.getPropertyValue<string>("body1"));
	setPropertyValue("body2", aPointToPointSpring.getPropertyValue<string>("body2"));
	setPropertyValue("point1", aPointToPointSpring.getPropertyValue<SimTK::Vec3>("point1"));
	setPropertyValue("point2", aPointToPointSpring.getPropertyValue<SimTK::Vec3>("point2"));

	setStiffness(aPointToPointSpring.getStiffness());
	setRestlength(aPointToPointSpring.getRestlength());
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
	const OpenSim::Body& body1 = _model->getBodySet().get(getPropertyValue<string>("body1"));
	const OpenSim::Body& body2 = _model->getBodySet().get(getPropertyValue<string>("body2"));
	_model->getSimbodyEngine().transformPosition(s, body1, getPropertyValue<SimTK::Vec3>("point1"), globalLocation1);
	_model->getSimbodyEngine().transformPosition(s, body2, getPropertyValue<SimTK::Vec3>("point2"), globalLocation2);
	((LineGeometry *)_displayer.getGeometry(0))->setPoints(globalLocation1, globalLocation2);
}
void PointToPointSpring::updateGeometry(const SimTK::State& s)
{
	int x=0;
	if (_displayer.countGeometry()==0){
		Geometry *g = new LineGeometry();
		g->setFixed(false);
		_displayer.addGeometry(g);
	}
	updateDisplayer(s);
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return  aBodyID ID (or number, or index) of the generalized Body.
 */
PointToPointSpring& PointToPointSpring::operator=(const PointToPointSpring &aPointToPointSpring)
{
	// BASE CLASS
	Force::operator =(aPointToPointSpring);

	copyData(aPointToPointSpring);

	return(*this);
}

//=============================================================================
// Create the underlying system component(s)
//=============================================================================
void PointToPointSpring::setup(Model& aModel)
{
	string errorMessage;

	const string &body1Name = getPropertyValue<string>("body1");
	const string &body2Name = getPropertyValue<string>("body2");

	// Base class
	Force::setup(aModel);

	// Look up the two bodies being connected together by name in the model
	if (!aModel.updBodySet().contains(body1Name)) {
		errorMessage = "Invalid body1 (" + body1Name + ") specified for Spring " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	if (!aModel.updBodySet().contains(body2Name)) {
		errorMessage = "Invalid body2 (" + body2Name + ") specified for Spring " + getName();
		throw (Exception(errorMessage.c_str()));
	}

	if(getName() == "")
		setName("spring");
}

//=============================================================================
// Create the underlying system component(s)
//=============================================================================
void PointToPointSpring::createSystem(SimTK::MultibodySystem& system) const
{
	Body &body1 = _model->updBodySet().get(getPropertyValue<string>("body1"));
	Body &body2 = _model->updBodySet().get(getPropertyValue<string>("body2"));

	// Get underlying mobilized bodies
	SimTK::MobilizedBody b1 = _model->updMatterSubsystem().getMobilizedBody(body1.getIndex());
	SimTK::MobilizedBody b2 = _model->updMatterSubsystem().getMobilizedBody(body2.getIndex());

    // Now create a Simbody Force::TwoPointLinearSpring
    SimTK::Force::TwoPointLinearSpring simtkSpring(_model->updForceSubsystem(), b1, getPropertyValue<SimTK::Vec3>("point1"), b2, getPropertyValue<SimTK::Vec3>("point2"), getPropertyValue<double>("stiffness"), getPropertyValue<double>("rest_length"));
    
    // Beyond the const Component get the index so we can access the SimTK::Constraint later
	PointToPointSpring* mutableThis = const_cast<PointToPointSpring *>(this);
	mutableThis->_index = simtkSpring.getForceIndex();

}



//=============================================================================
// Reporting
//=============================================================================
/** 
 * Provide names of the quantities (column labels) of the force value(s) reported
 * 
 */
OpenSim::Array<std::string> PointToPointSpring::getRecordLabels() const 
{
	const string &body1Name = getPropertyValue<string>("body1");
	const string &body2Name = getPropertyValue<string>("body2");

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
/**
 * Provide the value(s) to be reported that correspond to the labels
 */
OpenSim::Array<double> PointToPointSpring::getRecordValues(const SimTK::State& state) const 
{
	OpenSim::Array<double> values(1);

	const string &body1Name = getPropertyValue<string>("body1");
	const string &body2Name = getPropertyValue<string>("body2");
	const SimTK::Vec3 &point1 = getPropertyValue<SimTK::Vec3>("point1");

	const SimTK::Force::TwoPointLinearSpring &simtkSpring = (SimTK::Force::TwoPointLinearSpring &)(_model->getForceSubsystem().getForce(_index));

	SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
	SimTK::Vector_<SimTK::Vec3> particleForces(0);
	SimTK::Vector mobilityForces(0);

	//get the net force added to the system contributed by the Spring
	simtkSpring.calcForceContribution(state, bodyForces, particleForces, mobilityForces);
	SimTK::Vec3 forces = bodyForces(_model->getBodySet().get(body1Name).getIndex())[1];
	values.append(3, &forces[0]);

	SimTK::Vec3 gpoint(0);
	_model->getSimbodyEngine().getPosition(state, _model->getBodySet().get(body1Name), point1, gpoint);
	values.append(3, &gpoint[0]);

	forces = bodyForces(_model->getBodySet().get(body2Name).getIndex())[1];
	values.append(3, &forces[0]);

	_model->getSimbodyEngine().getPosition(state, _model->getBodySet().get(body2Name), point1, gpoint);
	values.append(3, &gpoint[0]);

	return values;
}

