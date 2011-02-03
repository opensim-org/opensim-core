// PrescribedForce.cpp
// Author: Peter Eastman
/*
* Copyright (c) 2008, Stanford University. All rights reserved. 
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "PrescribedForce.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Common/FunctionSet.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace OpenSim;
using SimTK::Vec3;
using namespace std;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
PrescribedForce::~PrescribedForce()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PrescribedForce::PrescribedForce(Body* body) : Force(),
	_bodyName(_bodyNameProp.getValueStr()),
	_pointIsGlobal(_pointIsGlobalProp.getValueBool()),
	_forceIsGlobal(_forceIsGlobalProp.getValueBool()),
	_forceFunctionSetProp(PropertyObj("", FunctionSet())),
    _forceFunctionSet((FunctionSet&)_forceFunctionSetProp.getValueObj()),
	_pointFunctionSetProp(PropertyObj("", FunctionSet())),
    _pointFunctionSet((FunctionSet&)_pointFunctionSetProp.getValueObj()),
	_torqueFunctionSetProp(PropertyObj("", FunctionSet())),
    _torqueFunctionSet((FunctionSet&)_torqueFunctionSetProp.getValueObj())
{
	setNull();
	setupProperties();
	updateFromXMLNode();

	_body = body;
	if (_body)
		_bodyName = _body->getName();
}

//_____________________________________________________________________________
/**
 * Constructor from XML file
 */
PrescribedForce::PrescribedForce(DOMElement* aNode) :
	Force(aNode),
	_bodyName(_bodyNameProp.getValueStr()),
	_pointIsGlobal(_pointIsGlobalProp.getValueBool()),
	_forceIsGlobal(_forceIsGlobalProp.getValueBool()),
	_forceFunctionSetProp(PropertyObj("", FunctionSet())),
    _forceFunctionSet((FunctionSet&)_forceFunctionSetProp.getValueObj()),
	_pointFunctionSetProp(PropertyObj("", FunctionSet())),
    _pointFunctionSet((FunctionSet&)_pointFunctionSetProp.getValueObj()),
	_torqueFunctionSetProp(PropertyObj("", FunctionSet())),
    _torqueFunctionSet((FunctionSet&)_torqueFunctionSetProp.getValueObj())
{
	setNull();
	setupProperties();
	updateFromXMLNode();

	_body = NULL;
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
PrescribedForce::PrescribedForce(const PrescribedForce& force) :
	Force(force),
	_bodyName(_bodyNameProp.getValueStr()),
	_pointIsGlobal(_pointIsGlobalProp.getValueBool()),
	_forceIsGlobal(_forceIsGlobalProp.getValueBool()),
	_forceFunctionSetProp(PropertyObj("", FunctionSet())),
    _forceFunctionSet((FunctionSet&)_forceFunctionSetProp.getValueObj()),
	_pointFunctionSetProp(PropertyObj("", FunctionSet())),
    _pointFunctionSet((FunctionSet&)_pointFunctionSetProp.getValueObj()),
	_torqueFunctionSetProp(PropertyObj("", FunctionSet())),
    _torqueFunctionSet((FunctionSet&)_torqueFunctionSetProp.getValueObj())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
	copyData(force);
}

//_____________________________________________________________________________
/**
 * Copy this force and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this force.
 */
Object* PrescribedForce::copy() const
{
	PrescribedForce *force = new PrescribedForce(*this);
	return force;
}

//-----------------------------------------------------------------------------
// UPDATE FROM XML NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update this object based on its XML node.
 */
void PrescribedForce::updateFromXMLNode()
{
	// Base class
	Force::updateFromXMLNode();

	//Specify all or none of the components
	if(_forceFunctionSet.getSize() != 3&& _forceFunctionSet.getSize() != 0)
	{
		throw Exception("PrescribedForce:: three components of the force must be specified.");
	}

	if(_pointFunctionSet.getSize() != 3 && _pointFunctionSet.getSize() != 0)
	{
		throw Exception("PrescribedForce:: three components of the point must be specified.");
	}

	if(_torqueFunctionSet.getSize() != 3 && _torqueFunctionSet.getSize() != 0)
	{
		throw Exception("PrescribedForce:: three components of the torque must be specified.");
	}
}	


/**
 * Connect properties to local pointers.
 */
void PrescribedForce::setupProperties()
{
	string comment;

	// Body name
	comment = "Name of the body the force is applied to.";
	_bodyNameProp.setComment(comment);
	_bodyNameProp.setName("body");
	_propertySet.append(&_bodyNameProp);

	// Point is Global 
	comment = "Flag indicating whether the point (specified in pointFunctions) is in global frame";
	_pointIsGlobalProp.setComment(comment);
	_pointIsGlobalProp.setName("pointIsGlobal");
	_pointIsGlobalProp.setValue(false);
	_propertySet.append(&_pointIsGlobalProp);

	// Force/Torque is global 
	comment = "Flag indicating whether the quantities (specified in force/torqueFunctions) is in global frame";
	_forceIsGlobalProp.setComment(comment);
	_forceIsGlobalProp.setName("forceIsGlobal");
	_forceIsGlobalProp.setValue(true);
	_propertySet.append(&_forceIsGlobalProp);

	comment = "Three functions describing the force to be applied.";
	_forceFunctionSetProp.setComment(comment);
	_forceFunctionSetProp.setName("forceFunctions");
	_forceFunctionSet.setName("forceFunctions");
	_forceFunctionSetProp.setMatchName(true);
	_forceFunctionSetProp.setAllowableArraySize(3);
	_propertySet.append(&_forceFunctionSetProp);

	comment = "Three functions describing the location at which the force is applied";
	_pointFunctionSetProp.setComment(comment);
	_pointFunctionSetProp.setName("pointFunctions");
	_pointFunctionSet.setName("pointFunctions");
	_pointFunctionSetProp.setMatchName(true);
	_pointFunctionSetProp.setAllowableArraySize(0, 3);
	_propertySet.append(&_pointFunctionSetProp);

	comment = "Three functions describing the torque the PrescribedForce applies";
	_torqueFunctionSetProp.setComment(comment);
	_torqueFunctionSetProp.setName("torqueFunctions");
	_torqueFunctionSet.setName("torqueFunctions");
	_torqueFunctionSetProp.setMatchName(true);
	_torqueFunctionSetProp.setAllowableArraySize(0, 3);
	_propertySet.append(&_torqueFunctionSetProp);

	
}

PrescribedForce& PrescribedForce::operator=(const PrescribedForce &aForce)
{
	Force::operator=(aForce);
	copyData(aForce);
	return(*this);
}

void PrescribedForce::setForceFunctions(Function* forceX, Function* forceY, Function* forceZ)
{
	_forceFunctionSet.setSize(0);
	_forceFunctionSet.append(forceX);
	_forceFunctionSet.append(forceY);
	_forceFunctionSet.append(forceZ);
}


void PrescribedForce::setPointFunctions(Function* pointX, Function* pointY, Function* pointZ)
{
	_pointFunctionSet.setSize(0);
	_pointFunctionSet.append(pointX);
	_pointFunctionSet.append(pointY);
	_pointFunctionSet.append(pointZ);
}

void PrescribedForce::setTorqueFunctions(Function* torqueX, Function* torqueY, Function* torqueZ)
{
	_torqueFunctionSet.setSize(0);
	_torqueFunctionSet.append(torqueX);
	_torqueFunctionSet.append(torqueY);
	_torqueFunctionSet.append(torqueZ);

}

void PrescribedForce::setTorqueFunctionNames(const OpenSim::Array<std::string>& aFunctionNames, 
											 const Storage& kineticsStore)  
{
	int forceSize = kineticsStore.getSize();
	if(forceSize<=0) return;
	double *t=0;
	// Expected column labels for the file
	kineticsStore.getTimeColumn(t);
	double *column=0;
	NaturalCubicSpline** tSpline = new NaturalCubicSpline*[3];
	for(int i=0;i<aFunctionNames.getSize();i++)
	{
		kineticsStore.getDataColumn(aFunctionNames[i], column);
		tSpline[i]= new NaturalCubicSpline((forceSize>10?10:forceSize), t, column, aFunctionNames[i]);
	}
	setTorqueFunctions(tSpline[0], tSpline[1], tSpline[2]);
	for (int i=0; i<aFunctionNames.getSize();i++)
		_torqueFunctionSet.get(i).setName(aFunctionNames.get(i));
}
void PrescribedForce::setForceFunctionNames(const OpenSim::Array<std::string>& aFunctionNames, 
											 const Storage& kineticsStore)  
{
	int forceSize = kineticsStore.getSize();
	if(forceSize<=0) return;
	double *t=0;
	// Expected column labels for the file
	kineticsStore.getTimeColumn(t);
	double *column=0;
	NaturalCubicSpline** tSpline = new NaturalCubicSpline*[3];
	for(int i=0;i<aFunctionNames.getSize();i++)
	{
		kineticsStore.getDataColumn(aFunctionNames[i], column);
		tSpline[i]= new NaturalCubicSpline((forceSize>10?10:forceSize), t, column, aFunctionNames[i]);
	}
	setForceFunctions(tSpline[0], tSpline[1], tSpline[2]);
	for (int i=0; i<aFunctionNames.getSize();i++)
		_forceFunctionSet.get(i).setName(aFunctionNames.get(i));
}
void PrescribedForce::setPointFunctionNames(const OpenSim::Array<std::string>& aFunctionNames, 
											 const Storage& kineticsStore)  
{
	int forceSize = kineticsStore.getSize();
	if(forceSize<=0) return;
	double *t=0;
	// Expected column labels for the file
	kineticsStore.getTimeColumn(t);
	double *column=0;
	NaturalCubicSpline** tSpline = new NaturalCubicSpline*[3];
	for(int i=0;i<aFunctionNames.getSize();i++)
	{
		kineticsStore.getDataColumn(aFunctionNames[i], column);
		tSpline[i]= new NaturalCubicSpline((forceSize>10?10:forceSize), t, column, aFunctionNames[i]);
	}
	setPointFunctions(tSpline[0], tSpline[1], tSpline[2]);
	for (int i=0; i<aFunctionNames.getSize();i++)
		_pointFunctionSet.get(i).setName(aFunctionNames.get(i));
}


bool PrescribedForce::getForceIsInGlobalFrame() const
{
	return _forceIsGlobal;
}
void PrescribedForce::setForceIsInGlobalFrame(bool isGlobal)
{
	_forceIsGlobal = isGlobal;
}
bool PrescribedForce::getPointIsInGlobalFrame() const
{
	return _pointIsGlobal;
}
void PrescribedForce::setPointIsInGlobalFrame(bool isGlobal)
{
	_pointIsGlobal = isGlobal;
}

//-----------------------------------------------------------------------------
// ABSTRACT METHODS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________

void PrescribedForce::computeForce(const SimTK::State& state, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const
{
	double time = state.getTime();
	const SimbodyEngine& engine = getModel().getSimbodyEngine();
	SimTK::Vector  timeAsVector(1, time);

	assert(_body!=0);
	Function* forceX=NULL;
	Function* forceY=NULL;
	Function* forceZ=NULL;
	if (_forceFunctionSet.getSize()==3){
		forceX=&_forceFunctionSet[0];	forceY=&_forceFunctionSet[1];	forceZ=&_forceFunctionSet[2];
	}
	if (forceX != NULL) {
		Function* pointX=NULL;
		Function* pointY=NULL;
		Function* pointZ=NULL;
		if (_pointFunctionSet.getSize()==3){
			pointX=&_pointFunctionSet[0];	pointY=&_pointFunctionSet[1];	pointZ=&_pointFunctionSet[2];
		}
		Vec3 force(forceX?forceX->calcValue(timeAsVector):0.0, 
			forceY?forceY->calcValue(timeAsVector):0.0, 
			forceZ?forceZ->calcValue(timeAsVector):0.0);
		if (!_forceIsGlobal)
			engine.transform(state, *_body, force, engine.getGroundBody(), force);
		if (pointX == NULL) {
			applyForce(state, *_body, force, bodyForces);
 
	    }else {
			Vec3 point(pointX?pointX->calcValue(timeAsVector):0.0, 
				pointY?pointY->calcValue(timeAsVector):0.0, 
				pointZ?pointZ->calcValue(timeAsVector):0.0);
			if (_pointIsGlobal)
				engine.transformPosition(state, engine.getGroundBody(), point, *_body, point);
			applyForceToPoint(state, *_body, point, force, bodyForces);
		}
	}
	Function* torqueX=NULL;
	Function* torqueY=NULL;
	Function* torqueZ=NULL;
	if (_torqueFunctionSet.getSize()==3){
		torqueX=&_torqueFunctionSet[0];	torqueY=&_torqueFunctionSet[1];	torqueZ=&_torqueFunctionSet[2];
	}
	if (torqueX != NULL) {
		Vec3 torque(torqueX?torqueX->calcValue(timeAsVector):0.0, 
			torqueY?torqueY->calcValue(timeAsVector):0.0, 
			torqueZ?torqueZ->calcValue(timeAsVector):0.0);
		if (!_forceIsGlobal)
			engine.transform(state, *_body, torque, engine.getGroundBody(), torque);
		applyTorque(state, *_body, torque, bodyForces);
	}
}

/**
 * Conevenince methods to access prescribed force functions
 */
Vec3 PrescribedForce::getForceAtTime(double aTime) const	
{
	SimTK::Vector timeAsVector(1, aTime);
	Function* forceX=NULL;
	Function* forceY=NULL;
	Function* forceZ=NULL;
	if (_forceFunctionSet.getSize()==3){
		forceX=&_forceFunctionSet[0];	forceY=&_forceFunctionSet[1];	forceZ=&_forceFunctionSet[2];
	}
	Vec3 force(forceX?forceX->calcValue(timeAsVector):0.0, 
		forceY?forceY->calcValue(timeAsVector):0.0, 
		forceZ?forceZ->calcValue(timeAsVector):0.0);
	return force;
}

Vec3 PrescribedForce::getPointAtTime(double aTime) const
{
	SimTK::Vector timeAsVector(1, aTime);
	Function* pointX=NULL;
	Function* pointY=NULL;
	Function* pointZ=NULL;
	if (_pointFunctionSet.getSize()==3){
		pointX=&_pointFunctionSet[0];	pointY=&_pointFunctionSet[1];	pointZ=&_pointFunctionSet[2];
	}
	Vec3 point(pointX?pointX->calcValue(timeAsVector):0.0, 
		pointY?pointY->calcValue(timeAsVector):0.0, 
		pointZ?pointZ->calcValue(timeAsVector):0.0);
	return point;
}

Vec3 PrescribedForce::getTorqueAtTime(double aTime) const
{
	SimTK::Vector timeAsVector(1, aTime);
	Function* torqueX=NULL;
	Function* torqueY=NULL;
	Function* torqueZ=NULL;
	if (_torqueFunctionSet.getSize()==3){
		torqueX=&_torqueFunctionSet[0];	torqueY=&_torqueFunctionSet[1];	torqueZ=&_torqueFunctionSet[2];
	}
	Vec3 torque(torqueX?torqueX->calcValue(timeAsVector):0.0, 
		torqueY?torqueY->calcValue(timeAsVector):0.0, 
		torqueZ?torqueZ->calcValue(timeAsVector):0.0);
	return torque;
}


//-----------------------------------------------------------------------------
// Reporting
//-----------------------------------------------------------------------------

OpenSim::Array<std::string> PrescribedForce::getRecordLabels() const {
	OpenSim::Array<std::string> labels("");

	// Compute flags to find out if a point is specified or not and if a torque or force or both are specified
	Function* forceX=NULL;
	Function* forceY=NULL;
	Function* forceZ=NULL;
	if (_forceFunctionSet.getSize()==3){
		forceX=&_forceFunctionSet[0];	forceY=&_forceFunctionSet[1];	forceZ=&_forceFunctionSet[2];
	}
	Function* pointX=NULL;
	Function* pointY=NULL;
	Function* pointZ=NULL;
	if (_pointFunctionSet.getSize()==3){
		pointX=&_pointFunctionSet[0];	pointY=&_pointFunctionSet[1];	pointZ=&_pointFunctionSet[2];
	}
	Function* torqueX=NULL;
	Function* torqueY=NULL;
	Function* torqueZ=NULL;
	if (_torqueFunctionSet.getSize()==3){
		torqueX=&_torqueFunctionSet[0];	torqueY=&_torqueFunctionSet[1];	torqueZ=&_torqueFunctionSet[2];
	}
	bool appliesForce = (forceX!=NULL) || (forceY!=NULL) || (forceZ!=NULL);
	bool pointSpecified = (pointX!=NULL) || (pointY!=NULL) || (pointZ!=NULL);
	// Instead of trying to figure out which values are not specified we'll write NaNs 
	std::string BodyToReport = (_forceIsGlobal?"ground":_body->getName());
	if (appliesForce) {
		labels.append(BodyToReport+"_"+getName()+"_fx");
		labels.append(BodyToReport+"_"+getName()+"_fy");
		labels.append(BodyToReport+"_"+getName()+"_fz");
	}
	if (pointSpecified) {
		labels.append(BodyToReport+"_"+getName()+"_px");
		labels.append(BodyToReport+"_"+getName()+"_py");
		labels.append(BodyToReport+"_"+getName()+"_pz");
	}
	if (torqueX!=NULL){
		labels.append(BodyToReport+"_"+getName()+"_torque_x");
		labels.append(BodyToReport+"_"+getName()+"_torque_y");
		labels.append(BodyToReport+"_"+getName()+"_torque_z");
	}
	return labels;
}
/**
 * Given SimTK::State object extract all the values necessary to report forces, application location
 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
OpenSim::Array<double> PrescribedForce::getRecordValues(const SimTK::State& state) const {
	OpenSim::Array<double>	values(SimTK::NaN);
	assert(_body!=0);
	// This is bad as it duplicates the code in computeForce we'll cleanup after it works!
	double time = state.getTime();
	const SimbodyEngine& engine = getModel().getSimbodyEngine();
	SimTK::Vector  timeAsVector(1, time);

	Function* forceX=NULL;
	Function* forceY=NULL;
	Function* forceZ=NULL;
	if (_forceFunctionSet.getSize()==3){
		forceX=&_forceFunctionSet[0];	forceY=&_forceFunctionSet[1];	forceZ=&_forceFunctionSet[2];
	}
	Function* pointX=NULL;
	Function* pointY=NULL;
	Function* pointZ=NULL;
	if (_pointFunctionSet.getSize()==3){
		pointX=&_pointFunctionSet[0];	pointY=&_pointFunctionSet[1];	pointZ=&_pointFunctionSet[2];
	}
	Function* torqueX=NULL;
	Function* torqueY=NULL;
	Function* torqueZ=NULL;
	if (_torqueFunctionSet.getSize()==3){
		torqueX=&_torqueFunctionSet[0];	torqueY=&_torqueFunctionSet[1];	torqueZ=&_torqueFunctionSet[2];
	}
	if (forceX != NULL) {
		Vec3 force(forceX?forceX->calcValue(timeAsVector):0.0, 
			forceY?forceY->calcValue(timeAsVector):0.0, 
			forceZ?forceZ->calcValue(timeAsVector):0.0);
		if (!_forceIsGlobal)
			engine.transform(state, *_body, force, engine.getGroundBody(), force);
		if (pointX == NULL) {
			//applyForce(*_body, force);
			for (int i=0; i<3; i++) values.append(force[i]);
	    }else {
			Vec3 point(pointX?pointX->calcValue(timeAsVector):0.0, 
				pointY?pointY->calcValue(timeAsVector):0.0, 
				pointZ?pointZ->calcValue(timeAsVector):0.0);
			if (_pointIsGlobal)
				engine.transformPosition(state, engine.getGroundBody(), point, *_body, point);
			//applyForceToPoint(*_body, point, force);
			for (int i=0; i<3; i++) values.append(force[i]);
			for (int i=0; i<3; i++) values.append(point[i]);
		}
	}
	if (torqueX != NULL) {
		Vec3 torque(torqueX?torqueX->calcValue(timeAsVector):0.0, 
			torqueY?torqueY->calcValue(timeAsVector):0.0, 
			torqueZ?torqueZ->calcValue(timeAsVector):0.0);
		if (!_forceIsGlobal)
			engine.transform(state, *_body, torque, engine.getGroundBody(), torque);
		for (int i=0; i<3; i++) values.append(torque[i]);
		//applyTorque(*_body, torque);
	}
	return values;
};

void PrescribedForce::setNull()
{
	_forceIsGlobal=true;
	_pointIsGlobal=false;
	setType("PrescribedForce");

	_forceFunctionSet.setMemoryOwner(false);
	_pointFunctionSet.setMemoryOwner(false);
	_torqueFunctionSet.setMemoryOwner(false);
}

void PrescribedForce::copyData(const PrescribedForce& orig)
{
	_forceIsGlobal = orig._forceIsGlobal;
	_pointIsGlobal = orig._pointIsGlobal;

	_forceFunctionSet = orig._forceFunctionSet;
	_pointFunctionSet = orig._pointFunctionSet;
	_torqueFunctionSet = orig._torqueFunctionSet;

	_bodyName = orig._bodyName;
	_body = orig._body;
}

void PrescribedForce::setup(Model& model)
{
	Force::setup(model);

	// hook up body pointer to name
	if (_model)
		_body = &_model->updBodySet().get(_bodyName);
}
