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
PrescribedForce::PrescribedForce(Body* body) :
	_bodyName(_bodyNameProp.getValueStr()),
	_pointIsGlobal(_pointIsGlobalProp.getValueBool()),
	_forceIsGlobal(_forceIsGlobalProp.getValueBool()),
	_forceFunctionSetProp(PropertyObj("", FunctionSet())),
    _forceFunctionSet((FunctionSet&)_forceFunctionSetProp.getValueObj()),
	_pointFunctionSetProp(PropertyObj("", FunctionSet())),
    _pointFunctionSet((FunctionSet&)_pointFunctionSetProp.getValueObj()),
	_torqueFunctionSetProp(PropertyObj("", FunctionSet())),
    _torqueFunctionSet((FunctionSet&)_torqueFunctionSetProp.getValueObj()),
	_forceX(NULL),
	_forceY(NULL),
	_forceZ(NULL),
	_pointX(NULL),
	_pointY(NULL),
	_pointZ(NULL),
	_torqueX(NULL),
	_torqueY(NULL),
	_torqueZ(NULL)
{
	setNull();
	setupProperties();
	_body = body;
	if (_body)
		_bodyName = _body->getName();
}

//_____________________________________________________________________________
/**
 * Constructor from XML file
 */
PrescribedForce::PrescribedForce(DOMElement* aNode) :
	CustomForce(aNode),
	_bodyName(_bodyNameProp.getValueStr()),
	_pointIsGlobal(_pointIsGlobalProp.getValueBool()),
	_forceIsGlobal(_forceIsGlobalProp.getValueBool()),
	_forceFunctionSetProp(PropertyObj("", FunctionSet())),
    _forceFunctionSet((FunctionSet&)_forceFunctionSetProp.getValueObj()),
	_pointFunctionSetProp(PropertyObj("", FunctionSet())),
    _pointFunctionSet((FunctionSet&)_pointFunctionSetProp.getValueObj()),
	_torqueFunctionSetProp(PropertyObj("", FunctionSet())),
    _torqueFunctionSet((FunctionSet&)_torqueFunctionSetProp.getValueObj()),
	_forceX(NULL),
	_forceY(NULL),
	_forceZ(NULL),
	_pointX(NULL),
	_pointY(NULL),
	_pointZ(NULL),
	_torqueX(NULL),
	_torqueY(NULL),
	_torqueZ(NULL)
{
	setNull();
	setupProperties();
	_body = NULL;
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
PrescribedForce::PrescribedForce(const PrescribedForce& force) :
	CustomForce(force),
	_bodyName(_bodyNameProp.getValueStr()),
	_pointIsGlobal(_pointIsGlobalProp.getValueBool()),
	_forceIsGlobal(_forceIsGlobalProp.getValueBool()),
	_forceFunctionSetProp(PropertyObj("", FunctionSet())),
    _forceFunctionSet((FunctionSet&)_forceFunctionSetProp.getValueObj()),
	_pointFunctionSetProp(PropertyObj("", FunctionSet())),
    _pointFunctionSet((FunctionSet&)_pointFunctionSetProp.getValueObj()),
	_torqueFunctionSetProp(PropertyObj("", FunctionSet())),
    _torqueFunctionSet((FunctionSet&)_torqueFunctionSetProp.getValueObj()),
	_forceX(force._forceX == NULL ? NULL : static_cast<Function*>(force._forceX->copy())),
	_forceY(force._forceY == NULL ? NULL : static_cast<Function*>(force._forceY->copy())),
	_forceZ(force._forceZ == NULL ? NULL : static_cast<Function*>(force._forceZ->copy())),
	_pointX(force._pointX == NULL ? NULL : static_cast<Function*>(force._pointX->copy())),
	_pointY(force._pointY == NULL ? NULL : static_cast<Function*>(force._pointY->copy())),
	_pointZ(force._pointZ == NULL ? NULL : static_cast<Function*>(force._pointZ->copy())),
	_torqueX(force._torqueX == NULL ? NULL : static_cast<Function*>(force._torqueX->copy())),
	_torqueY(force._torqueY == NULL ? NULL : static_cast<Function*>(force._torqueY->copy())),
	_torqueZ(force._torqueZ == NULL ? NULL : static_cast<Function*>(force._torqueZ->copy()))
{
	setNull();
	setupProperties();
	_body=force._body;
	_bodyName = force._bodyName;
	_forceFunctionSet = force._forceFunctionSet;
	_pointFunctionSet = force._pointFunctionSet;
	_torqueFunctionSet = force._torqueFunctionSet;
	_pointIsGlobal = force._pointIsGlobal;
	_forceIsGlobal = force._forceIsGlobal;
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
	CustomForce::operator=(aForce);
	copyData(aForce);
	return(*this);

}

void PrescribedForce::setForceFunctions(Function* forceX, Function* forceY, Function* forceZ)
{
	_forceX = forceX;
	_forceY = forceY;
	_forceZ = forceZ;
	_forceFunctionSet.setSize(0);
	_forceFunctionSet.append(forceX);
	_forceFunctionSet.append(forceY);
	_forceFunctionSet.append(forceZ);
}


void PrescribedForce::setPointFunctions(Function* pointX, Function* pointY, Function* pointZ)
{
	_pointX = pointX;
	_pointY = pointY;
	_pointZ = pointZ;
	_pointFunctionSet.setSize(0);
	_pointFunctionSet.append(pointX);
	_pointFunctionSet.append(pointY);
	_pointFunctionSet.append(pointZ);
}

void PrescribedForce::setTorqueFunctions(Function* torqueX, Function* torqueY, Function* torqueZ)
{
	_torqueX = torqueX;
	_torqueY = torqueY;
	_torqueZ = torqueZ;
	_torqueFunctionSet.setSize(0);
	_torqueFunctionSet.append(torqueX);
	_torqueFunctionSet.append(torqueY);
	_torqueFunctionSet.append(torqueZ);

}

void PrescribedForce::setFunctionsFromFile(std::string anExternalLoadsFileName)
{
	// LOAD COP, FORCE, AND TORQUE
	Storage kineticsStore(anExternalLoadsFileName);
	int forceSize = kineticsStore.getSize();
	if(forceSize<=0) return;

	double *t=0;

	// Expected column labels for the file

	std::string labels[9] = {"pointX", "pointY", "pointZ", "forceX", "forceY", "forceZ", "torqueX", "torqueY", "torqueZ"};

	kineticsStore.getTimeColumn(t);
	double *column=0;

	for(int i=0;i<9;i++)
	{
		kineticsStore.getDataColumn(labels[i], column);
		double value1 = column[7], value2 = column[8];
		NaturalCubicSpline currentSpline(forceSize, t, column, labels[i]);
		const double* values = currentSpline.getYValues();
		splines.append(currentSpline);
	}

	setPointFunctions(&splines.get(0), &splines.get(1), &splines.get(2));
	setForceFunctions(&splines.get(3), &splines.get(4), &splines.get(5));
	setTorqueFunctions(&splines.get(6), &splines.get(7), &splines.get(8));
	
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
	if (_forceX != NULL) {
		Vec3 force(_forceX?_forceX->calcValue(timeAsVector):0.0, 
			_forceY?_forceY->calcValue(timeAsVector):0.0, 
			_forceZ?_forceZ->calcValue(timeAsVector):0.0);
		if (!_forceIsGlobal)
			engine.transform(state, *_body, force, engine.getGroundBody(), force);
		if (_pointX == NULL) {
			applyForce(state, *_body, force, bodyForces);
 
	    }else {
			Vec3 point(_pointX?_pointX->calcValue(timeAsVector):0.0, 
				_pointY?_pointY->calcValue(timeAsVector):0.0, 
				_pointZ?_pointZ->calcValue(timeAsVector):0.0);
			if (_pointIsGlobal)
				engine.transformPosition(state, engine.getGroundBody(), point, *_body, point);
			applyForceToPoint(state, *_body, point, force, bodyForces);
		}
	}
	if (_torqueX != NULL) {
		Vec3 torque(_torqueX?_torqueX->calcValue(timeAsVector):0.0, 
			_torqueY?_torqueY->calcValue(timeAsVector):0.0, 
			_torqueZ?_torqueZ->calcValue(timeAsVector):0.0);
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
	Vec3 force(_forceX?_forceX->calcValue(timeAsVector):0.0, 
		_forceY?_forceY->calcValue(timeAsVector):0.0, 
		_forceZ?_forceZ->calcValue(timeAsVector):0.0);
	return force;
}

Vec3 PrescribedForce::getPointAtTime(double aTime) const
{
	SimTK::Vector timeAsVector(1, aTime);
	Vec3 point(_pointX?_pointX->calcValue(timeAsVector):0.0, 
		_pointY?_pointY->calcValue(timeAsVector):0.0, 
		_pointZ?_pointZ->calcValue(timeAsVector):0.0);
	return point;
}

Vec3 PrescribedForce::getTorqueAtTime(double aTime) const
{
	SimTK::Vector timeAsVector(1, aTime);
	Vec3 torque(_torqueX?_torqueX->calcValue(timeAsVector):0.0, 
		_torqueY?_torqueY->calcValue(timeAsVector):0.0, 
		_torqueZ?_torqueZ->calcValue(timeAsVector):0.0);
	return torque;
}


//-----------------------------------------------------------------------------
// Reporting
//-----------------------------------------------------------------------------

OpenSim::Array<std::string> PrescribedForce::getRecordLabels() const {
	OpenSim::Array<std::string> labels("");

	// Compute flags to find out if a point is specified or not and if a torque or force or both are specified
	bool appliesForce = (_forceX!=NULL) || (_forceY!=NULL) || (_forceZ!=NULL);
	bool pointSpecified = (_pointX!=NULL) || (_pointY!=NULL) || (_pointZ!=NULL);
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
	if (_torqueX!=NULL){
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

	if (_forceX != NULL) {
		Vec3 force(_forceX?_forceX->calcValue(timeAsVector):0.0, 
			_forceY?_forceY->calcValue(timeAsVector):0.0, 
			_forceZ?_forceZ->calcValue(timeAsVector):0.0);
		if (!_forceIsGlobal)
			engine.transform(state, *_body, force, engine.getGroundBody(), force);
		if (_pointX == NULL) {
			//applyForce(*_body, force);
			for (int i=0; i<3; i++) values.append(force[i]);
	    }else {
			Vec3 point(_pointX?_pointX->calcValue(timeAsVector):0.0, 
				_pointY?_pointY->calcValue(timeAsVector):0.0, 
				_pointZ?_pointZ->calcValue(timeAsVector):0.0);
			if (_pointIsGlobal)
				engine.transformPosition(state, engine.getGroundBody(), point, *_body, point);
			//applyForceToPoint(*_body, point, force);
			for (int i=0; i<3; i++) values.append(force[i]);
			for (int i=0; i<3; i++) values.append(point[i]);
		}
	}
	if (_torqueX != NULL) {
		Vec3 torque(_torqueX?_torqueX->calcValue(timeAsVector):0.0, 
			_torqueY?_torqueY->calcValue(timeAsVector):0.0, 
			_torqueZ?_torqueZ->calcValue(timeAsVector):0.0);
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
}

void PrescribedForce::setup(Model& model)
{
	setModel(model);

	// hook up body pointer to name
	if (_model)
		_body = &_model->updBodySet().get(_bodyName);
}

void PrescribedForce::setupFromXML()
{
	CustomForce::setupFromXML();
}
