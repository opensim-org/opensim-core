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
PrescribedForce::PrescribedForce(Body* body) : Force()
{
	setNull();
	setupProperties();
	//updateFromXMLNode();

	_body = body;
	if (_body)
		setPropertyValue("body", _body->getName());
}

//_____________________________________________________________________________
/**
 * Constructor from XML file
 */
PrescribedForce::PrescribedForce(SimTK::Xml::Element& aNode) :
	Force(aNode)
{
	setNull();
	setupProperties();
	updateFromXMLNode(aNode);

	_body = NULL;
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
PrescribedForce::PrescribedForce(const PrescribedForce& force) :
	Force(force)
{
	setNull();
	setupProperties();
	//updateFromXMLNode();
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
void PrescribedForce::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
	// Base class
	Force::updateFromXMLNode(aNode, versionNumber);

	const FunctionSet &forceFunctionSet = getPropertyValue<FunctionSet>("forceFunctions");
	const FunctionSet &pointFunctionSet = getPropertyValue<FunctionSet>("pointFunctions");
	const FunctionSet &torqueFunctionSet = getPropertyValue<FunctionSet>("torqueFunctions");

	//Specify all or none of the components
	if(forceFunctionSet.getSize() != 3&& forceFunctionSet.getSize() != 0)
	{
		throw Exception("PrescribedForce:: three components of the force must be specified.");
	}

	if(pointFunctionSet.getSize() != 3 && pointFunctionSet.getSize() != 0)
	{
		throw Exception("PrescribedForce:: three components of the point must be specified.");
	}

	if(torqueFunctionSet.getSize() != 3 && torqueFunctionSet.getSize() != 0)
	{
		throw Exception("PrescribedForce:: three components of the torque must be specified.");
	}
}	


/**
 * Connect properties to local pointers.
 */
void PrescribedForce::setupProperties()
{
	addProperty<string>("body",
		"Name of the body the force is applied to.",
		"");
	addProperty<bool>("pointIsGlobal",
		"Flag indicating whether the point (specified in pointFunctions) is in global frame",
		false);
	addProperty<bool>("forceIsGlobal",
		"Flag indicating whether the quantities (specified in force/torqueFunctions) is in global frame",
		true);
	addProperty<FunctionSet>("forceFunctions",
		"Three functions describing the force to be applied.",
		FunctionSet());
	Property2<FunctionSet> &forceFunctionSetProp = updProperty<FunctionSet>("forceFunctions");
	forceFunctionSetProp.setMatchName(true);
	forceFunctionSetProp.setAllowableArraySize(3);
	FunctionSet &forceFunctionSet = updPropertyValue<FunctionSet>("forceFunctions");
	forceFunctionSet.setName("forceFunctions");
	forceFunctionSet.setMemoryOwner(false);
	addProperty<FunctionSet>("pointFunctions",
		"Three functions describing the location at which the force is applied.",
		FunctionSet());
	Property2<FunctionSet> &pointFunctionSetProp = updProperty<FunctionSet>("pointFunctions");
	pointFunctionSetProp.setMatchName(true);
	pointFunctionSetProp.setAllowableArraySize(0, 3);
	FunctionSet &pointFunctionSet = updPropertyValue<FunctionSet>("pointFunctions");
	pointFunctionSet.setName("pointFunctions");
	pointFunctionSet.setMemoryOwner(false);
	addProperty<FunctionSet>("torqueFunctions",
		"Three functions describing the torque the PrescribedForce applies.",
		FunctionSet());
	Property2<FunctionSet> &torqueFunctionSetProp = updProperty<FunctionSet>("torqueFunctions");
	torqueFunctionSetProp.setMatchName(true);
	torqueFunctionSetProp.setAllowableArraySize(0, 3);
	FunctionSet &torqueFunctionSet = updPropertyValue<FunctionSet>("torqueFunctions");
	torqueFunctionSet.setName("torqueFunctions");
	torqueFunctionSet.setMemoryOwner(false);
}

PrescribedForce& PrescribedForce::operator=(const PrescribedForce &aForce)
{
	Force::operator=(aForce);
	copyData(aForce);
	return(*this);
}

void PrescribedForce::setForceFunctions(Function* forceX, Function* forceY, Function* forceZ)
{
	FunctionSet &forceFunctionSet = updPropertyValue<FunctionSet>("forceFunctions");

	forceFunctionSet.setSize(0);
	forceFunctionSet.append(forceX);
	forceFunctionSet.append(forceY);
	forceFunctionSet.append(forceZ);
}


void PrescribedForce::setPointFunctions(Function* pointX, Function* pointY, Function* pointZ)
{
	FunctionSet &pointFunctionSet = updPropertyValue<FunctionSet>("pointFunctions");

	pointFunctionSet.setSize(0);
	pointFunctionSet.append(pointX);
	pointFunctionSet.append(pointY);
	pointFunctionSet.append(pointZ);
}

void PrescribedForce::setTorqueFunctions(Function* torqueX, Function* torqueY, Function* torqueZ)
{
	FunctionSet &torqueFunctionSet = updPropertyValue<FunctionSet>("torqueFunctions");

	torqueFunctionSet.setSize(0);
	torqueFunctionSet.append(torqueX);
	torqueFunctionSet.append(torqueY);
	torqueFunctionSet.append(torqueZ);

}

void PrescribedForce::setTorqueFunctionNames(const OpenSim::Array<std::string>& aFunctionNames, 
											 const Storage& kineticsStore)  
{
	FunctionSet &torqueFunctionSet = updPropertyValue<FunctionSet>("torqueFunctions");

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
		torqueFunctionSet.get(i).setName(aFunctionNames.get(i));
}
void PrescribedForce::setForceFunctionNames(const OpenSim::Array<std::string>& aFunctionNames, 
											 const Storage& kineticsStore)  
{
	FunctionSet &forceFunctionSet = updPropertyValue<FunctionSet>("forceFunctions");

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
		forceFunctionSet.get(i).setName(aFunctionNames.get(i));
}
void PrescribedForce::setPointFunctionNames(const OpenSim::Array<std::string>& aFunctionNames, 
											 const Storage& kineticsStore)  
{
	FunctionSet &pointFunctionSet = updPropertyValue<FunctionSet>("pointFunctions");

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
		pointFunctionSet.get(i).setName(aFunctionNames.get(i));
}


bool PrescribedForce::getForceIsInGlobalFrame() const
{
	return getPropertyValue<bool>("forceIsGlobal");
}
void PrescribedForce::setForceIsInGlobalFrame(bool isGlobal)
{
	setPropertyValue("forceIsGlobal", isGlobal);
}
bool PrescribedForce::getPointIsInGlobalFrame() const
{
	return getPropertyValue<bool>("pointIsGlobal");
}
void PrescribedForce::setPointIsInGlobalFrame(bool isGlobal)
{
	setPropertyValue("pointIsGlobal", isGlobal);
}

//-----------------------------------------------------------------------------
// ABSTRACT METHODS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________

void PrescribedForce::computeForce(const SimTK::State& state, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const
{
	const bool &pointIsGlobal = getPropertyValue<bool>("pointIsGlobal");
	const bool &forceIsGlobal = getPropertyValue<bool>("forceIsGlobal");
	const FunctionSet &forceFunctionSet = getPropertyValue<FunctionSet>("forceFunctions");
	const FunctionSet &pointFunctionSet = getPropertyValue<FunctionSet>("pointFunctions");
	const FunctionSet &torqueFunctionSet = getPropertyValue<FunctionSet>("torqueFunctions");

	double time = state.getTime();
	const SimbodyEngine& engine = getModel().getSimbodyEngine();
	SimTK::Vector  timeAsVector(1, time);

	assert(_body!=0);
	Function* forceX=NULL;
	Function* forceY=NULL;
	Function* forceZ=NULL;
	if (forceFunctionSet.getSize()==3){
		forceX=&forceFunctionSet[0];	forceY=&forceFunctionSet[1];	forceZ=&forceFunctionSet[2];
	}
	if (forceX != NULL) {
		Function* pointX=NULL;
		Function* pointY=NULL;
		Function* pointZ=NULL;
		if (pointFunctionSet.getSize()==3){
			pointX=&pointFunctionSet[0];	pointY=&pointFunctionSet[1];	pointZ=&pointFunctionSet[2];
		}
		Vec3 force(forceX?forceX->calcValue(timeAsVector):0.0, 
			forceY?forceY->calcValue(timeAsVector):0.0, 
			forceZ?forceZ->calcValue(timeAsVector):0.0);
		if (!forceIsGlobal)
			engine.transform(state, *_body, force, engine.getGroundBody(), force);
		if (pointX == NULL) {
			applyForce(state, *_body, force, bodyForces);
 
	    }else {
			Vec3 point(pointX?pointX->calcValue(timeAsVector):0.0, 
				pointY?pointY->calcValue(timeAsVector):0.0, 
				pointZ?pointZ->calcValue(timeAsVector):0.0);
			if (pointIsGlobal)
				engine.transformPosition(state, engine.getGroundBody(), point, *_body, point);
			applyForceToPoint(state, *_body, point, force, bodyForces);
		}
	}
	Function* torqueX=NULL;
	Function* torqueY=NULL;
	Function* torqueZ=NULL;
	if (torqueFunctionSet.getSize()==3){
		torqueX=&torqueFunctionSet[0];	torqueY=&torqueFunctionSet[1];	torqueZ=&torqueFunctionSet[2];
	}
	if (torqueX != NULL) {
		Vec3 torque(torqueX?torqueX->calcValue(timeAsVector):0.0, 
			torqueY?torqueY->calcValue(timeAsVector):0.0, 
			torqueZ?torqueZ->calcValue(timeAsVector):0.0);
		if (!forceIsGlobal)
			engine.transform(state, *_body, torque, engine.getGroundBody(), torque);
		applyTorque(state, *_body, torque, bodyForces);
	}
}

/**
 * Conevenince methods to access prescribed force functions
 */
Vec3 PrescribedForce::getForceAtTime(double aTime) const	
{
	const FunctionSet &forceFunctionSet = getPropertyValue<FunctionSet>("forceFunctions");

	SimTK::Vector timeAsVector(1, aTime);
	Function* forceX=NULL;
	Function* forceY=NULL;
	Function* forceZ=NULL;
	if (forceFunctionSet.getSize()==3){
		forceX=&forceFunctionSet[0];	forceY=&forceFunctionSet[1];	forceZ=&forceFunctionSet[2];
	}
	Vec3 force(forceX?forceX->calcValue(timeAsVector):0.0, 
		forceY?forceY->calcValue(timeAsVector):0.0, 
		forceZ?forceZ->calcValue(timeAsVector):0.0);
	return force;
}

Vec3 PrescribedForce::getPointAtTime(double aTime) const
{
	const FunctionSet &pointFunctionSet = getPropertyValue<FunctionSet>("pointFunctions");

	SimTK::Vector timeAsVector(1, aTime);
	Function* pointX=NULL;
	Function* pointY=NULL;
	Function* pointZ=NULL;
	if (pointFunctionSet.getSize()==3){
		pointX=&pointFunctionSet[0];	pointY=&pointFunctionSet[1];	pointZ=&pointFunctionSet[2];
	}
	Vec3 point(pointX?pointX->calcValue(timeAsVector):0.0, 
		pointY?pointY->calcValue(timeAsVector):0.0, 
		pointZ?pointZ->calcValue(timeAsVector):0.0);
	return point;
}

Vec3 PrescribedForce::getTorqueAtTime(double aTime) const
{
	const FunctionSet &torqueFunctionSet = getPropertyValue<FunctionSet>("torqueFunctions");

	SimTK::Vector timeAsVector(1, aTime);
	Function* torqueX=NULL;
	Function* torqueY=NULL;
	Function* torqueZ=NULL;
	if (torqueFunctionSet.getSize()==3){
		torqueX=&torqueFunctionSet[0];	torqueY=&torqueFunctionSet[1];	torqueZ=&torqueFunctionSet[2];
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

	const bool &forceIsGlobal = getPropertyValue<bool>("forceIsGlobal");
	const FunctionSet &forceFunctionSet = getPropertyValue<FunctionSet>("forceFunctions");
	const FunctionSet &pointFunctionSet = getPropertyValue<FunctionSet>("pointFunctions");
	const FunctionSet &torqueFunctionSet = getPropertyValue<FunctionSet>("torqueFunctions");

	// Compute flags to find out if a point is specified or not and if a torque or force or both are specified
	Function* forceX=NULL;
	Function* forceY=NULL;
	Function* forceZ=NULL;
	if (forceFunctionSet.getSize()==3){
		forceX=&forceFunctionSet[0];	forceY=&forceFunctionSet[1];	forceZ=&forceFunctionSet[2];
	}
	Function* pointX=NULL;
	Function* pointY=NULL;
	Function* pointZ=NULL;
	if (pointFunctionSet.getSize()==3){
		pointX=&pointFunctionSet[0];	pointY=&pointFunctionSet[1];	pointZ=&pointFunctionSet[2];
	}
	Function* torqueX=NULL;
	Function* torqueY=NULL;
	Function* torqueZ=NULL;
	if (torqueFunctionSet.getSize()==3){
		torqueX=&torqueFunctionSet[0];	torqueY=&torqueFunctionSet[1];	torqueZ=&torqueFunctionSet[2];
	}
	bool appliesForce = (forceX!=NULL) || (forceY!=NULL) || (forceZ!=NULL);
	bool pointSpecified = (pointX!=NULL) || (pointY!=NULL) || (pointZ!=NULL);
	// Instead of trying to figure out which values are not specified we'll write NaNs 
	std::string BodyToReport = (forceIsGlobal?"ground":_body->getName());
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

	const bool &pointIsGlobal = getPropertyValue<bool>("pointIsGlobal");
	const bool &forceIsGlobal = getPropertyValue<bool>("forceIsGlobal");
	const FunctionSet &forceFunctionSet = getPropertyValue<FunctionSet>("forceFunctions");
	const FunctionSet &pointFunctionSet = getPropertyValue<FunctionSet>("pointFunctions");
	const FunctionSet &torqueFunctionSet = getPropertyValue<FunctionSet>("torqueFunctions");

	// This is bad as it duplicates the code in computeForce we'll cleanup after it works!
	double time = state.getTime();
	const SimbodyEngine& engine = getModel().getSimbodyEngine();
	SimTK::Vector  timeAsVector(1, time);

	Function* forceX=NULL;
	Function* forceY=NULL;
	Function* forceZ=NULL;
	if (forceFunctionSet.getSize()==3){
		forceX=&forceFunctionSet[0];	forceY=&forceFunctionSet[1];	forceZ=&forceFunctionSet[2];
	}
	Function* pointX=NULL;
	Function* pointY=NULL;
	Function* pointZ=NULL;
	if (pointFunctionSet.getSize()==3){
		pointX=&pointFunctionSet[0];	pointY=&pointFunctionSet[1];	pointZ=&pointFunctionSet[2];
	}
	Function* torqueX=NULL;
	Function* torqueY=NULL;
	Function* torqueZ=NULL;
	if (torqueFunctionSet.getSize()==3){
		torqueX=&torqueFunctionSet[0];	torqueY=&torqueFunctionSet[1];	torqueZ=&torqueFunctionSet[2];
	}
	if (forceX != NULL) {
		Vec3 force(forceX?forceX->calcValue(timeAsVector):0.0, 
			forceY?forceY->calcValue(timeAsVector):0.0, 
			forceZ?forceZ->calcValue(timeAsVector):0.0);
		if (!forceIsGlobal)
			engine.transform(state, *_body, force, engine.getGroundBody(), force);
		if (pointX == NULL) {
			//applyForce(*_body, force);
			for (int i=0; i<3; i++) values.append(force[i]);
	    }else {
			Vec3 point(pointX?pointX->calcValue(timeAsVector):0.0, 
				pointY?pointY->calcValue(timeAsVector):0.0, 
				pointZ?pointZ->calcValue(timeAsVector):0.0);
			if (pointIsGlobal)
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
		if (!forceIsGlobal)
			engine.transform(state, *_body, torque, engine.getGroundBody(), torque);
		for (int i=0; i<3; i++) values.append(torque[i]);
		//applyTorque(*_body, torque);
	}
	return values;
};

void PrescribedForce::setNull()
{
	setType("PrescribedForce");
}

void PrescribedForce::copyData(const PrescribedForce& orig)
{
	setPropertyValue("forceIsGlobal", orig.getPropertyValue<bool>("forceIsGlobal"));
	setPropertyValue("pointIsGlobal", orig.getPropertyValue<bool>("pointIsGlobal"));

	setPropertyValue("forceFunctions", orig.getPropertyValue<FunctionSet>("forceFunctions"));
	setPropertyValue("pointFunctions", orig.getPropertyValue<FunctionSet>("pointFunctions"));
	setPropertyValue("torqueFunctions", orig.getPropertyValue<FunctionSet>("torqueFunctions"));

	setPropertyValue("body", orig.getPropertyValue<string>("body"));
	_body = orig._body;
}

void PrescribedForce::setup(Model& model)
{
	Force::setup(model);

	// hook up body pointer to name
	if (_model)
		_body = &_model->updBodySet().get(getPropertyValue<string>("body"));
}
