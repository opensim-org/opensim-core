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
#include <OpenSim/Common/Function.h>

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

	const Property<Function>& forceFunctions = 
        getProperty<Function>("forceFunctions");
	const Property<Function>& pointFunctions = 
        getProperty<Function>("pointFunctions");
	const Property<Function>& torqueFunctions = 
        getProperty<Function>("torqueFunctions");

	//Specify all or none of the components
	if(forceFunctions.size() != 3 && forceFunctions.size() != 0)
	{
		throw Exception("PrescribedForce:: three components of the force must be specified.");
	}

	if(pointFunctions.size() != 3 && pointFunctions.size() != 0)
	{
		throw Exception("PrescribedForce:: three components of the point must be specified.");
	}

	if(torqueFunctions.size() != 3 && torqueFunctions.size() != 0)
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

	addListProperty<Function>("forceFunctions",
		"Three functions describing the force to be applied.")
        .setAllowableListSize(0, 3);

	addListProperty<Function>("pointFunctions",
		"Three functions describing the location at which the force is applied.")
        .setAllowableListSize(0, 3);

	addListProperty<Function>("torqueFunctions",
		"Three functions describing the torque the PrescribedForce applies.")
        .setAllowableListSize(0, 3);
}

PrescribedForce& PrescribedForce::operator=(const PrescribedForce &aForce)
{
	Force::operator=(aForce);
	copyData(aForce);
	return(*this);
}

void PrescribedForce::setForceFunctions(Function* forceX, Function* forceY, Function* forceZ)
{
	Property<Function>& forceFunctionProp = updProperty<Function>("forceFunctions");

	forceFunctionProp.clear();
	forceFunctionProp.appendValue(*forceX);
	forceFunctionProp.appendValue(*forceY);
	forceFunctionProp.appendValue(*forceZ);
}


void PrescribedForce::setPointFunctions(Function* pointX, Function* pointY, Function* pointZ)
{
	Property<Function>& pointFunctionProp = updProperty<Function>("pointFunctions");

	pointFunctionProp.clear();
	pointFunctionProp.appendValue(*pointX);
	pointFunctionProp.appendValue(*pointY);
	pointFunctionProp.appendValue(*pointZ);
}

void PrescribedForce::setTorqueFunctions(Function* torqueX, Function* torqueY, Function* torqueZ)
{
	Property<Function>& torqueFunctionProp = updProperty<Function>("torqueFunctions");

	torqueFunctionProp.clear();
	torqueFunctionProp.appendValue(*torqueX);
	torqueFunctionProp.appendValue(*torqueY);
	torqueFunctionProp.appendValue(*torqueZ);

}

void PrescribedForce::setTorqueFunctionNames(const OpenSim::Array<std::string>& aFunctionNames, 
											 const Storage& kineticsStore)  
{
	Property<Function>& torqueFunctionProp = updProperty<Function>("torqueFunctions");

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
		torqueFunctionProp[i].setName(aFunctionNames.get(i));
}
void PrescribedForce::setForceFunctionNames(const OpenSim::Array<std::string>& aFunctionNames, 
											 const Storage& kineticsStore)  
{
	Property<Function>& forceFunctionProp = updProperty<Function>("forceFunctions");

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
		forceFunctionProp[i].setName(aFunctionNames.get(i));
}
void PrescribedForce::setPointFunctionNames(const OpenSim::Array<std::string>& aFunctionNames, 
											 const Storage& kineticsStore)  
{
	Property<Function>& pointFunctionProp = updProperty<Function>("pointFunctions");

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
		pointFunctionProp[i].setName(aFunctionNames.get(i));
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
	const Property<Function>& forceFunctions = 
        getProperty<Function>("forceFunctions");
	const Property<Function>& pointFunctions = 
        getProperty<Function>("pointFunctions");
	const Property<Function>& torqueFunctions = 
        getProperty<Function>("torqueFunctions");

	double time = state.getTime();
	const SimbodyEngine& engine = getModel().getSimbodyEngine();
	SimTK::Vector  timeAsVector(1, time);

    const bool hasForceFunctions  = forceFunctions.size()==3;
    const bool hasPointFunctions  = pointFunctions.size()==3;
    const bool hasTorqueFunctions = torqueFunctions.size()==3;

	assert(_body!=0);
	if (hasForceFunctions) {
		Vec3 force(forceFunctions[0].calcValue(timeAsVector), 
			       forceFunctions[1].calcValue(timeAsVector), 
			       forceFunctions[2].calcValue(timeAsVector));
		if (!forceIsGlobal)
			engine.transform(state, *_body,                 force, 
                                    engine.getGroundBody(), force);
        Vec3 point(0); // Default is body origin.
		if (hasPointFunctions) {
            // Apply force to a specified point on the body.
			point = Vec3(pointFunctions[0].calcValue(timeAsVector), 
				         pointFunctions[1].calcValue(timeAsVector), 
				         pointFunctions[2].calcValue(timeAsVector));
			if (pointIsGlobal)
				engine.transformPosition(state, engine.getGroundBody(), point,
                                                *_body,                 point);
		}
		applyForceToPoint(state, *_body, point, force, bodyForces);
	}
	if (hasTorqueFunctions){
		Vec3 torque(torqueFunctions[0].calcValue(timeAsVector), 
			        torqueFunctions[1].calcValue(timeAsVector), 
			        torqueFunctions[2].calcValue(timeAsVector));
		if (!forceIsGlobal)
			engine.transform(state, *_body,                 torque, 
                                    engine.getGroundBody(), torque);
		applyTorque(state, *_body, torque, bodyForces);
	}
}

/**
 * Conevenince methods to access prescribed force functions
 */
Vec3 PrescribedForce::getForceAtTime(double aTime) const	
{
	const Property<Function>& forceFunctions = 
        getProperty<Function>("forceFunctions");

    if (forceFunctions.size() != 3)
        return Vec3(0);

	const SimTK::Vector timeAsVector(1, aTime);
	const Vec3 force(forceFunctions[0].calcValue(timeAsVector), 
		             forceFunctions[1].calcValue(timeAsVector), 
		             forceFunctions[2].calcValue(timeAsVector));
	return force;
}

Vec3 PrescribedForce::getPointAtTime(double aTime) const
{
	const Property<Function>& pointFunctions = 
        getProperty<Function>("pointFunctions");

    if (pointFunctions.size() != 3)
        return Vec3(0);

	const SimTK::Vector timeAsVector(1, aTime);
	const Vec3 point(pointFunctions[0].calcValue(timeAsVector), 
		             pointFunctions[1].calcValue(timeAsVector), 
		             pointFunctions[2].calcValue(timeAsVector));
	return point;
}

Vec3 PrescribedForce::getTorqueAtTime(double aTime) const
{
	const Property<Function>& torqueFunctions = 
        getProperty<Function>("torqueFunctions");

    if (torqueFunctions.size() != 3)
        return Vec3(0);

	const SimTK::Vector timeAsVector(1, aTime);
	const Vec3 torque(torqueFunctions[0].calcValue(timeAsVector), 
		              torqueFunctions[1].calcValue(timeAsVector), 
		              torqueFunctions[2].calcValue(timeAsVector));
	return torque;
}


//-----------------------------------------------------------------------------
// Reporting
//-----------------------------------------------------------------------------

OpenSim::Array<std::string> PrescribedForce::getRecordLabels() const {
	OpenSim::Array<std::string> labels("");

	const bool forceIsGlobal = getPropertyValue<bool>("forceIsGlobal");

	const Property<Function>& forceFunctions = 
        getProperty<Function>("forceFunctions");
	const Property<Function>& pointFunctions = 
        getProperty<Function>("pointFunctions");
	const Property<Function>& torqueFunctions = 
        getProperty<Function>("torqueFunctions");

    const bool appliesForce   = forceFunctions.size()==3;
    const bool pointSpecified = pointFunctions.size()==3;
    const bool appliesTorque  = torqueFunctions.size()==3;

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
	if (appliesTorque) {
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

	const bool pointIsGlobal = getPropertyValue<bool>("pointIsGlobal");
	const bool forceIsGlobal = getPropertyValue<bool>("forceIsGlobal");

	const Property<Function>& forceFunctions = 
        getProperty<Function>("forceFunctions");
	const Property<Function>& pointFunctions = 
        getProperty<Function>("pointFunctions");
	const Property<Function>& torqueFunctions = 
        getProperty<Function>("torqueFunctions");

    const bool appliesForce   = forceFunctions.size()==3;
    const bool pointSpecified = pointFunctions.size()==3;
    const bool appliesTorque  = torqueFunctions.size()==3;

	// This is bad as it duplicates the code in computeForce we'll cleanup after it works!
	const double time = state.getTime();
	const SimbodyEngine& engine = getModel().getSimbodyEngine();
	const SimTK::Vector timeAsVector(1, time);

	if (appliesForce) {
	    Vec3 force(forceFunctions[0].calcValue(timeAsVector), 
		           forceFunctions[1].calcValue(timeAsVector), 
		           forceFunctions[2].calcValue(timeAsVector));
		if (!forceIsGlobal)
			engine.transform(state, *_body, force, 
                             engine.getGroundBody(), force);
		if (!pointSpecified) {
			//applyForce(*_body, force);
			for (int i=0; i<3; i++) values.append(force[i]);
	    } else {
	        Vec3 point(pointFunctions[0].calcValue(timeAsVector), 
		               pointFunctions[1].calcValue(timeAsVector), 
		               pointFunctions[2].calcValue(timeAsVector));
			if (pointIsGlobal)
				engine.transformPosition(state, engine.getGroundBody(), point, 
                                         *_body, point);
			//applyForceToPoint(*_body, point, force);
			for (int i=0; i<3; i++) values.append(force[i]);
			for (int i=0; i<3; i++) values.append(point[i]);
		}
	}
	if (appliesTorque) {
	    Vec3 torque(torqueFunctions[0].calcValue(timeAsVector), 
		            torqueFunctions[1].calcValue(timeAsVector), 
		            torqueFunctions[2].calcValue(timeAsVector));
		if (!forceIsGlobal)
			engine.transform(state, *_body, torque, 
                             engine.getGroundBody(), torque);
		for (int i=0; i<3; i++) values.append(torque[i]);
		//applyTorque(*_body, torque);
	}
	return values;
};

void PrescribedForce::setNull()
{
}

void PrescribedForce::copyData(const PrescribedForce& orig)
{
	setPropertyValue("forceIsGlobal", orig.getProperty<bool>("forceIsGlobal"));
	setPropertyValue("pointIsGlobal", orig.getProperty<bool>("pointIsGlobal"));

	setPropertyValue("forceFunctions", orig.getProperty<Function>("forceFunctions"));
	setPropertyValue("pointFunctions", orig.getProperty<Function>("pointFunctions"));
	setPropertyValue("torqueFunctions", orig.getProperty<Function>("torqueFunctions"));

	setPropertyValue("body", orig.getProperty<string>("body"));
	_body = orig._body;
}

void PrescribedForce::setup(Model& model)
{
	Force::setup(model);

	// hook up body pointer to name
	if (_model)
		_body = &_model->updBodySet().get(getPropertyValue<string>("body"));
}
