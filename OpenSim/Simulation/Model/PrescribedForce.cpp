// PrescribedForce.cpp
// Author: Peter Eastman
/*
* Copyright (c) 2008-12, Stanford University. All rights reserved. 
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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Common/FunctionSet.h>
#include "PrescribedForce.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace OpenSim;
using SimTK::Vec3;
using namespace std;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
// default destructor, copy constructor, copy assignment
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PrescribedForce::PrescribedForce(Body* body)
{
	setNull();
	constructProperties();

	_body = body;
	if (_body)
		setBodyName(_body->getName());
}

//_____________________________________________________________________________
/**
 * Constructor from XML file
 */
PrescribedForce::PrescribedForce(SimTK::Xml::Element& aNode) : Super(aNode)
{
	setNull();
	constructProperties();
	updateFromXMLNode(aNode);
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
	Super::updateFromXMLNode(aNode, versionNumber);

	const FunctionSet& forceFunctions  = getForceFunctions();
	const FunctionSet& pointFunctions  = getPointFunctions();
	const FunctionSet& torqueFunctions = getTorqueFunctions();

	//Specify all or none of the components
	if(forceFunctions.getSize() != 3 && forceFunctions.getSize() != 0)
	{
		throw Exception("PrescribedForce:: three components of the force must be specified.");
	}

	if(pointFunctions.getSize() != 3 && pointFunctions.getSize() != 0)
	{
		throw Exception("PrescribedForce:: three components of the point must be specified.");
	}

	if(torqueFunctions.getSize() != 3 && torqueFunctions.getSize() != 0)
	{
		throw Exception("PrescribedForce:: three components of the torque must be specified.");
	}
}	


/*
 * Construct and initialize properties.
 */
void PrescribedForce::constructProperties()
{
	constructProperty_body();
	constructProperty_pointIsGlobal(false);
	constructProperty_forceIsGlobal(true);
    constructProperty_forceFunctions(FunctionSet());
    constructProperty_pointFunctions(FunctionSet());
    constructProperty_torqueFunctions(FunctionSet());
}

void PrescribedForce::setForceFunctions(Function* forceX, Function* forceY, Function* forceZ)
{
	FunctionSet& forceFunctions = updForceFunctions();

	forceFunctions.setSize(0);
	forceFunctions.append(*forceX);
	forceFunctions.append(*forceY);
	forceFunctions.append(*forceZ);
}


void PrescribedForce::setPointFunctions(Function* pointX, Function* pointY, Function* pointZ)
{
	FunctionSet& pointFunctions = updPointFunctions();

	pointFunctions.setSize(0);
	pointFunctions.append(*pointX);
	pointFunctions.append(*pointY);
	pointFunctions.append(*pointZ);
}

void PrescribedForce::setTorqueFunctions(Function* torqueX, Function* torqueY, Function* torqueZ)
{
	FunctionSet& torqueFunctions = updTorqueFunctions();

	torqueFunctions.setSize(0);
	torqueFunctions.append(*torqueX);
	torqueFunctions.append(*torqueY);
	torqueFunctions.append(*torqueZ);

}

void PrescribedForce::setTorqueFunctionNames
   (const OpenSim::Array<std::string>& aFunctionNames, 
	const Storage& kineticsStore)  
{
	FunctionSet& torqueFunctions = updTorqueFunctions();

	int forceSize = kineticsStore.getSize();
	if(forceSize<=0) return;
	double *t=0;
	// Expected column labels for the file
	kineticsStore.getTimeColumn(t);
	double *column=0;
	SimmSpline** tSpline = new SimmSpline*[3];
	for(int i=0;i<aFunctionNames.getSize();i++)
	{
		kineticsStore.getDataColumn(aFunctionNames[i], column);
		tSpline[i]= new SimmSpline((forceSize>10?10:forceSize), t, column, aFunctionNames[i]);
	}
	setTorqueFunctions(tSpline[0], tSpline[1], tSpline[2]);
	for (int i=0; i<aFunctionNames.getSize();i++)
		torqueFunctions[i].setName(aFunctionNames.get(i));
}
void PrescribedForce::setForceFunctionNames
   (const OpenSim::Array<std::string>& aFunctionNames, 
	const Storage& kineticsStore)  
{
	FunctionSet& forceFunctions = updForceFunctions();

	int forceSize = kineticsStore.getSize();
	if(forceSize<=0) return;
	double *t=0;
	// Expected column labels for the file
	kineticsStore.getTimeColumn(t);
	double *column=0;
	SimmSpline** tSpline = new SimmSpline*[3];
	for(int i=0;i<aFunctionNames.getSize();i++)
	{
		kineticsStore.getDataColumn(aFunctionNames[i], column);
		tSpline[i]= new SimmSpline((forceSize>10?10:forceSize), t, column, aFunctionNames[i]);
	}
	setForceFunctions(tSpline[0], tSpline[1], tSpline[2]);
	for (int i=0; i<aFunctionNames.getSize();i++)
		forceFunctions[i].setName(aFunctionNames.get(i));
}
void PrescribedForce::setPointFunctionNames
   (const OpenSim::Array<std::string>& aFunctionNames, 
	const Storage& kineticsStore)  
{
	FunctionSet& pointFunctions = updPointFunctions();

	int forceSize = kineticsStore.getSize();
	if(forceSize<=0) return;
	double *t=0;
	// Expected column labels for the file
	kineticsStore.getTimeColumn(t);
	double *column=0;
	SimmSpline** tSpline = new SimmSpline*[3];
	for(int i=0;i<aFunctionNames.getSize();i++)
	{
		kineticsStore.getDataColumn(aFunctionNames[i], column);
		tSpline[i]= new SimmSpline((forceSize>10?10:forceSize), 
                                           t, column, aFunctionNames[i]);
	}
	setPointFunctions(tSpline[0], tSpline[1], tSpline[2]);
	for (int i=0; i<aFunctionNames.getSize();i++)
		pointFunctions[i].setName(aFunctionNames.get(i));
}


//-----------------------------------------------------------------------------
// ABSTRACT METHODS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________

void PrescribedForce::computeForce(const SimTK::State& state, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const
{
	const bool pointIsGlobal = getProperty_pointIsGlobal();
	const bool forceIsGlobal = getProperty_forceIsGlobal();
	const FunctionSet& forceFunctions = getForceFunctions();
	const FunctionSet& pointFunctions = getPointFunctions();
	const FunctionSet& torqueFunctions = getTorqueFunctions();

	double time = state.getTime();
	const SimbodyEngine& engine = getModel().getSimbodyEngine();
	SimTK::Vector  timeAsVector(1, time);

    const bool hasForceFunctions  = forceFunctions.getSize()==3;
    const bool hasPointFunctions  = pointFunctions.getSize()==3;
    const bool hasTorqueFunctions = torqueFunctions.getSize()==3;

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
	const FunctionSet& forceFunctions = getForceFunctions();

    if (forceFunctions.getSize() != 3)
        return Vec3(0);

	const SimTK::Vector timeAsVector(1, aTime);
	const Vec3 force(forceFunctions[0].calcValue(timeAsVector), 
		             forceFunctions[1].calcValue(timeAsVector), 
		             forceFunctions[2].calcValue(timeAsVector));
	return force;
}

Vec3 PrescribedForce::getPointAtTime(double aTime) const
{
	const FunctionSet& pointFunctions = getPointFunctions();

    if (pointFunctions.getSize() != 3)
        return Vec3(0);

	const SimTK::Vector timeAsVector(1, aTime);
	const Vec3 point(pointFunctions[0].calcValue(timeAsVector), 
		             pointFunctions[1].calcValue(timeAsVector), 
		             pointFunctions[2].calcValue(timeAsVector));
	return point;
}

Vec3 PrescribedForce::getTorqueAtTime(double aTime) const
{
	const FunctionSet& torqueFunctions = getTorqueFunctions();

    if (torqueFunctions.getSize() != 3)
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

	const bool forceIsGlobal = getProperty_forceIsGlobal();

	const FunctionSet& forceFunctions = getForceFunctions();
	const FunctionSet& pointFunctions = getPointFunctions();
	const FunctionSet& torqueFunctions = getTorqueFunctions();

    const bool appliesForce   = forceFunctions.getSize()==3;
    const bool pointSpecified = pointFunctions.getSize()==3;
    const bool appliesTorque  = torqueFunctions.getSize()==3;

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

	const bool pointIsGlobal = getProperty_pointIsGlobal();
	const bool forceIsGlobal = getProperty_forceIsGlobal();

	const FunctionSet& forceFunctions = getForceFunctions();
	const FunctionSet& pointFunctions = getPointFunctions();
	const FunctionSet& torqueFunctions = getTorqueFunctions();

    const bool appliesForce   = forceFunctions.getSize()==3;
    const bool pointSpecified = pointFunctions.getSize()==3;
    const bool appliesTorque  = torqueFunctions.getSize()==3;

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
    _body = NULL;
}

void PrescribedForce::setup(Model& model)
{
	Super::setup(model);

	// hook up body pointer to name
	if (_model)
		_body = &_model->updBodySet().get(getBodyName());
}
