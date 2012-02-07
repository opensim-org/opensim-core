// ExternalForce.cpp
// Author: Ajay Seth
/*
* Copyright (c) 2011, Stanford University. All rights reserved. 
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
#include "ExternalForce.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Common/GCVSpline.h>

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
ExternalForce::~ExternalForce()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ExternalForce::ExternalForce() : Force()
{
	setNull();
	setupProperties();
	//updateFromXMLNode();
}

/**
 * Convenience Constructor of an ExternalForce. 
 * 
 */
ExternalForce::ExternalForce(const Storage &dataSource, string forceIdentifier, string pointIdentifier, string torqueIdentifier,
		string appliedToBodyName, string forceExpressedInBodyName, string pointExpressedInBodyName) : Force()
{
	setNull();
	setupProperties();
	_dataSource = &dataSource;

	setPropertyValue("applied_to_body", appliedToBodyName);
	setPropertyValue("force_expressed_in_body", forceExpressedInBodyName);
	setPropertyValue("point_expressed_in_body", pointExpressedInBodyName);
	setPropertyValue("data_source_name", dataSource.getName());
	setPropertyValue("force_identifier", forceIdentifier);
	setPropertyValue("point_identifier", pointIdentifier);
	setPropertyValue("torque_identifier", torqueIdentifier);
}


//_____________________________________________________________________________
/**
 * Constructor from XML file
 */
ExternalForce::ExternalForce(SimTK::Xml::Element& aNode) :
	Force(aNode)
{
	setNull();
	setupProperties();
	updateFromXMLNode(aNode);
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
ExternalForce::ExternalForce(const ExternalForce& force) :
	Force(force)
{
	setNull();
	setupProperties();
	copyData(force);
}

void ExternalForce::setNull()
{
	setType("ExternalForce");
	_dataSource = NULL;
	_appliedToBody = NULL;
	_forceExpressedInBody = NULL;
	_pointExpressedInBody = NULL; 
}

void ExternalForce::copyData(const ExternalForce& orig)
{
	setPropertyValue("applied_to_body", orig.getPropertyValue<string>("applied_to_body"));
	setPropertyValue("force_expressed_in_body", orig.getPropertyValue<string>("force_expressed_in_body"));
	setPropertyValue("point_expressed_in_body", orig.getPropertyValue<string>("point_expressed_in_body"));

	setPropertyValue("data_source_name", orig.getPropertyValue<string>("data_source_name"));
	setPropertyValue("force_identifier", orig.getPropertyValue<string>("force_identifier"));
	setPropertyValue("point_identifier", orig.getPropertyValue<string>("point_identifier"));
	setPropertyValue("torque_identifier", orig.getPropertyValue<string>("torque_identifier"));

	if(orig._dataSource)
		_dataSource = orig._dataSource;
}


//_____________________________________________________________________________
/**
 * Copy this force and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this force.
 */
Object* ExternalForce::copy() const
{
	ExternalForce *force = new ExternalForce(*this);
	return force;
}

//-----------------------------------------------------------------------------
// UPDATE FROM XML NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update this object based on its XML node.
 */
void ExternalForce::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
	// Base class
	Force::updateFromXMLNode(aNode, versionNumber);

	//Specify all or none of the components
	//if(_dataSourceName == "" || _dataSourceName == "Unassigned")
	//{
	//	throw Exception("ExternalForce:: data source name must be assigned.");
	//}

	if(getPropertyValue<string>("force_identifier")=="" && getPropertyValue<string>("torque_identifier")=="")
	{
		throw Exception("ExternalForce:: no force or torque identified.");
	}
}	


/**
 * Connect properties to local pointers.
 */
void ExternalForce::setupProperties()
{
	addProperty<string>("applied_to_body",
		"string",
		"Name of the body the force is applied to.",
		"");
	addProperty<string>("force_expressed_in_body",
		"string",
		"Name of the body the force is expressed in (default is ground).",
		"");
	addProperty<string>("point_expressed_in_body",
		"string",
		"Name of the body the point is expressed in (default is ground).",
		"");
	addProperty<string>("force_identifier",
		"string",
		"Identifier (string) to locate the force to be applied in the data source.",
		"");
	addProperty<string>("point_identifier",
		"string",
		"Identifier (string) to locate the point to be applied in the data source.",
		"");
	addProperty<string>("torque_identifier",
		"string",
		"Identifier (string) to locate the torque to be applied in the data source.",
		"");
	addProperty<string>("data_source_name",
		"string",
		"Name of the data source (Storage) that will supply the force data.",
		"");
}

ExternalForce& ExternalForce::operator=(const ExternalForce &aForce)
{
	Force::operator=(aForce);
	copyData(aForce);
	return(*this);
}

void ExternalForce::setDataSource(const Storage *dataSource)
{ 
	_dataSource = dataSource;
	setPropertyValue("data_source_name", dataSource->getName());
}

void ExternalForce::setup(Model& model)
{
	Force::setup(model);

	const string &appliedToBodyName = getPropertyValue<string>("applied_to_body");
	const string &forceExpressedInBodyName = getPropertyValue<string>("force_expressed_in_body");
	const string &pointExpressedInBodyName = getPropertyValue<string>("point_expressed_in_body");
	const string &dataSourceName = getPropertyValue<string>("data_source_name");
	const string &forceIdentifier = getPropertyValue<string>("force_identifier");
	const string &pointIdentifier = getPropertyValue<string>("point_identifier");
	const string &torqueIdentifier = getPropertyValue<string>("torque_identifier");

	_appliesForce = appliesForce();
	_specifiesPoint = specifiesPoint();
	_appliesTorque = appliesTorque();

	// hook up body pointers from names
	if (_model){
		_appliedToBody = &_model->updBodySet().get(appliedToBodyName);
		_forceExpressedInBody = &_model->updBodySet().get(forceExpressedInBodyName);
		_pointExpressedInBody = _specifiesPoint ? &_model->updBodySet().get(pointExpressedInBodyName) : NULL;
	}

	if(!_appliedToBody){
		throw(Exception("ExternalForce: Could not find body '"+appliedToBodyName+"' to apply force to." ));
	}
	if(!_forceExpressedInBody){
		cout << "WARNING::ExternalForce could not find body '"+forceExpressedInBodyName+"' that force is expressed in-"
			    "  ground is being assumed." << endl;
	}
	if(_specifiesPoint && !_pointExpressedInBody){
		cout << "WARNING::ExternalForce could not find body '"+pointExpressedInBodyName+"' that point is expressed in-"
			    "  ground is being assumed." << endl;
	}

	if(_dataSource == NULL){
		throw(Exception("ExternalForce: Data source has not been set." ));
	}
	else if(_dataSource->getName() != dataSourceName){
		throw(Exception("ExternalForce: Data source "+dataSourceName+" specified by name, but "+_dataSource->getName()+" was set." ));
	}

	// temporary data arrays
	Array<double> time;
	Array<Array<double> > force;
	Array<Array<double> > point;
	Array<Array<double> > torque;

	// Get data
	_dataSource->getTimeColumn(time);
	int nt = time.getSize();
	if( nt < 1)
		throw(Exception("ExternalForce: No times found in data source: "+_dataSource->getName()));

	// have to apply either a force or a torque
	if(!_appliesForce && !_appliesTorque)
		throw(Exception("ExternalForce:"+getName()+" does not apply neither a force nor a torque.")); 

	// if a force is not being applied then specifying a point makes no sense
	if(!_appliesForce && _specifiesPoint)
		throw(Exception("ExternalForce:"+getName()+" Point is specified for no applied force.")); 

	_dataSource->getDataForIdentifier(forceIdentifier, force);
	if(_appliesForce && (force.getSize() != 3)) // if applying force MUST have 3 components
		throw(Exception("ExternalForce: 3 unique force components could not be found, for force identifier: "
		+forceIdentifier+
		"\n. Please make sure data file contains exactly 3 unique columns with this common prefix."));

	if(_specifiesPoint){
		_dataSource->getDataForIdentifier(pointIdentifier, point);
		if(point.getSize() != 3) // if specifying a point of application, it MUST have 3 components
			throw(Exception("ExternalForce: 3 unique point components could not be found, for point identifier: "
			+pointIdentifier+
			"\n. Please make sure data file contains exactly 3 unique columns with this common prefix."));
	}

	_dataSource->getDataForIdentifier(torqueIdentifier, torque);
	if(_appliesTorque && (torque.getSize() != 3)) // if specifying a point of application, it MUST have 3 components
		throw(Exception("ExternalForce: 3 unique torque components could not be identified for torque identifier: "
			+torqueIdentifier+
			"\n. Please make sure data file contains exactly 3 unique columns with this common prefix."));

	// clear out functions from previous data source
	_forceFunctions.clear();
	_pointFunctions.clear();
	_torqueFunctions.clear();

	// Create functions now that we should have good data remaining
	if(_appliesForce){
		for(int i=0; i<3; ++i){
			switch(nt) {
				case 1 :
					_forceFunctions.push_back(new Constant(force[i][0]));
					break;
				case 2 :
					_forceFunctions.push_back(new PiecewiseLinearFunction(force[i].getSize(), &time[0], &(force[i][0])) );
					break;
				case 3 :
					_forceFunctions.push_back(new PiecewiseLinearFunction(force[i].getSize(), &time[0], &(force[i][0])) );
					break;
				default:
					_forceFunctions.push_back(new GCVSpline( 3, force[i].getSize(), &time[0], &(force[i][0])) );
			}	
		}

		if(_specifiesPoint){
			for(int i=0; i<3; ++i){
				switch(nt) {
				case 1:
					_pointFunctions.push_back(new Constant(point[i][0]));
					break;
				case 2:
					_pointFunctions.push_back(new PiecewiseLinearFunction(point[i].getSize(), &time[0], &(point[i][0])) );
					break;
				case 3:
					_pointFunctions.push_back(new PiecewiseLinearFunction(point[i].getSize(), &time[0], &(point[i][0])) );
					break;
				default:
					_pointFunctions.push_back(new GCVSpline( 3, point[i].getSize(), &time[0], &(point[i][0])) );
				}
			}
		}
	}
	if(_appliesTorque){
		for(int i=0; i<3; ++i){
			switch(nt) {
				case 1:
					_torqueFunctions.push_back(new Constant(torque[i][0]));
					break;
				case 2:
					_torqueFunctions.push_back(new PiecewiseLinearFunction(torque[i].getSize(), &time[0], &(torque[i][0])) );
					break;
				case 3:
					_torqueFunctions.push_back(new PiecewiseLinearFunction(torque[i].getSize(), &time[0], &(torque[i][0])) );
					break;
				default:
					_torqueFunctions.push_back(new GCVSpline( 3, torque[i].getSize(), &time[0], &(torque[i][0])) );
			}
		}
	}
}



//-----------------------------------------------------------------------------
// FORCE METHODS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________

void ExternalForce::computeForce(const SimTK::State& state, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const
{
	double time = state.getTime();
	const SimbodyEngine& engine = getModel().getSimbodyEngine();

	assert(_appliedToBody!=0);

	if (_appliesForce) {
		Vec3 force = getForceAtTime(time);
		engine.transform(state, *_forceExpressedInBody, force, engine.getGroundBody(), force);
		if (!_specifiesPoint) {
			applyForce(state, *_appliedToBody, force, bodyForces);
 
	    }else {
			Vec3 point = getPointAtTime(time);
			engine.transformPosition(state, *_pointExpressedInBody, point, *_appliedToBody, point);
			applyForceToPoint(state, *_appliedToBody, point, force, bodyForces);
		}
	}

	if (_appliesTorque) {
		Vec3 torque = getTorqueAtTime(time);
		engine.transform(state, *_forceExpressedInBody, torque, engine.getGroundBody(), torque);
		applyTorque(state, *_appliedToBody, torque, bodyForces);
	}
}

/**
 * Conevenince methods to access prescribed force functions
 */
Vec3 ExternalForce::getForceAtTime(double aTime) const	
{
	SimTK::Vector timeAsVector(1, aTime);
	const Function* forceX=NULL;
	const Function* forceY=NULL;
	const Function* forceZ=NULL;
	if (_forceFunctions.size()==3){
		forceX=_forceFunctions[0];	forceY=_forceFunctions[1];	forceZ=_forceFunctions[2];
	}
	Vec3 force(forceX?forceX->calcValue(timeAsVector):0.0, 
		forceY?forceY->calcValue(timeAsVector):0.0, 
		forceZ?forceZ->calcValue(timeAsVector):0.0);
	return force;
}

Vec3 ExternalForce::getPointAtTime(double aTime) const
{
	SimTK::Vector timeAsVector(1, aTime);
	const Function* pointX=NULL;
	const Function* pointY=NULL;
	const Function* pointZ=NULL;
	if (_pointFunctions.size()==3){
		pointX=_pointFunctions[0];	pointY=_pointFunctions[1];	pointZ=_pointFunctions[2];
	}
	Vec3 point(pointX?pointX->calcValue(timeAsVector):0.0, 
		pointY?pointY->calcValue(timeAsVector):0.0, 
		pointZ?pointZ->calcValue(timeAsVector):0.0);
	return point;
}

Vec3 ExternalForce::getTorqueAtTime(double aTime) const
{
	SimTK::Vector timeAsVector(1, aTime);
	const Function* torqueX=NULL;
	const Function* torqueY=NULL;
	const Function* torqueZ=NULL;
	if (_torqueFunctions.size()==3){
		torqueX=_torqueFunctions[0];	torqueY=_torqueFunctions[1];	torqueZ=_torqueFunctions[2];
	}
	Vec3 torque(torqueX?torqueX->calcValue(timeAsVector):0.0, 
		torqueY?torqueY->calcValue(timeAsVector):0.0, 
		torqueZ?torqueZ->calcValue(timeAsVector):0.0);
	return torque;
}


//-----------------------------------------------------------------------------
// Reporting
//-----------------------------------------------------------------------------

OpenSim::Array<std::string> ExternalForce::getRecordLabels() const {
	OpenSim::Array<std::string> labels("");

	const string &appliedToBodyName = getPropertyValue<string>("applied_to_body");

	if (_appliesForce) {
		labels.append(appliedToBodyName+"_"+getName()+"_Fx");
		labels.append(appliedToBodyName+"_"+getName()+"_Fy");
		labels.append(appliedToBodyName+"_"+getName()+"_Fz");

		if (_specifiesPoint) {
			labels.append(appliedToBodyName+"_"+getName()+"_px");
			labels.append(appliedToBodyName+"_"+getName()+"_py");
			labels.append(appliedToBodyName+"_"+getName()+"_pz");
		}
	}
	if (_appliesTorque){
		labels.append(appliedToBodyName+"_"+getName()+"_Tx");
		labels.append(appliedToBodyName+"_"+getName()+"_Ty");
		labels.append(appliedToBodyName+"_"+getName()+"_Tz");
	}
	return labels;
}
/**
 * Given SimTK::State object extract all the values necessary to report forces, application location
 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
OpenSim::Array<double> ExternalForce::getRecordValues(const SimTK::State& state) const
{
	const SimbodyEngine& engine = getModel().getSimbodyEngine();
	OpenSim::Array<double>	values(SimTK::NaN);
	double time = state.getTime();

	if (_appliesForce) {
		Vec3 force = getForceAtTime(time);
		engine.transform(state, *_forceExpressedInBody, force, engine.getGroundBody(), force);
		for(int i=0; i<3; ++i)
			values.append(force[i]);
	
		if (_specifiesPoint) {
			Vec3 point = getPointAtTime(time);
			engine.transformPosition(state, *_pointExpressedInBody, point, *_appliedToBody, point);
			for(int i=0; i<3; ++i)
				values.append(point[i]);
		}
	}
	if (_appliesTorque){
		Vec3 torque = getTorqueAtTime(time);
		engine.transform(state, *_forceExpressedInBody, torque, engine.getGroundBody(), torque);
		for(int i=0; i<3; ++i)
			values.append(torque[i]);
	}

	return values;
};


