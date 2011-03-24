// DynamicsTool.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Ajay Seth
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2010 Stanford University
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
#include "DynamicsTool.h"
#include <string>
#include <iostream>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/GCVSplineSet.h>

using namespace OpenSim;
using namespace std;
using namespace SimTK;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
DynamicsTool::~DynamicsTool()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
DynamicsTool::DynamicsTool() : Tool(),
	_modelFileName(_modelFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblVec()),
	_excludedForces(_excludedForcesProp.getValueStrArray()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr())
{
	setType("DynamicsTool");
	setNull();
}
//_____________________________________________________________________________
/**
 * Construct from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
DynamicsTool::DynamicsTool(const string &aFileName, bool aLoadModel) :
	Tool(aFileName, false),
	_modelFileName(_modelFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblVec()),
	_excludedForces(_excludedForcesProp.getValueStrArray()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr())
{
	setType("DynamicsTool");
	setNull();
	updateFromXMLNode();

	if(aLoadModel) {
		//loadModel(aFileName);
	}
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aTool Object to be copied.

 */
DynamicsTool::DynamicsTool(const DynamicsTool &aTool) : Tool(aTool),
	_modelFileName(_modelFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblVec()),
	_excludedForces(_excludedForcesProp.getValueStrArray()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr())
{
	setType("DynamicsTool");
	setNull();
	*this = aTool;
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void DynamicsTool::setNull()
{
	setupProperties();
	_model = NULL;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void DynamicsTool::setupProperties()
{
	_modelFileNameProp.setComment("Name of the .osim file used to construct a model.");
	_modelFileNameProp.setName("model_file");
	_propertySet.append( &_modelFileNameProp );

	SimTK::Vec2  defaultTimeRange(-SimTK::Infinity, SimTK::Infinity);
	_timeRangeProp.setComment("Time range over which the inverse dynamics problem is solved.");
	_timeRangeProp.setName("time_range");
	_timeRangeProp.setValue(defaultTimeRange);
	_propertySet.append(&_timeRangeProp);

	_excludedForcesProp.setComment("List of forces by individual or grouping name (e.g. All, actuators, muscles, ...)"
		" to be excluded when computing model dynamics.");
	_excludedForcesProp.setName("forces_to_exclude");
	_propertySet.append(&_excludedForcesProp);

	string comment = "XML file (.xml) containing the external loads applied to the model as a set of PrescribedForce(s).";
	_externalLoadsFileNameProp.setComment(comment);
	_externalLoadsFileNameProp.setName("external_loads_file");
	_propertySet.append( &_externalLoadsFileNameProp );

}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
DynamicsTool& DynamicsTool::operator=(const DynamicsTool &aTool)
{
	// BASE CLASS
	Tool::operator=(aTool);

	// MEMBER VARIABLES
	_modelFileName = aTool._modelFileName;
	_timeRange = aTool._timeRange;
	_excludedForces = aTool._excludedForces;
	_externalLoadsFileName = aTool._externalLoadsFileName;

	return(*this);
}

/** Modify model to exclude specified forces by disabling those identified by name or group */
void DynamicsTool::disableModelForces(Model &model, SimTK::State &s, const Array<std::string> &forcesByNameOrGroup)
{	
	ForceSet &modelForces = model.updForceSet();
	Array<string> groupNames;
	modelForces.getGroupNames(groupNames);

	/* The search for inidividual group or force names IS case-sensitive BUT keywords are not*/
	for(int i=0; i<forcesByNameOrGroup.getSize(); i++){
		//Check for kewords first starting with ALL
		if(IO::Uppercase(forcesByNameOrGroup[i]) == "ALL"){
			for(int i=0; i<modelForces.getSize(); i++){
				modelForces[i].setDisabled(s, true);
			}
			return;
		}
		if(IO::Uppercase(forcesByNameOrGroup[i]) == "ACTUATORS"){
			Set<Actuator> &acts = model.updActuators();
			for(int i=0; i<acts.getSize(); i++){
				acts[i].setDisabled(s, true);
			}
			continue;
		}
		if(IO::Uppercase(forcesByNameOrGroup[i]) == "MUSCLES"){
			Set<Muscle> &muscles = model.updMuscles();
			for(int i=0; i<muscles.getSize(); i++){
				muscles[i].setDisabled(s, true);
			}
			continue;
		}

		// index result when a name is not a force or group name for forces in the model
		int k = -1;  
		// It is possible to set ACTUATORS and/or MUSCLES and other forces by name or group
		// So check what else is in the list.
		// see if name is a group name first	
		if(groupNames.getSize() > 0){
			k = groupNames.findIndex(forcesByNameOrGroup[i]);
			if(k > -1){ //found
				const ObjectGroup* group = modelForces.getGroup(k);
				Array<Object*> members = group->getMembers();
				for(int j=0; j<members.getSize(); j++)
					((Force *)(members[j]))->setDisabled(s, true);
			}
		} //otherwise, check for individual forces
		else{
			k = modelForces.getIndex(forcesByNameOrGroup[i]);
			if(k > -1){ //found
				modelForces[k].setDisabled(s, true);
			}
		}
		// No force or group was found
		if(k < 0)
			cout << "\nWARNING: Tool could not find force or group named '" << forcesByNameOrGroup[i] << "' to be excluded." << endl;

	}
}

bool DynamicsTool::createExternalLoads( const string& aExternalLoadsFileName,
                                        Model& aModel) {

    if(aExternalLoadsFileName==""||aExternalLoadsFileName=="Unassigned") {
        cout<<"No external loads will be applied (external loads file not specified)."<<endl;
        return false;
    }


	// CREATE FORCE AND TORQUE APPLIERS
	_externalLoads = ExternalLoads(aModel, aExternalLoadsFileName);
	_externalLoads.setup(aModel);
	_externalLoads.setMemoryOwner(false);
	for (int i=0; i<_externalLoads.getSize(); i++){
		aModel.updForceSet().append(&_externalLoads.get(i)); // Call this instead of addForce to avoid calling setup repeatedly
	}

    return(true);

}

void DynamicsTool::
initializeExternalLoads( SimTK::State& s, 
                         const double& analysisStartTime, 
                         const double& analysisFinalTime
						 )
{
	const string &aExternalLoadsFileName=_externalLoadsFileName;
	const string &aExternalLoadsModelKinematicsFileName = _externalLoads.getExternalLoadsModelKinematicsFileName();
	double aLowpassCutoffFrequencyForLoadKinematics=_externalLoads.getLowpassCutoffFrequencyForLoadKinematics();
	bool externalLoadKinematicsSpecified = (aExternalLoadsModelKinematicsFileName!="");
	//const Model& aModel = _externalLoads.getModel();
	Storage *qStore=NULL;
	Storage *uStore=NULL;
	if (externalLoadKinematicsSpecified){
		cout<<"\n\nLoading external loads kinematics from file "<<aExternalLoadsModelKinematicsFileName<<" ...\n";
		Storage loadsKinStore(aExternalLoadsModelKinematicsFileName);

		// Form complete storage objects for the q's and u's
		// This means filling in unspecified generalized coordinates and
		// setting constrained coordinates to their valid values.
		Storage *uStoreTmp=NULL;
		_model->getSimbodyEngine().formCompleteStorages(s, loadsKinStore,qStore,uStoreTmp);
		_model->getSimbodyEngine().convertDegreesToRadians(*qStore);
		// Filter
		qStore->pad(qStore->getSize()/2); 
		if(aLowpassCutoffFrequencyForLoadKinematics>=0) {
			int order = 50;
			cout<<"Low-pass filtering external load kinematics with a cutoff frequency of "
				<<aLowpassCutoffFrequencyForLoadKinematics<<"..."<<endl;
			qStore->lowpassFIR(order, aLowpassCutoffFrequencyForLoadKinematics);
		} else {
			cout<<"Note- not filtering the external loads model kinematics."<<endl;
		}
		// Spline
		GCVSplineSet qSet(3,qStore);
		uStore = qSet.constructStorage(1);
	}
	// LOAD COP, FORCE, AND TORQUE
	Storage kineticsStore(_externalLoads.getDataFileName());

	int copSize = kineticsStore.getSize();
	if(copSize<=0) return;
	// Make sure we have data for the requested range
	if (kineticsStore.getFirstTime() > analysisStartTime || 
		kineticsStore.getLastTime() < analysisFinalTime){
		char durationString[100];
		sprintf(durationString, "from t=%lf to t=%lf", analysisStartTime, analysisFinalTime);
		string msg = "ERR- Requested simulation time  extends outside provided data." + string(durationString);
		throw Exception(msg,__FILE__,__LINE__);
	}
	
	/* Due to earlier resampling and in general analysisStartTime, analysisFinalTime may not coincide with actual
	 * rows in qStore, in this case we need to evaluate the ForceApplier and TorqueApplier outside 
	 * analysis[start, final]time up to the time of the row before analysisStartTime, and the row after analysisFinalTime.
	 * Adjust analysisStartTime, analysisFinalTime to qStartTime, qFinalTime to account for that */
	if (externalLoadKinematicsSpecified){
		int startIndex = qStore->findIndex(analysisStartTime);
		double qStartTime, qFinalTime;
		qStore->getTime(startIndex, qStartTime);
		int finalIndex = qStore->findIndex(analysisFinalTime);
		qStore->getTime(finalIndex, qFinalTime);
		Array<double> analysisBoundTimes;
		if (qStartTime < analysisStartTime && startIndex >=1){
			analysisBoundTimes.append(analysisStartTime);
			//qStore->getTime(startIndex-1, qStartTime); 
			qStartTime = analysisStartTime;
		}
		if (qFinalTime < analysisFinalTime && finalIndex < qStore->getSize()-1){
			analysisBoundTimes.append(analysisFinalTime);
			//qStore->getTime(finalIndex+1, qFinalTime); 
			qFinalTime = analysisFinalTime;
		}
		qStore->interpolateAt(analysisBoundTimes);
	}
	// Construt point, force and torque functions from file
	_externalLoads.computeFunctions(s, analysisStartTime, analysisFinalTime, kineticsStore, qStore, uStore);

}
