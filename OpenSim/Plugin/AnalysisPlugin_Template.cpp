// AnalysisPlugin_Template.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, Ajay Seth, Tim Dorn
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2012 Stanford University
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
#include "AnalysisPlugin_Template.h"

using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND SETUP
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor
 */
AnalysisPlugin_Template::AnalysisPlugin_Template() : Analysis()
{
    setNull();
    constructProperties();
}



//_____________________________________________________________________________
/**
 * SetNull()
 */
void AnalysisPlugin_Template::
setNull()
{
    _bodyIndices = NULL;
    _bodypos = NULL;
}


//_____________________________________________________________________________
/*
 * constructProperties()
 */
void AnalysisPlugin_Template::
constructProperties()
{
    Array<string> defaultBodyNames;
    defaultBodyNames.append("all");
    constructProperty_body_names(defaultBodyNames);

    // Here are some examples of other constructing other scalar property types.
    // Uncomment them as you need them.
    // ------------------------------------------------------
    //constructProperty_string_property("defaultString");
    //constructProperty_int_property(10);
    //constructProperty_bool_property(true);
    //constructProperty_double_property(1.5);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a description for the body kinematics files.
 */
void AnalysisPlugin_Template::
constructDescription()
{
	string descrip;

	descrip = "\nThis file contains body positions (of origin) and orientations.\n";
	descrip += "\nUnits are S.I. units (seconds, meters, Newtons, ...)";
	if(getInDegrees()) {
		descrip += "\nAngles are in degrees.";
	} else {
		descrip += "\nAngles are in radians.";
	}
	descrip += "\n\n";

	setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the output results.
 *
 * For analyses that run during a simulation, the first column is almost
 * always time.  For the purpose of example, the code below adds labels
 * appropriate for recording the translation and orientation of each
 * body in the model.
 *
 * This method needs to be called as necessary to update the column labels.
 */
void AnalysisPlugin_Template::
constructColumnLabels()
{
	if(_model==NULL) return;

	Array<string> labels;
	labels.append("time");

	const BodySet& bodySet = _model->getBodySet();

	if(get_body_names(0) == "all"){
		_bodyIndices.setSize(bodySet.getSize());
		// Get indices of all the bodies.
		for(int j=0;j<bodySet.getSize();j++)
			_bodyIndices[j]=j;
	}
	else{
		_bodyIndices.setSize(getProperty_body_names().size());
		// Get indices of just the bodies listed.
		for(int j=0;j<getProperty_body_names().size();j++)
			_bodyIndices[j]=bodySet.getIndex(get_body_names(j));
	}

	//Do the analysis on the bodies that are in the indices list
	for(int i=0; i<_bodyIndices.getSize(); i++) {
		const Body& body = bodySet.get(_bodyIndices[i]);
		labels.append(body.getName() + "_X");
		labels.append(body.getName() + "_Y");
		labels.append(body.getName() + "_Z");
		labels.append(body.getName() + "_Ox");
		labels.append(body.getName() + "_Oy");
		labels.append(body.getName() + "_Oz");
	}

	setColumnLabels(labels);
}

//_____________________________________________________________________________
/**
 * Set up storage objects.
 *
 * In general, the storage objects in your analysis are used to record
 * the results of your analysis and write them to file.  You will often
 * have a number of storage objects, each for recording a different
 * kind of result.
 */
void AnalysisPlugin_Template::
setupStorage()
{
	// Positions
	_storePos.reset(0);
	_storePos.setName("Positions");
	_storePos.setDescription(getDescription());
	_storePos.setColumnLabels(getColumnLabels());
}


//_____________________________________________________________________________
/**
 * Set the model for which this analysis is to be run.
 *
 * Sometimes the model on which an analysis should be run is not available
 * at the time an analysis is created.  Or, you might want to change the
 * model.  This method is used to set the model on which the analysis is
 * to be run.
 *
 * @param aModel Model pointer
 */
void AnalysisPlugin_Template::
setModel(Model& aModel)
{
	// SET THE MODEL IN THE BASE CLASS
	Super::setModel(aModel);

	// UPDATE VARIABLES IN THIS CLASS
	constructDescription();
	constructColumnLabels();
	setupStorage();

	//Setup size of work array to hold body positions
	int numBodies = _bodyIndices.getSize();
	_bodypos.setSize(6*numBodies);
}


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute and record the results.
 *
 * This method, for the purpose of example, records the position and
 * orientation of each body in the model.  You will need to customize it
 * to perform your analysis.
 *
 * @param aT Current time in the simulation.
 * @param aX Current values of the controls.
 * @param aY Current values of the states: includes generalized coords and speeds
 */
int AnalysisPlugin_Template::
record(const SimTK::State& s)
{
	// VARIABLES
	double dirCos[3][3];
	SimTK::Vec3 vec,angVec;
	double Mass = 0.0;

	// GROUND BODY
	const Body& ground = _model->getGroundBody();

	// POSITION
	const BodySet& bodySet = _model->getBodySet();

	for(int i=0;i<_bodyIndices.getSize();i++) {

		const Body& body = bodySet.get(_bodyIndices[i]);
		SimTK::Vec3 com;
		body.getMassCenter(com);

		// GET POSITIONS AND EULER ANGLES
		_model->getSimbodyEngine().getPosition(s,body,com,vec);
		_model->getSimbodyEngine().getDirectionCosines(s,body,dirCos);
		_model->getSimbodyEngine().convertDirectionCosinesToAngles(dirCos,
			&angVec[0],&angVec[1],&angVec[2]);

		// CONVERT TO DEGREES?
		if(getInDegrees()) {
			angVec *= SimTK_RADIAN_TO_DEGREE;
		}			

		// FILL KINEMATICS ARRAY
		int I=6*i;
		memcpy(&_bodypos[I],&vec[0],3*sizeof(double));
		memcpy(&_bodypos[I+3],&angVec[0],3*sizeof(double));
	}
	_storePos.append(s.getTime(),_bodypos.getSize(),&_bodypos[0]);

	// VELOCITY 

	// ACCELERATIONS


	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the begining of an integration in
 * Model::integBeginCallback() and has the same argument list.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that will be attempted.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 *
 * @return -1 on error, 0 otherwise.
 */
int AnalysisPlugin_Template::
begin(SimTK::State& s)
{
	if(!proceed()) return(0);

	// RESET STORAGE
	_storePos.reset(s.getTime());  //->reset(s.getTime());

	// RECORD
	int status = 0;
	if(_storePos.getSize()<=0) {
		status = record(s);
	}

	return(status);
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * When called during an integration, this method is meant to be called in
 * Model::integStepCallback(), which has the same argument list.
 *
 * @param aXPrev Controls at the beginining of the current time step.
 * @param aYPrev States at the beginning of the current time step.
 * @param aYPPrev Pseudo states at the beginning of the current time step.
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just taken.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 *
 * @return -1 on error, 0 otherwise.
 */
int AnalysisPlugin_Template::
step(const SimTK::State& s, int stepNumber)
{
	if(!proceed(stepNumber)) return(0);

	record(s);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration in
 * Model::integEndCallback() and has the same argument list.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just completed.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 *
 * @return -1 on error, 0 otherwise.
 */
int AnalysisPlugin_Template::
end(SimTK::State& s)
{
	if(!proceed()) return(0);

	record(s);

	return(0);
}




//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Print results.
 * 
 * The file names are constructed as
 * aDir + "/" + aBaseName + "_" + ComponentName + aExtension
 *
 * @param aDir Directory in which the results reside.
 * @param aBaseName Base file name.
 * @param aDT Desired time interval between adjacent storage vectors.  Linear
 * interpolation is used to print the data out at the desired interval.
 * @param aExtension File extension.
 *
 * @return 0 on success, -1 on error.
 */
int AnalysisPlugin_Template::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	// POSITIONS
	//_storePos.scaleTime(_model->getTimeNormConstant());
	Storage::printResult(&_storePos,aBaseName+"_"+getName()+"_pos",aDir,aDT,aExtension);

	// VELOCITIES


	// ACCELERATIONS

	return(0);
}


