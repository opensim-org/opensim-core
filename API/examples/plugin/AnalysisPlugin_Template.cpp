// AnalysisPlugin_Template.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, Ajay Seth
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2008 Stanford University
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
#include <iostream>
#include <string>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include "AnalysisPlugin_Template.h"

using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTANTS
//=============================================================================

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 * Delete any variables allocated using the "new" operator.  You will not
 * necessarily have any of these.
 */
AnalysisPlugin_Template::~AnalysisPlugin_Template()
{
	//deleteStorage();
}
//_____________________________________________________________________________
/*
 * Construct an AnalysisPlugin_Template instance.
 *
 * @param aModel Model for which the analysis is to be run.
 */
AnalysisPlugin_Template::AnalysisPlugin_Template(Model *aModel) :
	Analysis(aModel),
	//_bool(_boolProp.getValueBool()),
	//_boolArray(_boolArrayProp.getValueBoolArray()),
	//_int(_intProp.getValueInt()),
	//_intArray(_intArrayProp.getValueIntArray()),
	//_dbl(_dblProp.getValueDbl()),
	//_dblArray(_dblArrayProp.getValueDblArray()),
	//_vec3(_vec3Prop.getValueDblVec3()),
	//_str(_strProp.getValueStr()),
	_bodyNames(_strArrayProp.getValueStrArray())
{
	// make sure members point to NULL if not valid. 
	setNull();
	if(_model==NULL) return;

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

}
//_____________________________________________________________________________
/*
 * Construct an object from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
AnalysisPlugin_Template::AnalysisPlugin_Template(const std::string &aFileName):
	Analysis(aFileName, false),
	//_bool(_boolProp.getValueBool()),
	//_boolArray(_boolArrayProp.getValueBoolArray()),
	//_int(_intProp.getValueInt()),
	//_intArray(_intArrayProp.getValueIntArray()),
	//_dbl(_dblProp.getValueDbl()),
	//_dblArray(_dblArrayProp.getValueDblArray()),
	//_vec3(_vec3Prop.getValueDblVec3()),
	//_str(_strProp.getValueStr()),
	_bodyNames(_strArrayProp.getValueStrArray())
{
	setNull();

	// Read properties from XML
	updateFromXMLNode();
}

// Copy constrctor and virtual copy 
//_____________________________________________________________________________
/*
 * Copy constructor.
 *
 */
AnalysisPlugin_Template::AnalysisPlugin_Template(const AnalysisPlugin_Template &aAnalysisPlugin_Template):
	Analysis(aAnalysisPlugin_Template),
	//_bool(_boolProp.getValueBool()),
	//_boolArray(_boolArrayProp.getValueBoolArray()),
	//_int(_intProp.getValueInt()),
	//_intArray(_intArrayProp.getValueIntArray()),
	//_dbl(_dblProp.getValueDbl()),
	//_dblArray(_dblArrayProp.getValueDblArray()),
	//_vec3(_vec3Prop.getValueDblVec3()),
	//_str(_strProp.getValueStr()),
	_bodyNames(_strArrayProp.getValueStrArray())
{
	setNull();
	// COPY TYPE AND NAME
	*this = aAnalysisPlugin_Template;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* AnalysisPlugin_Template::copy() const
{
	AnalysisPlugin_Template *object = new AnalysisPlugin_Template(*this);
	return(object);

}
//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/*
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
AnalysisPlugin_Template& AnalysisPlugin_Template::
operator=(const AnalysisPlugin_Template &aAnalysisPlugin_Template)
{
	// Base Class
	Analysis::operator=(aAnalysisPlugin_Template);

	// Member Variables
	_bodyNames = aAnalysisPlugin_Template._bodyNames;

	return(*this);
}

//_____________________________________________________________________________
/**
 * SetNull().
 */
void AnalysisPlugin_Template::
setNull()
{
	setType("AnalysisPlugin_Template");
	setupProperties();

	// Property Default Values
	//_bool = true;
	//_boolArray.setSize(0);
	//_int = 0;
	//_intArray.setSize(0);
	//_dbl = 0.0;
	//_dblArray.setSize(0);
	//_vec3[0] = _vec3[1] = _vec3[2] = 0.0;
	//_str = "";
	_bodyNames.setSize(1);
	_bodyNames[0] = "all";
}
//_____________________________________________________________________________
/*
 * Set up the properties for your analysis.
 *
 * You should give each property a meaningful name and an informative comment.
 * The name you give each property is the tag that will be used in the XML
 * file.  The comment will appear before the property in the XML file.
 * In addition, the comments are used for tool tips in the OpenSim GUI.
 *
 * All properties are added to the property set.  Once added, they can be
 * read in and written to file.
 */
void AnalysisPlugin_Template::
setupProperties()
{
	// Uncomment and rename the properties required for your analysis

	//_boolProp.setName("bool_parameter");
	//_boolProp.setComment("Flag indicating whether or not to ...");
	//_propertySet.append(&_boolProp);

	//_boolArrayProp.setName("array_of_bool_params");
	//_boolArrayProp.setComment("Array of flags indicating whether or not to ...");
	//_propertySet.append(&_boolArrayProp);

	//_intProp.setName("integer_parameter");
	//_intProp.setComment("Number of ...");
	//_propertySet.append(&_intProp);

	//_intArrayProp.setName("array_of_integer_paramters");
	//_intArrayProp.setComment("Array of numbers per ...");
	//_propertySet.append(&_intArrayProp);

	//_dblProp.setName("double_precision_parameter");
	//_dblProp.setComment("A double precision value for...");
	//_propertySet.append(&_dblProp);

	//_dblArrayProp.setName("array_of_doubles");
	//_dblArrayProp.setComment("Array of double precision parameters ...");
	//_propertySet.append(&_dblArrayProp);

	//_vec3Prop.setName("vec3_parameter");
	//_vec3Prop.setComment("Vector in 3 space.");
	//_propertySet.append(&_vec3Prop);

	//_strProp.setName("string_parameter");
	//_strProp.setComment("String parameter identifying ...");
	//_propertySet.append(&_strProp);

	_strArrayProp.setName("body_names");
	_strArrayProp.setComment("Names of the bodies on which to perform the analysis."
		"The key word 'All' indicates that the analysis should be performed for all bodies.");
	_propertySet.append(&_strArrayProp);
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

	if(_bodyNames[0] == "all"){
		_bodyIndices.setSize(bodySet.getSize());
		// Get indices of all the bodies.
		for(int j=0;j<bodySet.getSize();j++)
			_bodyIndices[j]=j;
	}
	else{
		_bodyIndices.setSize(_bodyNames.getSize());
		// Get indices of just the bodies listed.
		for(int j=0;j<_bodyNames.getSize();j++)
			_bodyIndices[j]=bodySet.getIndex(_bodyNames[j]);
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


//=============================================================================
// GET AND SET
//=============================================================================
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
	Analysis::setModel(aModel);

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
step(const SimTK::State& s)
{
	if(!proceed(_model->getStep(s))) return(0);

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
	_storePos.scaleTime(_model->getTimeNormConstant());
	Storage::printResult(&_storePos,aBaseName+"_"+getName()+"_pos",aDir,aDT,aExtension);

	// VELOCITIES


	// ACCELERATIONS

	return(0);
}


