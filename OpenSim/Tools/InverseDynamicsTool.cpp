// InverseDynamicsTool.cpp
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
#include "InverseDynamicsTool.h"
#include <string>
#include <iostream>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/JointSet.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/FunctionSet.h> 
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/Constant.h>
#include "AnalyzeTool.h"

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
InverseDynamicsTool::~InverseDynamicsTool()
{
	if (_coordinateValues) {
		delete _coordinateValues; _coordinateValues=NULL; 
	}
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
InverseDynamicsTool::InverseDynamicsTool() : DynamicsTool(),
	_coordinatesFileName(_coordinatesFileNameProp.getValueStr()),
	_lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
	_outputGenForceFileName(_outputGenForceFileNameProp.getValueStr()),
	_jointsForReportingBodyForces(_jointsForReportingBodyForcesProp.getValueStrArray()),
	_outputBodyForcesAtJointsFileName(_outputBodyForcesAtJointsFileNameProp.getValueStr())
{
	setType("InverseDynamicsTool");
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
InverseDynamicsTool::InverseDynamicsTool(const string &aFileName, bool aLoadModel) :
	DynamicsTool(aFileName, false),
	_coordinatesFileName(_coordinatesFileNameProp.getValueStr()),
	_lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
	_outputGenForceFileName(_outputGenForceFileNameProp.getValueStr()),
	_jointsForReportingBodyForces(_jointsForReportingBodyForcesProp.getValueStrArray()),
	_outputBodyForcesAtJointsFileName(_outputBodyForcesAtJointsFileNameProp.getValueStr())
{
	setType("InverseDynamicsTool");
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
InverseDynamicsTool::InverseDynamicsTool(const InverseDynamicsTool &aTool) :
	DynamicsTool(aTool),
	_coordinatesFileName(_coordinatesFileNameProp.getValueStr()),
	_lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
	_outputGenForceFileName(_outputGenForceFileNameProp.getValueStr()),
	_jointsForReportingBodyForces(_jointsForReportingBodyForcesProp.getValueStrArray()),
	_outputBodyForcesAtJointsFileName(_outputBodyForcesAtJointsFileNameProp.getValueStr())
{
	setType("InverseDynamicsTool");
	setNull();
	*this = aTool;
}

//_____________________________________________________________________________
/**
 * Virtual copy constructor.
 */
Object* InverseDynamicsTool::copy() const
{
	InverseDynamicsTool *object = new InverseDynamicsTool(*this);
	return(object);
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void InverseDynamicsTool::setNull()
{
	setupProperties();
	_model = NULL;
	_lowpassCutoffFrequency = -1.0;
	_coordinateValues = NULL;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void InverseDynamicsTool::setupProperties()
{
	_modelFileNameProp.setComment("Name of the .osim file used to construct a model.");
	_modelFileNameProp.setName("model_file");
	_propertySet.append( &_modelFileNameProp );

	_coordinatesFileNameProp.setComment("The name of the file containing coordinate data. Can be a motion (.mot) or a states (.sto) file.");
	_coordinatesFileNameProp.setName("coordinates_file");
	_propertySet.append(&_coordinatesFileNameProp);

	string comment = "Low-pass cut-off frequency for filtering the coordinates_file data (currently does not apply to states_file or speeds_file). "
				 "A negative value results in no filtering. The default value is -1.0, so no filtering.";
	_lowpassCutoffFrequencyProp.setComment(comment);
	_lowpassCutoffFrequencyProp.setName("lowpass_cutoff_frequency_for_coordinates");
	_propertySet.append( &_lowpassCutoffFrequencyProp );

	SimTK::Vec2  defaultTimeRange(-std::numeric_limits<SimTK::Real>::infinity(), std::numeric_limits<SimTK::Real>::infinity());
	_timeRangeProp.setComment("Time range over which the inverse dynamics problem is solved.");
	_timeRangeProp.setName("time_range");
	_timeRangeProp.setValue(defaultTimeRange);
	_propertySet.append(&_timeRangeProp);

	_outputGenForceFileNameProp.setComment("Name of the storage file (.sto) to which the generalized forces are written.");
	_outputGenForceFileNameProp.setName("output_gen_force_file");
	_outputGenForceFileNameProp.setValue("inverse_dynamics.sto");
	_propertySet.append(&_outputGenForceFileNameProp);

	_jointsForReportingBodyForcesProp.setComment("List of joints (keyword All, for all joints)"
		" to report body forces acting at the joint frame expressed in ground.");
	_jointsForReportingBodyForcesProp.setName("joints_to_report_body_forces");
	_propertySet.append(&_jointsForReportingBodyForcesProp);

	_outputBodyForcesAtJointsFileNameProp.setComment("Name of the storage file (.sto) to which the body forces at specified joints are written.");
	_outputBodyForcesAtJointsFileNameProp.setName("output_body_forces_file");
	_outputBodyForcesAtJointsFileNameProp.setValue("body_forces_at_joints.sto");
	_propertySet.append(&_outputBodyForcesAtJointsFileNameProp);
}

//_____________________________________________________________________________
/**
 * Register InverseDynamicsTool and any Object types it may employ internally.
 */
void InverseDynamicsTool::registerTypes()
{
	Object::RegisterType(InverseDynamicsTool());
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
InverseDynamicsTool& InverseDynamicsTool::
operator=(const InverseDynamicsTool &aTool)
{
	// BASE CLASS
	DynamicsTool::operator=(aTool);

	// MEMBER VARIABLES
	_modelFileName = aTool._modelFileName;
	_coordinatesFileName = aTool._coordinatesFileName;
	_lowpassCutoffFrequency = aTool._lowpassCutoffFrequency;
	_outputGenForceFileName = aTool._outputGenForceFileName;
	_outputBodyForcesAtJointsFileName = aTool._outputBodyForcesAtJointsFileName;
	_coordinateValues = NULL;

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================

void InverseDynamicsTool::setCoordinateValues(const OpenSim::Storage& aStorage)
{
	if (_coordinateValues) delete _coordinateValues;
	_coordinateValues = new Storage(aStorage);
}


/** Build the list of Joints for computing and reporting equivalent body forces */
void InverseDynamicsTool::getJointsByName(Model &model, const Array<std::string> &jointNames, JointSet &joints) const
{	
	const JointSet &modelJoints = model.getJointSet();
	Array<string> groupNames;
	modelJoints.getGroupNames(groupNames);

	/* The search for inidividual group or force names IS case-sensitive BUT keywords are not*/
	for(int i=0; i<jointNames.getSize();  ++i){
		//Check for kewords first starting with ALL
		if(IO::Uppercase(jointNames[i]) == "ALL"){
			for(int j=0; j<modelJoints.getSize(); ++j){
				joints.append(&modelJoints[j]);
			}
			break;
		} 
		
		int k = modelJoints.getIndex(jointNames[i]);
		if (k >= 0){
			joints.append(&modelJoints[k]);
		} else {
			cout << "\nWARNING: InverseDynamicsTool could not find Joint named '" << jointNames[i] << "' to report body forces." << endl;
		}
	}
	joints.setMemoryOwner(false);
}


//=============================================================================
// RUN
//=============================================================================
//_____________________________________________________________________________
/**
 * Run the inverse Dynamics tool.
 */
bool InverseDynamicsTool::run()
{
	bool success = false;
	bool modelFromFile=true;
	try{
		//Load and create the indicated model
		if (!_model) 
			_model = new Model(_modelFileName);
		else
			modelFromFile = false;
		_model->printBasicInfo(cout);

		cout<<"Running tool " << getName() <<".\n"<<endl;

		// Do the maneuver to change then restore working directory 
		// so that the parsing code behaves properly if called from a different directory.
		string saveWorkingDirectory = IO::getCwd();
		string directoryOfSetupFile = IO::getParentDirectory(getDocumentFileName());
		IO::chDir(directoryOfSetupFile);

		const CoordinateSet &coords = _model->getCoordinateSet();
		int nq = _model->getNumCoordinates();

		FunctionSet *coordFunctions = NULL;
		//Storage *coordinateValues = NULL;

		if(hasCoordinateValues()){
			if(_lowpassCutoffFrequency>=0) {
				cout<<"\n\nLow-pass filtering coordinates data with a cutoff frequency of "<<_lowpassCutoffFrequency<<"..."<<endl<<endl;
				_coordinateValues->pad(_coordinateValues->getSize()/2);
				_coordinateValues->lowpassIIR(_lowpassCutoffFrequency);
				if (getVerboseLevel()==Debug) _coordinateValues->print("coordinateDataFiltered.sto");
			}
			// Convert degrees to radian if indicated
			if(_coordinateValues->isInDegrees()){
				_model->getSimbodyEngine().convertDegreesToRadians(*_coordinateValues);
			}
			// Create differentiable splines of the coordinate data
			coordFunctions = new GCVSplineSet(5, _coordinateValues);

			//Functions must correspond to model coordinates and their order for the solver
			for(int i=0; i<nq; i++){
				if(coordFunctions->contains(coords[i].getName())){
					coordFunctions->insert(i,coordFunctions->get(coords[i].getName()));
				}
				else{
					coordFunctions->insert(i,new Constant(coords[i].getDefaultValue()));
					std::cout << "InverseDynamicsTool: coordinate file does not contain coordinate " << coords[i].getName() << " assuming default value" << std::endl;
				}
			}
			if(coordFunctions->getSize() > nq){
				coordFunctions->setSize(nq);
			}
		}
		else{
			IO::chDir(saveWorkingDirectory);
			throw Exception("InverseDynamicsTool: no coordinate file found.");

		}

		bool externalLoads = createExternalLoads(_externalLoadsFileName, *_model, _coordinateValues);
		// Initialize the the model's underlying computational system and get its default state.
		SimTK::State& s = _model->initSystem();

		// Exclude user-specified forces from the dynamics for this analysis
		disableModelForces(*_model, s, _excludedForces);

		double first_time = _coordinateValues->getFirstTime();
		double last_time = _coordinateValues->getLastTime();

		// Determine the starting and final time for the Tool by comparing to what data is available
		double start_time = ( first_time > _timeRange[0]) ? first_time : _timeRange[0];
		double final_time = ( last_time < _timeRange[1]) ? last_time : _timeRange[1];
		int start_index = _coordinateValues->findIndex(start_time);
		int final_index = _coordinateValues->findIndex(final_time);

		// create the solver given the input data
		InverseDynamicsSolver ivdSolver(*_model);

		const clock_t start = clock();

		int nt = final_index-start_index+1;
		
		Array_<double> times(nt, 0.0);
		for(int i=0; i<nt; i++){
			times[i]=_coordinateValues->getStateVector(start_index+i)->getTime();
		}

		// Preallocate results
		Array_<Vector> genForceTraj(nt, Vector(nq, 0.0));

		// solve for the trajectory of generlized forces that correspond to the coordinate
		// trajectories provided
		ivdSolver.solve(s, *coordFunctions, times, genForceTraj);


		success = true;

		cout << "InverseDynamicsTool: " << nt << " time frames in " <<(double)(clock()-start)/CLOCKS_PER_SEC << "s\n" <<endl;
	
		JointSet jointsForEquivalentBodyForces;
		getJointsByName(*_model, _jointsForReportingBodyForces, jointsForEquivalentBodyForces);
		int nj = jointsForEquivalentBodyForces.getSize();

		Array<string> labels("time", nq+1);
		for(int i=0; i<nq; i++){
			labels[i+1] = coords[i].getName();
			labels[i+1] += (coords[i].getMotionType() == Coordinate::Rotational) ? "_moment" : "_force";
		}

		Array<string> body_force_labels("time", 6*nj+1);
		string XYZ = "XYZ";
		for(int i=0; i<nj; i++){
			string joint_body_label = jointsForEquivalentBodyForces[i].getName()+"_";
			joint_body_label += jointsForEquivalentBodyForces[i].getBody().getName();
			for(int k=0; k<3; ++k){
				body_force_labels[6*i+k+1] =  joint_body_label+"_F"+XYZ[k]; //first label is time
				body_force_labels[6*i+k+3+1] =  joint_body_label+"_M"+XYZ[k];
			}
		}

		Storage genForceResults(nt);
		Storage bodyForcesResults(nt);
		SpatialVec equivalentBodyForceAtJoint;

		for(int i=0; i<nt; i++){
			StateVector genForceVec(times[i], nq, &((genForceTraj[i])[0]));
			genForceResults.append(genForceVec);

			// if there are joints requested for equivalent body forces then calculate them
			if(nj>0){
				Vector forces(6*nj, 0.0);
				StateVector bodyForcesVec(times[i], 6*nj, &forces[0]);

				s.updTime() = times[i];
				Vector &q = s.updQ();
				Vector &u = s.updU();

				for(int j=0; j<nq; ++j){
					q[j] = coordFunctions->evaluate(j, 0, times[i]);
					u[j] = coordFunctions->evaluate(j, 1, times[i]);
				}
			
				for(int j=0; j<nj; ++j){
					equivalentBodyForceAtJoint = jointsForEquivalentBodyForces[j].calcEquivalentSpatialForce(s, genForceTraj[i]);
					for(int k=0; k<3; ++k){
						// body force components
						bodyForcesVec.setDataValue(6*j+k, equivalentBodyForceAtJoint[1][k]); 
						// body torque components
						bodyForcesVec.setDataValue(6*j+k+3, equivalentBodyForceAtJoint[0][k]);
					}
				}
				bodyForcesResults.append(bodyForcesVec);

			}
		}

		genForceResults.setColumnLabels(labels);
		genForceResults.setName("Inverse Dynamics Generalized Forces");

		IO::makeDir(getResultsDir());
		Storage::printResult(&genForceResults, _outputGenForceFileName, getResultsDir(), -1, ".sto");
		IO::chDir(saveWorkingDirectory);

		// if body forces to be reported for specified joints
		if(nj >0){
			bodyForcesResults.setColumnLabels(body_force_labels);
			bodyForcesResults.setName("Inverse Dynamics Body Forces at Specified Joints");

			IO::makeDir(getResultsDir());
			Storage::printResult(&bodyForcesResults, _outputBodyForcesAtJointsFileName, getResultsDir(), -1, ".sto");
			IO::chDir(saveWorkingDirectory);
		}

	}
	catch (OpenSim::Exception& ex) {
		std::cout << "InverseDynamicsTool Failed: " << ex.what() << std::endl;
		throw (Exception("InverseDynamicsTool Failed, please see messages window for details..."));
	}

	if (modelFromFile) delete _model;
	return success;
}

bool InverseDynamicsTool::hasCoordinateValues()
{
	if (_coordinateValues!= NULL) // Coordinates has been set from GUI
		return true;
	// Try constructing cooridnates from specified file
	if(_coordinatesFileName != "" && _coordinatesFileName != "Unassigned"){
			_coordinateValues = new Storage(_coordinatesFileName);
			_coordinateValues->setName(_coordinatesFileName);
			return true;
	}
	return false;
}
/* Handle reading older formats/Versioning */
void InverseDynamicsTool::updateFromXMLNode()
{
	int documentVersion = getDocument()->getDocumentVersion();
	if ( documentVersion < XMLDocument::getLatestVersion()){
		std::string newFileName = getDocumentFileName();
		if (documentVersion < 20300){
			std::string origFilename = getDocumentFileName();
			newFileName=IO::replaceSubstring(newFileName, ".xml", "_v23.xml");
			cout << "Old version setup file encountered. Converting to new file "<< newFileName << endl;
			SimTK::Xml::Document doc = SimTK::Xml::Document(origFilename);
			doc.writeToFile(newFileName);
		}
		if (documentVersion < 20201) {
			AnalyzeTool updateAnalyzeTool(newFileName, false);
			updateAnalyzeTool.print(newFileName);
		}
		if (documentVersion < 20202){
			// get filename and use SimTK::Xml to parse it
			SimTK::Xml::Document doc = SimTK::Xml::Document(newFileName);
			Xml::Element oldRoot = doc.getRootElement();
			SimTK::Xml::Document newDoc;
			string prefix = "";
			if (oldRoot.getElementTag()=="AnalyzeTool"){
				// Make OpenSimDocument node and copy root underneath it
				newDoc.getRootElement().setElementTag("OpenSimDocument");
				newDoc.getRootElement().setAttributeValue("Version", "20201");
				prefix = oldRoot.getRequiredAttributeValueAs<string>("name");
				// Move all children of root to toolNode
				newDoc.getRootElement().insertNodeAfter(newDoc.getRootElement().node_end(), oldRoot.clone());
			}
			else
				newDoc = doc;
			Xml::Element root = newDoc.getRootElement();
			if (root.getElementTag()=="OpenSimDocument"){
				int curVersion = root.getRequiredAttributeValueAs<int>("Version");
				if (curVersion <= 20201) root.setAttributeValue("Version", "20300");
				Xml::element_iterator iterTool(root.element_begin("AnalyzeTool"));
				iterTool->setElementTag("InverseDynamicsTool");
				prefix = iterTool->getRequiredAttributeValueAs<string>("name");
				// Remove children <output_precision>, <initial_time>, <final_time>
				Xml::element_iterator initTimeIter(iterTool->element_begin("initial_time"));
				double tool_initial_time = initTimeIter->getValueAs<double>();
				if (initTimeIter->isValid()) iterTool->eraseNode(initTimeIter);
				Xml::element_iterator finalTimeIter(iterTool->element_begin("final_time"));
				double tool_final_time = finalTimeIter->getValueAs<double>();
				if (finalTimeIter->isValid()) iterTool->eraseNode(finalTimeIter);
				Xml::element_iterator precisionIter(iterTool->element_begin("output_precision"));
				if (precisionIter->isValid()) iterTool->eraseNode(precisionIter);
				bool use_model_forces=false;
				// Handle missing or uninitialized values after parsing the old InverseDynamics "Analysis"
				// Find Analyses underneath it AnalyzeTool
				Xml::element_iterator iterAnalysisSet(iterTool->element_begin("AnalysisSet"));
				Xml::element_iterator iterObjects(iterAnalysisSet->element_begin("objects"));
				Xml::element_iterator iterAnalysis(iterObjects->element_begin("InverseDynamics"));
				if (iterAnalysis!= iterObjects->element_end()){
					// move children to top level
					Xml::element_iterator p = iterAnalysis->element_begin();
					//std::vector<std::string> deprectaed({"on", "in_degrees", "step_interval"});
					for (; p!= iterAnalysis->element_end(); ++p) {
						// skip <on>, <step_interval>, <in_degrees>
						if (p->getElementTag()=="on" ||
							p->getElementTag()=="in_degrees" ||
							p->getElementTag()=="step_interval" ||
							p->getElementTag()=="start_time" ||
							p->getElementTag()=="end_time")
							continue;
						else if (p->getElementTag()=="use_model_force_set"){
							String use_model_forcesStr = p->getValueAs<String>();
							use_model_forces = (use_model_forcesStr=="true");
						}
						else
							iterTool->insertNodeAfter( iterTool->node_end(), p->clone());
					}
					// insert elements for "forces_to_exclude" & "time_range"
					std::ostringstream stream;
					stream << tool_initial_time << " " << tool_final_time;
					iterTool->insertNodeAfter( iterTool->node_end(), Xml::Element("time_range", stream.str()));
					iterTool->insertNodeAfter( iterTool->node_end(), Xml::Element("forces_to_exclude", use_model_forces?"":"Muscles"));
					iterTool->insertNodeAfter( iterTool->node_end(), Xml::Element("output_gen_force_file", prefix+"_InverseDynamics.sto"));
					iterTool->insertNodeAfter( iterTool->node_end(), Xml::Element("coordinates_in_degrees", "true"));
					iterTool->eraseNode(iterAnalysisSet);
				}
				newDoc.writeToFile(newFileName);
				_document = new XMLDocument(newFileName);
				_node = _document->getRootDataElement();
			}

		}
	}
	Object::updateFromXMLNode();
}
