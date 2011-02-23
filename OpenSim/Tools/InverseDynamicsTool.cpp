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
#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/FunctionSet.h> 
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
InverseDynamicsTool::~InverseDynamicsTool()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
InverseDynamicsTool::InverseDynamicsTool() : DynamicsTool(),
	_coordinatesFileName(_coordinatesFileNameProp.getValueStr()),
	_inDegrees(_inDegreesProp.getValueBool()),
	_lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
	_outputGenForceFileName(_outputGenForceFileNameProp.getValueStr())
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
	_inDegrees(_inDegreesProp.getValueBool()),
	_lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
	_outputGenForceFileName(_outputGenForceFileNameProp.getValueStr())
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
	_inDegrees(_inDegreesProp.getValueBool()),
	_lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
	_outputGenForceFileName(_outputGenForceFileNameProp.getValueStr())
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

	_inDegreesProp.setComment("Flag (true or false) indicating whether coordinates are in degrees or not.");
	_inDegreesProp.setName("coordinates_in_degrees");
	_inDegreesProp.setValue(true);
	_propertySet.append( &_inDegreesProp );

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

	_outputGenForceFileNameProp.setComment("Name of the storage file (.sto) to which the results should be written.");
	_outputGenForceFileNameProp.setName("output_gen_force_file");
	_propertySet.append(&_outputGenForceFileNameProp);
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
	_inDegrees = aTool._inDegrees;
	_lowpassCutoffFrequency = aTool._lowpassCutoffFrequency;
	_outputGenForceFileName = aTool._outputGenForceFileName;


	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================


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
	try{
		//Load and create the indicated model
		_model = new Model(_modelFileName);
		_model->printBasicInfo(cout);

		cout<<"Running tool " << getName() <<".\n"<<endl;

		// Initialize the the model's underlying computational system and get its default state.
		State& s = _model->initSystem();

		// Exclude user-specified forces from the dynamics for this analysis
		disableModelForces(*_model, s, _excludedForces);

		const CoordinateSet &coords = _model->getCoordinateSet();
		int nq = _model->getNumCoordinates();

		FunctionSet *coordFunctions = NULL;
		Storage *coordinateValues = NULL;

		if(_coordinatesFileName != "" && _coordinatesFileName != "Unassigned"){
			coordinateValues = new Storage(_coordinatesFileName);
			if(_lowpassCutoffFrequency>=0) {
				cout<<"\n\nLow-pass filtering coordinates data with a cutoff frequency of "<<_lowpassCutoffFrequency<<"..."<<endl<<endl;
				coordinateValues->pad(coordinateValues->getSize()/2);
				coordinateValues->lowpassIIR(_lowpassCutoffFrequency);
			}
			// Convert degrees to radian if indicated
			if(_inDegrees){
				_model->getSimbodyEngine().convertDegreesToRadians(*coordinateValues);
			}
			// Create differentiable splines of the coordinate data
			coordFunctions = new GCVSplineSet(5, coordinateValues);

			//Functions must correspond to model coordinates and their order for the solver
			for(int i=0; i<nq; i++){
				if(coordFunctions->contains(coords[i].getName())){
					coordFunctions->insert(i,coordFunctions->get(coords[i].getName()));
				}
				else{
					throw Exception("InverseDynamicsTool: coordinate file does not contain coordinate " + coords[i].getName());
				}
			}
			if(coordFunctions->getSize() > nq){
				coordFunctions->setSize(nq);
			}
		}
		else{
			throw Exception("InverseDynamicsTool: no coordinate file found.");
		}

		double first_time = coordinateValues->getFirstTime();
		double last_time = coordinateValues->getLastTime();

		// Determine the starting and final time for the Tool by comparing to what data is available
		double start_time = ( first_time > _timeRange[0]) ? first_time : _timeRange[0];
		double final_time = ( last_time < _timeRange[1]) ? last_time : _timeRange[1];
		int start_index = coordinateValues->findIndex(start_time);
		int final_index = coordinateValues->findIndex(final_time);

		// create the solver given the input data
		InverseDynamicsSolver ivdSolver(*_model);

		const clock_t start = clock();

		int nt = final_index-start_index+1;
		
		Array_<double> times(nt, 0.0);
		for(int i=0; i<nt; i++){
			times[i]=coordinateValues->getStateVector(start_index+i)->getTime();
		}

		// Preallocate results
		Array_<Vector> genForceTraj(nt, Vector(nq, 0.0));

		// solve for the trajectory of generlized forces that correspond to the coordinate
		// trajectories provided
		ivdSolver.solve(s, *coordFunctions, times, genForceTraj);

		success = true;

		cout << "InverseDynamicsTool: " << nt << " time frames in " <<(double)(clock()-start)/CLOCKS_PER_SEC << "s\n" <<endl;
	
		Storage ivdResults(nt);
		for(int i=0; i<nt; i++){
			StateVector *dataVec = new StateVector(times[i], nq, &((genForceTraj[i])[0]));
			ivdResults.append(*dataVec);
		}

		Array<string> labels("time", nq+1);
		for(int i=0; i<nq; i++){
			labels[i+1] = coords[i].getName();
			labels[i+1] += (coords[i].getMotionType() == Coordinate::Rotational) ? "_moment" : "_force";
		}

		ivdResults.setColumnLabels(labels);
		ivdResults.setName("Inverse Dynamics");
		//ivdResults.print(_outputGenForceFileName);
		IO::makeDir(getResultsDir());
		Storage::printResult(&ivdResults, _outputGenForceFileName, getResultsDir(), -1, ".sto");
	}
	catch (std::exception ex) {
		std::cout << "InverseDynamicsTool Failed: " << ex.what() << std::endl;
	}

	delete _model;

	return success;
}
/* Handle reading older formats/Versioning */
void InverseDynamicsTool::updateFromXMLNode()
{
	int documentVersion = getDocument()->getDocumentVersion();
	if ( documentVersion < XMLDocument::getLatestVersion()){
		if (documentVersion <= 20202){
			// get filename and use SimTK::Xml to parse it
			SimTK::Xml::Document doc = SimTK::Xml::Document(getDocumentFileName());
			Xml::Element root = doc.getRootElement();
			SimTK::Xml::Document newDoc;
			if (root.getElementTag()=="AnalyzeTool"){
				// Make OpenSimDocument node and copy root underneath it
				newDoc.getRootElement().setElementTag("OpenSimDocument");
				newDoc.getRootElement().setAttributeValue("Version", "20201");
				// Move all children of root to toolNode
				newDoc.getRootElement().insertNodeAfter(newDoc.getRootElement().node_end(), root.clone());
				//root.insertNodeAfter(root.element_end(), toolNode);
				newDoc.writeToFile(getDocumentFileName());
				root = newDoc.getRootElement();
				cout << root.getElementTag() << endl;
			}
			if (root.getElementTag()=="OpenSimDocument"){
				int curVersion = root.getRequiredAttributeValueAs<int>("Version");
				if (curVersion <= 20201) root.setAttributeValue("Version", "20202");
				Xml::element_iterator iterTool(root.element_begin("AnalyzeTool"));
				iterTool->setElementTag("InverseDynamicsTool");
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
					iterTool->insertNodeAfter( iterTool->node_end(), Xml::Element("forces_to_exclude", use_model_forces?"":"muscles"));
					iterTool->insertNodeAfter( iterTool->node_end(), Xml::Element("output_gen_force_file", "_InverseDynamics"));
					iterTool->insertNodeAfter( iterTool->node_end(), Xml::Element("coordinates_in_degrees", "false"));
					iterTool->eraseNode(iterAnalysisSet);
				}
				doc.writeToFile("_temp.xml");
				*this=InverseDynamicsTool("_temp.xml");
				return;
			}

		}
	}
	DynamicsTool::updateFromXMLNode();
}
