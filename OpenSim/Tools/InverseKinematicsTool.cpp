// InverseKinematicsTool.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2006 Stanford University
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
#include "InverseKinematicsTool.h"
#include <string>
#include <iostream>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/CoordinateReference.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>

#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/XMLDocument.h>

#include <OpenSim/Analyses/Kinematics.h>

#include "IKTaskSet.h"
#include "IKCoordinateTask.h"
#include "IKMarkerTask.h"

#include "SimTKsimbody.h"


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
InverseKinematicsTool::~InverseKinematicsTool()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
InverseKinematicsTool::InverseKinematicsTool() : Tool(),
	_modelFileName(_modelFileNameProp.getValueStr()),
	_constraintWeight(_constraintWeightProp.getValueDbl()),
	_accuracy(_accuracyProp.getValueDbl()),
	_ikTaskSetProp(PropertyObj("", IKTaskSet())),
	_ikTaskSet((IKTaskSet&)_ikTaskSetProp.getValueObj()),
	_markerFileName(_markerFileNameProp.getValueStr()),
	_coordinateFileName(_coordinateFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
	_reportErrors(_reportErrorsProp.getValueBool()),
	_outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
	_reportMarkerLocations(_reportMarkerLocationsProp.getValueBool())
{
	setType("InverseKinematicsTool");
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
InverseKinematicsTool::InverseKinematicsTool(const string &aFileName, bool aLoadModel) :
	Tool(aFileName, false),
	_modelFileName(_modelFileNameProp.getValueStr()),
	_constraintWeight(_constraintWeightProp.getValueDbl()),
	_accuracy(_accuracyProp.getValueDbl()),
	_ikTaskSetProp(PropertyObj("", IKTaskSet())),
	_ikTaskSet((IKTaskSet&)_ikTaskSetProp.getValueObj()),
	_markerFileName(_markerFileNameProp.getValueStr()),
	_coordinateFileName(_coordinateFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
	_reportErrors(_reportErrorsProp.getValueBool()),
	_outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
	_reportMarkerLocations(_reportMarkerLocationsProp.getValueBool())
{
	setType("InverseKinematicsTool");
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
InverseKinematicsTool::InverseKinematicsTool(const InverseKinematicsTool &aTool) :
	Tool(aTool),
	_modelFileName(_modelFileNameProp.getValueStr()),
	_constraintWeight(_constraintWeightProp.getValueDbl()),
	_accuracy(_accuracyProp.getValueDbl()),
	_ikTaskSetProp(PropertyObj("", IKTaskSet())),
	_ikTaskSet((IKTaskSet&)_ikTaskSetProp.getValueObj()),
	_markerFileName(_markerFileNameProp.getValueStr()),
	_coordinateFileName(_coordinateFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
	_reportErrors(_reportErrorsProp.getValueBool()),
	_outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
	_reportMarkerLocations(_reportMarkerLocationsProp.getValueBool())
{
	setType("InverseKinematicsTool");
	setNull();
	*this = aTool;
}

//_____________________________________________________________________________
/**
 * Virtual copy constructor.
 */
Object* InverseKinematicsTool::copy() const
{
	InverseKinematicsTool *object = new InverseKinematicsTool(*this);
	return(object);
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void InverseKinematicsTool::setNull()
{
	setupProperties();
	_model = NULL;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void InverseKinematicsTool::setupProperties()
{
	_modelFileNameProp.setComment("Name of the .osim file used to construct a model.");
	_modelFileNameProp.setName("model_file");
	_propertySet.append( &_modelFileNameProp );

	_constraintWeightProp.setComment("A positive scalar that is used to weight the importance of satisfying constraints."
		"A weighting of 'Infinity' or if it is unassigned results in the constraints being strictly enforced.");
	_constraintWeightProp.setName("constraint_weight");
	_constraintWeightProp.setValue(std::numeric_limits<SimTK::Real>::infinity());
	_propertySet.append( &_constraintWeightProp );

	_accuracyProp.setComment("The accuracy of the solution in absolute terms. I.e. the number of significant"
	    "digits to which the solution can be trusted.");
	_accuracyProp.setName("accuracy");
	_accuracyProp.setValue(1e-5);
	_propertySet.append( &_accuracyProp );

	_ikTaskSetProp.setComment("Markers and coordinates to be considered (tasks) and their weightings.");
	_ikTaskSetProp.setName("IKTaskSet");
	_propertySet.append(&_ikTaskSetProp);

	_markerFileNameProp.setComment("TRC file (.trc) containing the time history of observations of marker positions.");
	_markerFileNameProp.setName("marker_file");
	_propertySet.append(&_markerFileNameProp);

	_coordinateFileNameProp.setComment("The name of the storage (.sto or .mot) file containing coordinate observations."
		"Coordinate values from this file are included if there is a corresponding coordinate task. ");
	_coordinateFileNameProp.setName("coordinate_file");
	_propertySet.append(&_coordinateFileNameProp);

	const double defaultTimeRange[] = {-std::numeric_limits<SimTK::Real>::infinity(), std::numeric_limits<SimTK::Real>::infinity()};
	_timeRangeProp.setComment("Time range over which the inverse kinematics problem is solved.");
	_timeRangeProp.setName("time_range");
	_timeRangeProp.setValue(2, defaultTimeRange);
	_timeRangeProp.setAllowableArraySize(2);
	_propertySet.append(&_timeRangeProp);

	_reportErrorsProp.setComment("Flag (true or false) indicating whether or not to report marker "
		"errors from the inverse kinematics solution.");
	_reportErrorsProp.setName("report_errors");
	_reportErrorsProp.setValue(true);
	_propertySet.append(&_reportErrorsProp);

	_outputMotionFileNameProp.setComment("Name of the motion file (.mot) to which the results should be written.");
	_outputMotionFileNameProp.setName("output_motion_file");
	_propertySet.append(&_outputMotionFileNameProp);

	_reportMarkerLocationsProp.setComment("Flag indicating whether or not to report model marker locations in ground.");
	_reportMarkerLocationsProp.setName("report_marker_locations");
	_reportMarkerLocationsProp.setValue(false);
	_propertySet.append(&_reportMarkerLocationsProp);

}

//_____________________________________________________________________________
/**
 * Register InverseKinematicsTool and any Object types it may employ internally.
 */
void InverseKinematicsTool::registerTypes()
{
	Object::RegisterType(InverseKinematicsTool());
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
InverseKinematicsTool& InverseKinematicsTool::
operator=(const InverseKinematicsTool &aTool)
{
	// BASE CLASS
	Tool::operator=(aTool);

	// MEMBER VARIABLES
	_modelFileName = aTool._modelFileName;
	_constraintWeight = aTool._constraintWeight;
	_accuracy = aTool._accuracy;
	_ikTaskSet = aTool._ikTaskSet;
	_markerFileName = aTool._markerFileName;
	_timeRange = aTool._timeRange;
	_reportErrors = aTool._reportErrors; 
	_coordinateFileName = aTool._coordinateFileName;
	_reportErrors = aTool._reportErrors;
	_outputMotionFileName = aTool._outputMotionFileName;
	_reportMarkerLocations = aTool._reportMarkerLocations;

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
 * Run the inverse kinematics tool.
 */
bool InverseKinematicsTool::run()
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


		// Do the maneuver to change then restore working directory 
		// so that the parsing code behaves properly if called from a different directory.
		string saveWorkingDirectory = IO::getCwd();
		string directoryOfSetupFile = IO::getParentDirectory(getDocumentFileName());
		IO::chDir(directoryOfSetupFile);

		// Define reporter for output
		Kinematics kinematicsReporter(_model);
		kinematicsReporter.setRecordAccelerations(false);
		kinematicsReporter.setInDegrees(true);

		cout<<"Running tool "<<getName()<<".\n";

		// Initialize the the model's underlying computational system and get its default state.
		SimTK::State& s = modelFromFile?_model->initSystem(): _model->updMultibodySystem().updDefaultState();

		//Convert old Tasks to references for assembly and tracking
		MarkersReference markersReference;
		Set<MarkerWeight> markerWeights;
		SimTK::Array_<CoordinateReference> coordinateReferences;

		FunctionSet *coordFunctions = NULL;
		// Load the coordinate data
		bool haveCoordinateFile = false;
		if(_coordinateFileName != "" && _coordinateFileName != "Unassigned"){
			Storage coordinateValues(_coordinateFileName);
			// Convert degrees to radian (TODO: this needs to have a check that the storage is infact in degrees!)
			_model->getSimbodyEngine().convertDegreesToRadians(coordinateValues);
			haveCoordinateFile = true;
			coordFunctions = new GCVSplineSet(5,&coordinateValues);
		}

		// Loop through old "IKTaskSet" and assign weights to the coordinate and marker references
		// For coordinates, create the functions for coordinate reference values
		int index = 0;
		for(int i=0; i < _ikTaskSet.getSize(); i++){
			if (!_ikTaskSet[i].getApply()) continue;
			if(IKCoordinateTask *coordTask = dynamic_cast<IKCoordinateTask *>(&_ikTaskSet[i])){
				CoordinateReference *coordRef = NULL;
				if(coordTask->getValueType() == IKCoordinateTask::FromFile){
					 if (!coordFunctions)
						throw Exception("InverseKinematicsTool: value for coordinate "+coordTask->getName()+" not found.");

					 index = coordFunctions->getIndex(coordTask->getName(), index);
					 if(index >= 0){
						 coordRef = new CoordinateReference(coordTask->getName(),coordFunctions->get(index));
					 }
				}
				else if((coordTask->getValueType() == IKCoordinateTask::ManualValue)){
                        Constant reference(Constant(coordTask->getValue()));
						coordRef = new CoordinateReference(coordTask->getName(), reference);
				}
				else{ // assume it should be held at its current/default value
					double value = _model->getCoordinateSet().get(coordTask->getName()).getValue(s);
					Constant reference = Constant(value);
					coordRef = new CoordinateReference(coordTask->getName(), reference);
				}

				if(coordRef == NULL)
					throw Exception("InverseKinematicsTool: value for coordinate "+coordTask->getName()+" not found.");
				else
					coordRef->setWeight(coordTask->getWeight());

				coordinateReferences.push_back(*coordRef);
			}
			else if(IKMarkerTask *markerTask = dynamic_cast<IKMarkerTask *>(&_ikTaskSet[i])){
				MarkerWeight *markerWeight = new MarkerWeight(markerTask->getName(), markerTask->getWeight());
				markerWeights.append(markerWeight);
			}
		}

		//Set the weights for markers
		markersReference.setMarkerWeightSet(markerWeights);
		//Load the makers
		markersReference.loadMarkersFile(_markerFileName);
		// marker names
		const SimTK::Array_<std::string>& markerNames =  markersReference.getNames();

		// Determine the start time, if the provided time range is not specified then use time from marker reference
		// also adjust the time range for the tool if the provided range exceeds that of the marker data
		SimTK::Vec2 markersValidTimRange = markersReference.getValidTimeRange();
		double start_time = (markersValidTimRange[0] > _timeRange[0]) ? markersValidTimRange[0] : _timeRange[0];
		double final_time = (markersValidTimRange[1] < _timeRange[1]) ? markersValidTimRange[1] : _timeRange[1];

		// create the solver given the input data
		InverseKinematicsSolver ikSolver(*_model, markersReference, coordinateReferences, _constraintWeight);
		ikSolver.setAccuracy(_accuracy);
		s.updTime() = start_time;
		ikSolver.assemble(s);
		kinematicsReporter.begin(s);

		const clock_t start = clock();
		double dt = 1.0/markersReference.getSamplingFrequency();
		int Nframes = int((final_time-start_time)/dt)+1;
		AnalysisSet& analysisSet = _model->updAnalysisSet();
		analysisSet.begin(s);
		// number of markers
		int nm = markerWeights.getSize();
		SimTK::Array_<double> squaredMarkerErrors(nm, 0.0);
		SimTK::Array_<Vec3> markerLocations(nm, Vec3(0));
		
		Storage *modelMarkerLocations = _reportMarkerLocations ? new Storage(Nframes, "ModelMarkerLocations") : NULL;

		for (int i = 1; i < Nframes; i++) {
			s.updTime() = start_time + i*dt;
			ikSolver.track(s);
			
			if(_reportErrors){
				double totalSquaredMarkerError = 0.0;
				double maxSquaredMarkerError = 0.0;
				int worst = -1;

				ikSolver.computeCurrentSquaredMarkerErrors(squaredMarkerErrors);
				for(int j=0; j<nm; ++j){
					totalSquaredMarkerError += squaredMarkerErrors[j];
					if(squaredMarkerErrors[j] > maxSquaredMarkerError){
						maxSquaredMarkerError = squaredMarkerErrors[j];
						worst = j;
					}
				}
				cout << "Frame " << i << " (t=" << s.getTime() << "):\t";
				cout << "total squared error = " << totalSquaredMarkerError;
				cout << ", marker error: RMS=" << sqrt(totalSquaredMarkerError/nm);
				cout << ", max=" << sqrt(maxSquaredMarkerError) << " (" << markerNames[worst] << ")" << endl;
			}

			if(_reportMarkerLocations){
				ikSolver.computeCurrentMarkerLocations(markerLocations);
				Array<double> locations(0.0, 3*nm);
				for(int j=0; j<nm; ++j){
					for(int k=0; k<3; ++k)
						locations.set(3*j+k, markerLocations[j][k]);
				}

				modelMarkerLocations->append(s.getTime(), 3*nm, &locations[0]);

			}

			kinematicsReporter.step(s, i);
			analysisSet.step(s, i);
		}

		// Do the maneuver to change then restore working directory 
		// so that output files are saved to same folder as setup file.
		if (_outputMotionFileName!= "" && _outputMotionFileName!="Unassigned"){
			kinematicsReporter.getPositionStorage()->print(_outputMotionFileName);
		}

		if(modelMarkerLocations){
			Array<string> labels("", 3*nm+1);
			labels[0] = "time";
			Array<string> XYZ("", 3*nm);
			XYZ[0] = "_tx"; XYZ[1] = "_ty"; XYZ[2] = "_tz";

			for(int j=0; j<nm; ++j){
				for(int k=0; k<3; ++k)
					labels.set(3*j+k+1, markerNames[j]+XYZ[k]);
			}
			modelMarkerLocations->setColumnLabels(labels);
			modelMarkerLocations->setName("Model Marker Locations from IK");
	
			IO::makeDir(getResultsDir());
			Storage::printResult(modelMarkerLocations, "ik_model_marker_locations", getResultsDir(), -1, ".sto");

			delete modelMarkerLocations;
		}

		IO::chDir(saveWorkingDirectory);

		success = true;

		cout << "InverseKinematicsTool completed " << Nframes-1 << " frames in " <<(double)(clock()-start)/CLOCKS_PER_SEC << "s\n" <<endl;
	}
	catch (std::exception ex) {
		std::cout << "InverseKinematicsTool Failed: " << ex.what() << std::endl;
		throw (Exception("InverseKinematicsTool Failed, please see messages window for details..."));
	}

	if (modelFromFile) delete _model;

	return success;
}

// Handle conversion from older format
void InverseKinematicsTool::updateFromXMLNode()
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
		if (documentVersion <= 20201){
			// get filename and use SimTK::Xml to parse it
			SimTK::Xml doc = SimTK::Xml(newFileName);
			Xml::Element root = doc.getRootElement();
			if (root.getElementTag()=="OpenSimDocument"){
				int curVersion = root.getRequiredAttributeValueAs<int>("Version");
				if (curVersion <= 20201) root.setAttributeValue("Version", "20300");
				Xml::element_iterator iter(root.element_begin("IKTool"));
				iter->setElementTag("InverseKinemtaticsTool");
				Xml::element_iterator toolIter(iter->element_begin("IKTrialSet"));
				// No optimizer_algorithm specification anymore
				Xml::element_iterator optIter(iter->element_begin("optimizer_algorithm"));
				if (optIter!= iter->element_end())
					iter->eraseNode(optIter);

				Xml::element_iterator objIter(toolIter->element_begin("objects")); 
				Xml::element_iterator trialIter(objIter->element_begin("IKTrial")); 
				// Move children of (*trialIter) to root
				Xml::node_iterator p = trialIter->node_begin();
				for (; p!= trialIter->node_end(); ++p) {
					iter->insertNodeAfter( iter->node_end(), p->clone());
			}
				// Append constraint_weight of 100 and accuracy of 1e-5
				iter->insertNodeAfter( iter->node_end(), Xml::Comment(_constraintWeightProp.getComment()));
				iter->insertNodeAfter( iter->node_end(), Xml::Element("constraint_weight", "20.0"));
				iter->insertNodeAfter( iter->node_end(), Xml::Comment(_accuracyProp.getComment()));
				iter->insertNodeAfter( iter->node_end(), Xml::Element("accuracy", "1e-4"));
				// erase node for IKTrialSet
				iter->eraseNode(toolIter);	
				Xml::Document newDocument;
				Xml::Element docElement= newDocument.getRootElement();
				docElement.setAttributeValue("Version", "20300");
				docElement.setElementTag("OpenSimDocument");
				// Copy all children of root to newRoot
				docElement.insertNodeAfter(docElement.node_end(), iter->clone());
				newDocument.writeToFile(newFileName);
				_document = new XMLDocument(newFileName);
				_node = _document->getRootDataElement();
			}
			else { 
				if (root.getElementTag()=="IKTool"){
					root.setElementTag("InverseKinemtaticsTool");
					Xml::element_iterator toolIter(root.element_begin("IKTrialSet"));
					if (toolIter== root.element_end())
						throw (Exception("Old IKTool setup file doesn't have required IKTrialSet element.. Aborting"));
					// No optimizer_algorithm specification anymore
					Xml::element_iterator optIter(root.element_begin("optimizer_algorithm"));
					if (optIter!= root.element_end())
					root.eraseNode(optIter);

					Xml::element_iterator objIter(toolIter->element_begin("objects")); 
					Xml::element_iterator trialIter(objIter->element_begin("IKTrial")); 
					// Move children of (*trialIter) to root
					Xml::node_iterator p = trialIter->node_begin();
					for (; p!= trialIter->node_end(); ++p) {
						root.insertNodeAfter( root.node_end(), p->clone());
					}
					// Append constraint_weight of 100 and accuracy of 1e-5
					root.insertNodeAfter( root.node_end(), Xml::Comment(_constraintWeightProp.getComment()));
					root.insertNodeAfter( root.node_end(), Xml::Element("constraint_weight", "20.0"));
					root.insertNodeAfter( root.node_end(), Xml::Comment(_accuracyProp.getComment()));
					root.insertNodeAfter( root.node_end(), Xml::Element("accuracy", "1e-5"));
					// erase node for IKTrialSet
					root.eraseNode(toolIter);
					
					// Create an OpenSimDocument node and move root inside it
					Xml::Document newDocument;
					Xml::Element docElement= newDocument.getRootElement();
					docElement.setAttributeValue("Version", "20300");
					docElement.setElementTag("OpenSimDocument");
					// Copy all children of root to newRoot
					docElement.insertNodeAfter(docElement.node_end(), doc.getRootElement().clone());
					newDocument.writeToFile(newFileName);
					_document = new XMLDocument(newFileName);
					_node = _document->getRootDataElement();
				}
				else
				;	// Somthing wrong! bail out
			}
		}
	}
	Object::updateFromXMLNode();
}