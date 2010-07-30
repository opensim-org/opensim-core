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

#include <OpenSim/Analyses/Kinematics.h>

#include "IKTaskSet.h"
#include "IKCoordinateTask.h"
#include "IKMarkerTask.h"

#include "SimTKsimbody.h"


using namespace OpenSim;
using namespace std;


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
	_outputMotionFileName(_outputMotionFileNameProp.getValueStr())
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
	_outputMotionFileName(_outputMotionFileNameProp.getValueStr())
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
	_outputMotionFileName(_outputMotionFileNameProp.getValueStr())
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
	_constraintWeightProp.setValue(SimTK::Infinity);
	_propertySet.append( &_constraintWeightProp );

	_accuracyProp.setComment("The accuracy of the solution in absolute terms. I.e. the number of significant"
	    "digits to which the solution can be trusted.");
	_accuracyProp.setName("accuracy");
	_accuracyProp.setValue(1e-3);
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

	const double defaultTimeRange[] = {-SimTK::Infinity, SimTK::Infinity};
	_timeRangeProp.setComment("Time range over which the inverse kinematics problem is solved.");
	_timeRangeProp.setName("time_range");
	_timeRangeProp.setValue(2, defaultTimeRange);
	_timeRangeProp.setAllowableArraySize(2);
	_propertySet.append(&_timeRangeProp);

	_reportErrorsProp.setComment("Flag (true or false) indicating whether or not to report marker "
		"and coordinate errors from the inverse kinematics solution.");
	_reportErrorsProp.setName("report_errors");
	_reportErrorsProp.setValue(false);
	_propertySet.append(&_reportErrorsProp);

	_outputMotionFileNameProp.setComment("Name of the motion file (.mot) to which the results should be written.");
	_outputMotionFileNameProp.setName("output_motion_file");
	_propertySet.append(&_outputMotionFileNameProp);

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
	_coordinateFileName = aTool._coordinateFileName;
	_outputMotionFileName = aTool._outputMotionFileName;

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
	try{
		//Load and create the indicated model
		_model = new Model(_modelFileName);
		_model->printBasicInfo(cout);

		// Define reporter for output
		Kinematics kinematicsReporter(_model);
		kinematicsReporter.setRecordAccelerations(false);
		kinematicsReporter.setInDegrees(true);

		cout<<"Running tool "<<getName()<<".\n";

		// Initialize the the model's underlying computational system and get its default state.
		SimTK::State& s = _model->initSystem();

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
			if(IKCoordinateTask *coordTask = dynamic_cast<IKCoordinateTask *>(&_ikTaskSet[i])){
				CoordinateReference *coordRef = NULL;
				if(coordTask->getValueType() == IKCoordinateTask::FromFile){
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

		// Determine the start time, if the provided time range is not specified then use time from marker reference
		// also adjust the time range for the tool if the provided range exceed that of the marker data
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
		for (int i = 1; i < Nframes; i++) {
			s.updTime() = start_time + i*dt;
			// update solver state to match new observed locations
			ikSolver.track(s);
			kinematicsReporter.step(s, i);
		}

		kinematicsReporter.printResults(_outputMotionFileName);

		success = true;

		cout << "InverseKinematicsTool: " << Nframes-1 << " frames in " <<(double)(clock()-start)/CLOCKS_PER_SEC << "s\n" <<endl;
	}
	catch (std::exception ex) {
		std::cout << "InverseKinematicsTool Failed: " << ex.what() << std::endl;
	}

	delete _model;

	return success;
}
