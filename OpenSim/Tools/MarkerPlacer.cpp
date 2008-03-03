// MarkerPlacer.cpp
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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
#include "MarkerPlacer.h"
#include "IKTrial.h"
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmFileWriter.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Common/Storage.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
MarkerPlacer::MarkerPlacer() :
	_apply(_applyProp.getValueBool()),
   _markerFileName(_markerFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
	_ikTaskSetProp(PropertyObj("", IKTaskSet())),
	_ikTaskSet((IKTaskSet&)_ikTaskSetProp.getValueObj()),
	_coordinateFileName(_coordinateFileNameProp.getValueStr()),
	_outputJointFileName(_outputJointFileNameProp.getValueStr()),
	_outputMuscleFileName(_outputMuscleFileNameProp.getValueStr()),
	_outputModelFileName(_outputModelFileNameProp.getValueStr()),
	_outputMarkerFileName(_outputMarkerFileNameProp.getValueStr()),
	_outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
	_maxMarkerMovement(_maxMarkerMovementProp.getValueDbl()),
	_optimizerAlgorithm(_optimizerAlgorithmProp.getValueStr())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
MarkerPlacer::~MarkerPlacer()
{
	delete _ikTrial;
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMarkerPlacer MarkerPlacer to be copied.
 */
MarkerPlacer::MarkerPlacer(const MarkerPlacer &aMarkerPlacer) :
   Object(aMarkerPlacer),
	_apply(_applyProp.getValueBool()),
   _markerFileName(_markerFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
	_ikTaskSetProp(PropertyObj("", IKTaskSet())),
	_ikTaskSet((IKTaskSet&)_ikTaskSetProp.getValueObj()),
	_coordinateFileName(_coordinateFileNameProp.getValueStr()),
	_outputJointFileName(_outputJointFileNameProp.getValueStr()),
	_outputMuscleFileName(_outputMuscleFileNameProp.getValueStr()),
	_outputModelFileName(_outputModelFileNameProp.getValueStr()),
	_outputMarkerFileName(_outputMarkerFileNameProp.getValueStr()),
	_outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
	_maxMarkerMovement(_maxMarkerMovementProp.getValueDbl()),
	_optimizerAlgorithm(_optimizerAlgorithmProp.getValueStr())
{
	setNull();
	setupProperties();
	copyData(aMarkerPlacer);
}

//_____________________________________________________________________________
/**
 * Copy this marker placement params and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this MarkerPlacer.
 */
Object* MarkerPlacer::copy() const
{
	MarkerPlacer *markerPlacementParams = new MarkerPlacer(*this);
	return(markerPlacementParams);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one MarkerPlacer to another.
 *
 * @param aMarkerPlacer MarkerPlacer to be copied.
 */
void MarkerPlacer::copyData(const MarkerPlacer &aMarkerPlacer)
{
	_apply = aMarkerPlacer._apply;
	_markerFileName = aMarkerPlacer._markerFileName;
	_timeRange = aMarkerPlacer._timeRange;
	_ikTaskSet = aMarkerPlacer._ikTaskSet;
	_coordinateFileName = aMarkerPlacer._coordinateFileName;
	_outputJointFileName = aMarkerPlacer._outputJointFileName;
	_outputMuscleFileName = aMarkerPlacer._outputMuscleFileName;
	_outputModelFileName = aMarkerPlacer._outputModelFileName;
	_outputMarkerFileName = aMarkerPlacer._outputMarkerFileName;
	_outputMotionFileName = aMarkerPlacer._outputMotionFileName;
	_maxMarkerMovement = aMarkerPlacer._maxMarkerMovement;
	_optimizerAlgorithm = aMarkerPlacer._optimizerAlgorithm;
	_printResultFiles = aMarkerPlacer._printResultFiles;
}

//_____________________________________________________________________________
/**
 * Set the data members of this MarkerPlacer to their null values.
 */
void MarkerPlacer::setNull()
{
	setType("MarkerPlacer");

	_apply = true;
	_coordinateFileName = "";
	_optimizerAlgorithm = "ipopt";

	_printResultFiles = true;
	_moveModelMarkers = true;
	_ikTrial = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MarkerPlacer::setupProperties()
{
	_applyProp.setComment("Whether or not to use the marker placer during scale");
	_applyProp.setName("apply");
	_propertySet.append(&_applyProp);

	_ikTaskSetProp.setComment("Task set used to specify weights used in the IK computation of the static pose.");
	_ikTaskSetProp.setName("IKTaskSet");
	_propertySet.append(&_ikTaskSetProp);

	_markerFileNameProp.setComment("TRC file (.trc) containing the time history of experimental marker positions "
		"(usually a static trial).");
	_markerFileNameProp.setName("marker_file");
	_propertySet.append(&_markerFileNameProp);

	_coordinateFileNameProp.setComment("Name of file containing the joint angles "
		"used to set the initial configuration of the model for the purpose of placing the markers. "
		"These coordinate values can also be included in the optimization problem used to place the markers. "
		"Before the model markers are placed, a single frame of an inverse kinematics (IK) problem is solved. "
		"The IK problem can be solved simply by matching marker positions, but if the model markers are not "
		"in the correct locations, the IK solution will not be very good and neither will marker placement. "
		"Alternatively, coordinate values (specified in this file) can be specified and used to influence the IK solution. "
		"This is valuable particularly if you have high confidence in the coordinate values. "
		"For example, you know for the static trial the subject was standing will all joint angles close to zero. "
		"If the coordinate set (see the CoordinateSet property) contains non-zero weights for coordinates, "
		"the IK solution will try to match not only the marker positions, but also the coordinates in this file. "
		"Least-squared error is used to solve the IK problem. ");
	_coordinateFileNameProp.setName("coordinate_file");
	_propertySet.append(&_coordinateFileNameProp);

	_timeRangeProp.setComment("Time range over which the marker positions are averaged.");
	const double defaultTimeRange[] = {-1.0, -1.0};
	_timeRangeProp.setName("time_range");
	_timeRangeProp.setValue(2, defaultTimeRange);
	_timeRangeProp.setAllowableArraySize(2);
	_propertySet.append(&_timeRangeProp);

	_outputJointFileNameProp.setComment("Name of the new SIMM Joint file (.jnt) after scaling and marker placement (optional).");
	_outputJointFileNameProp.setName("output_joint_file");
	_propertySet.append(&_outputJointFileNameProp);

	_outputMuscleFileNameProp.setComment("Name of the SIMM muscle file (.msl) after scaling and marker placement (optional).");
	_outputMuscleFileNameProp.setName("output_muscle_file");
	_propertySet.append(&_outputMuscleFileNameProp);

	_outputMotionFileNameProp.setComment("Name of the motion file (.mot) written after marker relocation (optional).");
	_outputMotionFileNameProp.setName("output_motion_file");
	_propertySet.append(&_outputMotionFileNameProp);

	_outputModelFileNameProp.setComment("Output OpenSim model file (.osim) after scaling and maker placement.");
	_outputModelFileNameProp.setName("output_model_file");
	_propertySet.append(&_outputModelFileNameProp);

	_outputMarkerFileNameProp.setComment("Output marker set containing the new marker locations after markers have been placed.");
	_outputMarkerFileNameProp.setName("output_marker_file");
	_propertySet.append(&_outputMarkerFileNameProp);

	_maxMarkerMovementProp.setComment("Maximum amount of movement allowed in marker data when averaging frames of the static trial. "
		"A negative value means there is not limit.");
	_maxMarkerMovementProp.setName("max_marker_movement");
	_maxMarkerMovementProp.setValue(-1.0); // units of this value are the units of the marker data in the static pose (usually mm)
	_propertySet.append(&_maxMarkerMovementProp);

	_optimizerAlgorithmProp.setComment("Preferred optimizer algorithm (currently support \"ipopt\" or \"cfsqp\", "
		"the latter requiring the osimFSQP library.");
	_optimizerAlgorithmProp.setName("optimizer_algorithm");
	_propertySet.append( &_optimizerAlgorithmProp );
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
MarkerPlacer& MarkerPlacer::operator=(const MarkerPlacer &aMarkerPlacer)
{
	// BASE CLASS
	Object::operator=(aMarkerPlacer);

	copyData(aMarkerPlacer);

	return(*this);
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * This method creates a SimmMotionTrial instance with the markerFile and
 * timeRange parameters. It also creates a Storage instance with the
 * coordinateFile parameter. Then it updates the coordinates and markers in
 * the model, if specified. Then it does IK to fit the model to the static
 * pose. Then it uses the current model pose to relocate all non-fixed markers
 * according to their locations in the SimmMotionTrial. Then it writes the
 * output files selected by the user.
 *
 * @param aModel the model to use for the marker placing process.
 * @return Whether the marker placing process was successful or not.
 */
bool MarkerPlacer::processModel(Model* aModel, const string& aPathToSubject)
{
	if(!getApply()) return false;

	cout << endl << "Step 3: Placing markers on model" << endl;

	/* Load the static pose marker file, and average all the
	 * frames in the user-specified time range.
	 */
	MarkerData staticPose(aPathToSubject + _markerFileName);
	staticPose.averageFrames(_maxMarkerMovement, _timeRange[0], _timeRange[1]);
	staticPose.convertToUnits(aModel->getLengthUnits());

	/* Delete any markers from the model that are not in the static
	 * pose marker file.
	 */
	aModel->getDynamicsEngine().deleteUnusedMarkers(staticPose.getMarkerNames());

	delete _ikTrial;
	_ikTrial = new IKTrial();
	if(!_coordinateFileName.empty() && _coordinateFileName!=PropertyStr::getDefaultStr()) _ikTrial->setCoordinateFileName(aPathToSubject + _coordinateFileName);
	_ikTrial->setStartTime(_timeRange[0]);
	_ikTrial->setEndTime(_timeRange[0]);
	_ikTrial->setOptimizerAlgorithm(_optimizerAlgorithm);
	_ikTrial->setPrintResultFiles(false); // We'll do our own printing
//	_ikTrial->setIncludeMarkers(true);
	if(!_ikTrial->initializeTrialCommon(*aModel,_ikTaskSet,staticPose)) return false;
	if(!_ikTrial->solveTrial(*aModel,_ikTaskSet)) return false;
	_ikTrial->getOutputStorage()->setName(_markerFileName);

	/* Now move the non-fixed markers on the model so that they are coincident
	 * with the measured markers in the static pose. The model is already in
	 * the proper configuration so the coordinates do not need to be changed.
	 */
	if(_moveModelMarkers) moveModelMarkersToPose(*aModel, staticPose);

	if(_printResultFiles) {
		/* Write output files, if names specified by the user. */
		SimmFileWriter *sfw = new SimmFileWriter(aModel);
		if (sfw)
		{
			if (!_outputJointFileNameProp.getUseDefault())
				sfw->writeJointFile(aPathToSubject + _outputJointFileName);

			if (!_outputMuscleFileNameProp.getUseDefault())
				sfw->writeMuscleFile(aPathToSubject + _outputMuscleFileName);

			delete sfw;
		}

		if (!_outputModelFileNameProp.getUseDefault())
		{
			aModel->print(aPathToSubject + _outputModelFileName);
			cout << "Wrote model file " << _outputModelFileName << " from model " << aModel->getName() << endl;
		}

		if (!_outputMarkerFileNameProp.getUseDefault())
		{
			aModel->getDynamicsEngine().writeMarkerFile(aPathToSubject + _outputMarkerFileName);
			cout << "Wrote marker file " << _outputMarkerFileName << " from model " << aModel->getName() << endl;
		}

		if (!_outputMotionFileNameProp.getUseDefault())
		{
			Storage motionData(*_ikTrial->getOutputStorage());
			aModel->getDynamicsEngine().convertRadiansToDegrees(motionData);
			motionData.setWriteSIMMHeader(true);
			motionData.setName("static pose");
			motionData.print(aPathToSubject + _outputMotionFileName, 
				"w", "File generated from solving marker data for model "+aModel->getName());
		}
	}

	return true;
}

//_____________________________________________________________________________
/**
 * Set the local offset of each non-fixed marker so that in the model's
 * current pose the marker coincides with the marker's global position
 * in the passed-in MarkerData.
 *
 * @param aModel the model to use
 * @param aPose the static-pose marker cloud to get the marker locations from
 */
void MarkerPlacer::moveModelMarkersToPose(Model& aModel, MarkerData& aPose)
{
	aPose.averageFrames(0.01);
	MarkerFrame* frame = aPose.getFrame(0);

	AbstractDynamicsEngine& engine = aModel.getDynamicsEngine();

	MarkerSet* markerSet = aModel.getDynamicsEngine().getMarkerSet();

	int i;
	for (i = 0; i < markerSet->getSize(); i++)
	{
		AbstractMarker* modelMarker = markerSet->get(i);

		if (!modelMarker->getFixed())
		{
			int index = aPose.getMarkerIndex(modelMarker->getName());
			if (index >= 0)
			{
				SimmPoint& globalMarker = frame->getMarker(index);
				if (globalMarker.isVisible())
				{
					Vec3 pt, pt2;
					Vec3 globalPt = globalMarker.get();
					double conversionFactor = aPose.getUnits().convertTo(aModel.getLengthUnits());
					pt = conversionFactor*globalPt;
					engine.transformPosition(engine.getGroundBody(), pt, *modelMarker->getBody(), pt2);
					modelMarker->setOffset(pt2);
				}
				else
				{
					cout << "___WARNING___: marker " << modelMarker->getName() << " does not have valid coordinates in " << aPose.getFileName() << endl;
					cout << "               It will not be moved to match location in marker file." << endl;
				}
			}
		}
	}

	cout << "Moved markers in model " << aModel.getName() << " to match locations in marker file " << aPose.getFileName() << endl;
}

Storage *MarkerPlacer::getOutputStorage() 
{
	return _ikTrial ? _ikTrial->getOutputStorage() : NULL; 
}
