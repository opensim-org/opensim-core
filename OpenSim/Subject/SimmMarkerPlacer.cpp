// SimmMarkerPlacer.cpp
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include "SimmMarkerPlacer.h"
#include "SimmIKTrial.h"
#include "SimmFileWriter.h"
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerData.h>
#include <OpenSim/Simulation/SIMM/SimmMotionData.h>
#include <OpenSim/Simulation/SIMM/AbstractBody.h>
#include <OpenSim/Applications/IK/SimmIKSolverImpl.h>
#include <OpenSim/Applications/IK/SimmInverseKinematicsTarget.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmMarkerPlacer::SimmMarkerPlacer() :
   _markerFileName(_markerFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
   _coordinateFileName(_coordinateFileNameProp.getValueStr()),
	_coordinateSetProp(PropertyObj("", CoordinateSet())),
	_coordinateSet((CoordinateSet&)_coordinateSetProp.getValueObj()),
	_coordinatesFromFile(_coordinatesFromFileProp.getValueStrArray()),
	_markerSetProp(PropertyObj("", MarkerSet())),
	_markerSet((MarkerSet&)_markerSetProp.getValueObj()),
	_outputJointFileName(_outputJointFileNameProp.getValueStr()),
	_outputMuscleFileName(_outputMuscleFileNameProp.getValueStr()),
	_outputModelFileName(_outputModelFileNameProp.getValueStr()),
	_outputMarkerFileName(_outputMarkerFileNameProp.getValueStr()),
	_outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
	_maxMarkerMovement(_maxMarkerMovementProp.getValueDbl())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmMarkerPlacer::SimmMarkerPlacer(DOMElement *aElement) :
   Object(aElement),
   _markerFileName(_markerFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
   _coordinateFileName(_coordinateFileNameProp.getValueStr()),
	_coordinateSetProp(PropertyObj("", CoordinateSet())),
	_coordinateSet((CoordinateSet&)_coordinateSetProp.getValueObj()),
	_coordinatesFromFile(_coordinatesFromFileProp.getValueStrArray()),
	_markerSetProp(PropertyObj("", MarkerSet())),
	_markerSet((MarkerSet&)_markerSetProp.getValueObj()),
	_outputJointFileName(_outputJointFileNameProp.getValueStr()),
	_outputMuscleFileName(_outputMuscleFileNameProp.getValueStr()),
	_outputModelFileName(_outputModelFileNameProp.getValueStr()),
	_outputMarkerFileName(_outputMarkerFileNameProp.getValueStr()),
	_outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
	_maxMarkerMovement(_maxMarkerMovementProp.getValueDbl())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmMarkerPlacer::~SimmMarkerPlacer()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMarkerPlacer SimmMarkerPlacer to be copied.
 */
SimmMarkerPlacer::SimmMarkerPlacer(const SimmMarkerPlacer &aMarkerPlacer) :
   Object(aMarkerPlacer),
   _markerFileName(_markerFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
   _coordinateFileName(_coordinateFileNameProp.getValueStr()),
	_coordinateSetProp(PropertyObj("", CoordinateSet())),
	_coordinateSet((CoordinateSet&)_coordinateSetProp.getValueObj()),
	_coordinatesFromFile(_coordinatesFromFileProp.getValueStrArray()),
	_markerSetProp(PropertyObj("", MarkerSet())),
	_markerSet((MarkerSet&)_markerSetProp.getValueObj()),
	_outputJointFileName(_outputJointFileNameProp.getValueStr()),
	_outputMuscleFileName(_outputMuscleFileNameProp.getValueStr()),
	_outputModelFileName(_outputModelFileNameProp.getValueStr()),
	_outputMarkerFileName(_outputMarkerFileNameProp.getValueStr()),
	_outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
	_maxMarkerMovement(_maxMarkerMovementProp.getValueDbl())
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
 * @return Pointer to a copy of this SimmMarkerPlacer.
 */
Object* SimmMarkerPlacer::copy() const
{
	SimmMarkerPlacer *markerPlacementParams = new SimmMarkerPlacer(*this);
	return(markerPlacementParams);
}

//_____________________________________________________________________________
/**
 * Copy this SimmMarkerPlacer and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmMarkerPlacer::SimmMarkerPlacer(DOMElement*) in order to establish the
 * relationship of the SimmMarkerPlacer object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmMarkerPlacer object. Finally, the data members of the
 * copy are updated using SimmMarkerPlacer::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmMarkerPlacer.
 */
Object* SimmMarkerPlacer::copy(DOMElement *aElement) const
{
	SimmMarkerPlacer *markerPlacementParams = new SimmMarkerPlacer(aElement);
	*markerPlacementParams = *this;
	markerPlacementParams->updateFromXMLNode();
	return(markerPlacementParams);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimmMarkerPlacer to another.
 *
 * @param aMarkerPlacer SimmMarkerPlacer to be copied.
 */
void SimmMarkerPlacer::copyData(const SimmMarkerPlacer &aMarkerPlacer)
{
	_markerFileName = aMarkerPlacer._markerFileName;
	_timeRange = aMarkerPlacer._timeRange;
	_coordinateFileName = aMarkerPlacer._coordinateFileName;
	_coordinateSet = aMarkerPlacer._coordinateSet;
	_coordinatesFromFile = aMarkerPlacer._coordinatesFromFile;
	_markerSet = aMarkerPlacer._markerSet;
	_outputJointFileName = aMarkerPlacer._outputJointFileName;
	_outputMuscleFileName = aMarkerPlacer._outputMuscleFileName;
	_outputModelFileName = aMarkerPlacer._outputModelFileName;
	_outputMarkerFileName = aMarkerPlacer._outputMarkerFileName;
	_outputMotionFileName = aMarkerPlacer._outputMotionFileName;
	_maxMarkerMovement = aMarkerPlacer._maxMarkerMovement;
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimmMarkerPlacer to their null values.
 */
void SimmMarkerPlacer::setNull()
{
	setType("SimmMarkerPlacer");

	_coordinateFileName = "";
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmMarkerPlacer::setupProperties()
{
	_markerFileNameProp.setName("marker_file");
	_propertySet.append(&_markerFileNameProp);

	const double defaultTimeRange[] = {-1.0, -1.0};
	_timeRangeProp.setName("time_range");
	_timeRangeProp.setComment("Time range for averaging marker positions.");
	_timeRangeProp.setValue(2, defaultTimeRange);
	_propertySet.append(&_timeRangeProp);

	_coordinateFileNameProp.setName("coordinate_file");
	_coordinateFileNameProp.setComment("Name of file containing the joint angles used to set the initial configuration of the model for the purpose of relocating the non-fixed markers.");
	_propertySet.append(&_coordinateFileNameProp);

	_markerSetProp.setName("MarkerSet");
	_propertySet.append(&_markerSetProp);

	_coordinateSetProp.setName("CoordinateSet");
	_propertySet.append(&_coordinateSetProp);

	_coordinatesFromFileProp.setName("coordinates_from_file");
	Array<string> def("");
	_coordinatesFromFileProp.setValue(def);
	_propertySet.append(&_coordinatesFromFileProp);

	_outputJointFileNameProp.setName("output_joint_file");
	_outputJointFileNameProp.setComment("Name of the new SIMM Joint file after scaling and marker relocation.");
	_propertySet.append(&_outputJointFileNameProp);

	_outputMuscleFileNameProp.setName("output_muscle_file");
	_outputMuscleFileNameProp.setComment("Name of the SIMM muscle file after scaling and marker relocation.");
	_propertySet.append(&_outputMuscleFileNameProp);

	_outputModelFileNameProp.setName("output_model_file");
	_outputModelFileNameProp.setComment("Name of the new OpenSim model file after scaling and maker relocation.");
	_propertySet.append(&_outputModelFileNameProp);

	_outputMarkerFileNameProp.setName("output_marker_file");
	_outputMarkerFileNameProp.setComment("Name of the xml file containing the marker set after the non-fixed markers have been moved.");
	_propertySet.append(&_outputMarkerFileNameProp);

	_outputMotionFileNameProp.setName("output_motion_file");
	_outputMotionFileNameProp.setComment("Name of the motion file written after marker relocation.");
	_propertySet.append(&_outputMotionFileNameProp);

	_maxMarkerMovementProp.setName("max_marker_movement");
	_maxMarkerMovementProp.setValue(-1.0); // units of this value are the units of the marker data in the static pose (usually mm)
	_maxMarkerMovementProp.setComment("Maximum amount of movement allowed in marker data when averaging frames of the static trial.");
	_propertySet.append(&_maxMarkerMovementProp);
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
SimmMarkerPlacer& SimmMarkerPlacer::operator=(const SimmMarkerPlacer &aMarkerPlacer)
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
 * timeRange parameters. It also creates a SimmMotionData instance with the
 * coordinateFile parameter. Then it updates the coordinates and markers in
 * the model, if specified. Then it does IK to fit the model to the static
 * pose. Then it uses the current model pose to relocate all non-fixed markers
 * according to their locations in the SimmMotionTrial. Then it writes the
 * output files selected by the user.
 *
 * @param aModel the model to use for the marker placing process.
 * @return Whether the marker placing process was successful or not.
 */
bool SimmMarkerPlacer::processModel(AbstractModel* aModel, const string& aPathToSubject)
{
	cout << endl << "Step 3: Placing markers on model" << endl;

	/* Update the model with the markers specified in the params section. */
	aModel->getDynamicsEngine().updateMarkerSet(_markerSet);

	/* Load the static pose marker file, and average all the
	 * frames in the user-specified time range.
	 */
	SimmMarkerData staticPose(aPathToSubject + _markerFileName);
	staticPose.averageFrames(_maxMarkerMovement, _timeRange[0], _timeRange[1]);
	staticPose.convertToUnits(aModel->getLengthUnits());

	/* Delete any markers from the model that are not in the static
	 * pose marker file.
	 */
	aModel->getDynamicsEngine().deleteUnusedMarkers(staticPose.getMarkerNames());

	Storage outputStorage;
	outputStorage.setName(_markerFileName);

	SimmIKTrial ikTrial;
	if(_coordinateFileName != "") ikTrial.setCoordinateFileName(aPathToSubject + _coordinateFileName);
	ikTrial.setStartTime(_timeRange[0]);
	ikTrial.setEndTime(_timeRange[0]);
	ikTrial.setIncludeMarkers(true);
	if(!ikTrial.processTrialCommon(*aModel,_coordinateSet,_coordinatesFromFile,staticPose,outputStorage)) 
		return false;

	/* Now move the non-fixed markers on the model so that they are coincident
	 * with the measured markers in the static pose. The model is already in
	 * the proper configuration so the coordinates do not need to be changed.
	 */
	moveModelMarkersToPose(*aModel, staticPose);

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
		SimmMotionData motionData(outputStorage);
		motionData.convertRadiansToDegrees(*aModel);
		motionData.writeSIMMMotionFile(aPathToSubject + _outputMotionFileName, aModel->getName());
	}

	return true;
}

//_____________________________________________________________________________
/**
 * Set the local offset of each non-fixed marker so that in the model's
 * current pose the marker coincides with the marker's global position
 * in the passed-in SimmMarkerData.
 *
 * @param aModel the model to use
 * @param aPose the static-pose marker cloud to get the marker locations from
 */
void SimmMarkerPlacer::moveModelMarkersToPose(AbstractModel& aModel, SimmMarkerData& aPose)
{
	aPose.averageFrames(0.01);
	SimmMarkerFrame* frame = aPose.getFrame(0);

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
					double pt[3], pt2[3], *globalPt = globalMarker.get();
					double conversionFactor = aPose.getUnits().convertTo(aModel.getLengthUnits());
					for (int k = 0; k < 3; k++)
						pt[k] = globalPt[k] * conversionFactor;
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

void SimmMarkerPlacer::peteTest() const
{
	int i;

	cout << "   MarkerPlacementParams: " << getName() << endl;
	cout << "      markerFileName: " << _markerFileName << endl;
	cout << "      timeRange: " << _timeRange << endl;
	cout << "      coordinateFileName: " << _coordinateFileName << endl;
	for (i = 0; i < _coordinateSet.getSize(); i++)
		_coordinateSet.get(i)->peteTest();
	for (i = 0; i < _markerSet.getSize(); i++)
		_markerSet.get(i)->peteTest();
	cout << "      outputJointFile: " << _outputJointFileName << endl;
	cout << "      outputMuscleFile: " << _outputMuscleFileName << endl;
	cout << "      outputModelFile: " << _outputModelFileName << endl;
	cout << "      outputMarkerFile: " << _outputMarkerFileName << endl;
	cout << "      outputMotionFile: " << _outputMotionFileName << endl;
	cout << "      maxMarkerMovement: " << _maxMarkerMovement << endl;
}
