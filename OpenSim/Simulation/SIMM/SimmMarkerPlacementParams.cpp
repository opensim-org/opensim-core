// SimmMarkerPlacementParams.cpp
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
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
#include "SimmMarkerPlacementParams.h"
#include "SimmModel.h"
#include "SimmMarkerData.h"
#include "SimmMotionData.h"

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmMarkerPlacementParams::SimmMarkerPlacementParams() :
   _markerFileName(_markerFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
   _coordinateFileName(_coordinateFileNameProp.getValueStr()),
	_coordinateSetProp(PropertyObj("", SimmCoordinateSet())),
	_coordinateSet((SimmCoordinateSet&)_coordinateSetProp.getValueObj()),
	_markerSetProp(PropertyObj("", SimmMarkerSet())),
	_markerSet((SimmMarkerSet&)_markerSetProp.getValueObj()),
	_outputJointFileName(_outputJointFileNameProp.getValueStr()),
	_outputMuscleFileName(_outputMuscleFileNameProp.getValueStr()),
	_outputModelFileName(_outputModelFileNameProp.getValueStr()),
	_outputMarkerFileName(_outputMarkerFileNameProp.getValueStr()),
	_outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
	_maxMarkerMovement(_maxMarkerMovementProp.getValueDbl())
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmMarkerPlacementParams::SimmMarkerPlacementParams(DOMElement *aElement) :
   Object(aElement),
   _markerFileName(_markerFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
   _coordinateFileName(_coordinateFileNameProp.getValueStr()),
	_coordinateSetProp(PropertyObj("", SimmCoordinateSet())),
	_coordinateSet((SimmCoordinateSet&)_coordinateSetProp.getValueObj()),
	_markerSetProp(PropertyObj("", SimmMarkerSet())),
	_markerSet((SimmMarkerSet&)_markerSetProp.getValueObj()),
	_outputJointFileName(_outputJointFileNameProp.getValueStr()),
	_outputMuscleFileName(_outputMuscleFileNameProp.getValueStr()),
	_outputModelFileName(_outputModelFileNameProp.getValueStr()),
	_outputMarkerFileName(_outputMarkerFileNameProp.getValueStr()),
	_outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
	_maxMarkerMovement(_maxMarkerMovementProp.getValueDbl())
{
	setNull();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmMarkerPlacementParams::~SimmMarkerPlacementParams()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMarkerPlacementParams SimmMarkerPlacementParams to be copied.
 */
SimmMarkerPlacementParams::SimmMarkerPlacementParams(const SimmMarkerPlacementParams &aMarkerPlacementParams) :
   Object(aMarkerPlacementParams),
   _markerFileName(_markerFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
   _coordinateFileName(_coordinateFileNameProp.getValueStr()),
	_coordinateSetProp(PropertyObj("", SimmCoordinateSet())),
	_coordinateSet((SimmCoordinateSet&)_coordinateSetProp.getValueObj()),
	_markerSetProp(PropertyObj("", SimmMarkerSet())),
	_markerSet((SimmMarkerSet&)_markerSetProp.getValueObj()),
	_outputJointFileName(_outputJointFileNameProp.getValueStr()),
	_outputMuscleFileName(_outputMuscleFileNameProp.getValueStr()),
	_outputModelFileName(_outputModelFileNameProp.getValueStr()),
	_outputMarkerFileName(_outputMarkerFileNameProp.getValueStr()),
	_outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
	_maxMarkerMovement(_maxMarkerMovementProp.getValueDbl())
{
	setupProperties();
	copyData(aMarkerPlacementParams);
}
//_____________________________________________________________________________
/**
 * Copy this marker placement params and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmMarkerPlacementParams.
 */
Object* SimmMarkerPlacementParams::copy() const
{
	SimmMarkerPlacementParams *markerPlacementParams = new SimmMarkerPlacementParams(*this);
	return(markerPlacementParams);
}
//_____________________________________________________________________________
/**
 * Copy this SimmMarkerPlacementParams and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmMarkerPlacementParams::SimmMarkerPlacementParams(DOMElement*) in order to establish the
 * relationship of the SimmMarkerPlacementParams object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmMarkerPlacementParams object. Finally, the data members of the
 * copy are updated using SimmMarkerPlacementParams::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmMarkerPlacementParams.
 */
Object* SimmMarkerPlacementParams::copy(DOMElement *aElement) const
{
	SimmMarkerPlacementParams *markerPlacementParams = new SimmMarkerPlacementParams(aElement);
	*markerPlacementParams = *this;
	markerPlacementParams->updateFromXMLNode();
	return(markerPlacementParams);
}

void SimmMarkerPlacementParams::copyData(const SimmMarkerPlacementParams &aMarkerPlacementParams)
{
	_markerFileName = aMarkerPlacementParams._markerFileName;
	_timeRange = aMarkerPlacementParams._timeRange;
	_coordinateFileName = aMarkerPlacementParams._coordinateFileName;
	_coordinateSet = aMarkerPlacementParams._coordinateSet;
	_markerSet = aMarkerPlacementParams._markerSet;
	_outputJointFileName = aMarkerPlacementParams._outputJointFileName;
	_outputMuscleFileName = aMarkerPlacementParams._outputMuscleFileName;
	_outputModelFileName = aMarkerPlacementParams._outputModelFileName;
	_outputMarkerFileName = aMarkerPlacementParams._outputMarkerFileName;
	_outputMotionFileName = aMarkerPlacementParams._outputMotionFileName;
	_maxMarkerMovement = aMarkerPlacementParams._maxMarkerMovement;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmMarkerPlacementParams to their null values.
 */
void SimmMarkerPlacementParams::setNull()
{
	setupProperties();
	setType("SimmMarkerPlacementParams");
	setName("");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmMarkerPlacementParams::setupProperties()
{
	_markerFileNameProp.setName("marker_trial");
	_markerFileNameProp.setComment("Name of the .trc file containing the time history of marker positions. This is usually a static trial.");
	_propertySet.append(&_markerFileNameProp);

	const double defaultTimeRange[] = {-1.0, -1.0};
	_timeRangeProp.setName("time_range");
	_timeRangeProp.setComment("Time range for averaging marker positions.");
	_timeRangeProp.setValue(2, defaultTimeRange);
	_propertySet.append(&_timeRangeProp);

	_coordinateFileNameProp.setName("coordinate_trial");
	_coordinateFileNameProp.setComment("Name of file containing the joint angles used to set the initial configuration of the model for the purpose of relocating the non-fixed markers.");
	_propertySet.append(&_coordinateFileNameProp);

	_coordinateSetProp.setName("SimmCoordinateSet");
	_coordinateSetProp.setComment("Coordinate parameters (e.g., weights, locked/unlocked) can be specified here.");
	_propertySet.append(&_coordinateSetProp);

	_markerSetProp.setName("SimmMarkerSet");
	_markerSetProp.setComment("Markers to use if different from or in addition to the generic model.");
	_propertySet.append(&_markerSetProp);

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

SimmMarkerPlacementParams& SimmMarkerPlacementParams::operator=(const SimmMarkerPlacementParams &aMarkerPlacementParams)
{
	// BASE CLASS
	Object::operator=(aMarkerPlacementParams);

	copyData(aMarkerPlacementParams);

	return(*this);
}


/* This method creates a simmMotionTrial instance with the markerFile and timeRange
parameters. It also creates a SimmMotionData instance with the coordinateFile
parameter. Then it calls model->updateCoordinateSet() with the coordinate set
parameters. Then it calls model->updateMarkerSet() with the marker set parameters.
Then it calls model->solveInverseKinematics(simmMotionTrial) to fit the model to
the static pose. Then it calls model->moveMarkers(simmMotionTrial) which uses the
current model pose to relocate all non-fixed markers according to their locations
in the simmMotionTrial. Then it writes the output files selected by the user.
*/
bool SimmMarkerPlacementParams::processModel(SimmModel* aModel, const std::string& pathToSubject)
{
	cout << endl << "Step 3: Placing markers on model" << endl;

	SimmMotionData* solvedStatic = NULL;

	try
	{
		/* Load the coordinate file. */
		if (!_coordinateFileNameProp.getUseDefault())
		{
			SimmMotionData coordinateValues(pathToSubject+_coordinateFileName);

			/* For each coordinate whose "value" field the user specified
			 * as "fromFile", read the value from the first frame in the
			 * coordinate file (a SIMM motion file) and use it to overwrite
			 * the "fromFile" specification.
			 */
			if (coordinateValues.getNumColumns() > 0)
			{
				for (int i = 0; i < _coordinateSet.getSize(); i++)
				{
					if (_coordinateSet[i]->getValueStr() == "fromFile")
					{
						double newValue = coordinateValues.getValue(_coordinateSet[i]->getName(), 0);
						_coordinateSet[i]->setValue(newValue);
					}
				}

				/* Update the model with the coordinates specified
				 * by the user in the params section.
			    */
				aModel->updateCoordinates(_coordinateSet);
			}
		}

		/* Update the model with the markers specified in the params section. */
		aModel->updateMarkers(_markerSet);

		/* Load the static pose marker file, and average all the
		 * frames in the user-specified time range.
		 */
		SimmMarkerData staticPose(pathToSubject+_markerFileName);
		staticPose.averageFrames(_maxMarkerMovement, _timeRange[0], _timeRange[1]);
		staticPose.convertToUnits(aModel->getLengthUnits());

		/* Delete any markers from the model that are not in the static
		 * pose marker file.
		 */
		aModel->deleteUnusedMarkers(staticPose.getMarkerNames());
#if 0
		/* Now solve the static pose. */
		SimmIKTrialParams options;
		options.setStartTime(_timeRange[0]);
		options.setEndTime(_timeRange[0]);
		options.setIncludeMarkers(true);

		//solvedStatic = aModel->solveInverseKinematics(options, staticPose);

		/* Now move the non-fixed markers on the model so that they are coincident
		 * with the measured markers in the static pose. The model is already in
		 * the proper configuration so the coordinates do not need to be changed.
		 */
		aModel->moveMarkersToCloud(staticPose);
#endif
	}
	catch (Exception &x)
	{
		x.print(cout);

		if (solvedStatic)
			delete solvedStatic;

		cout << "Press Return to continue. " << endl;
		cout.flush();
		int c = getc( stdin );
		return false;
	}

	delete solvedStatic;

	return true;
}


void SimmMarkerPlacementParams::peteTest() const
{
	int i;

	cout << "   MarkerPlacementParams: " << getName() << endl;
	cout << "      markerFileName: " << _markerFileName << endl;
	cout << "      timeRange: " << _timeRange << endl;
	cout << "      coordinateFileName: " << _coordinateFileName << endl;
	for (i = 0; i < _coordinateSet.getSize(); i++)
		_coordinateSet[i]->peteTest();
	for (i = 0; i < _markerSet.getSize(); i++)
		_markerSet[i]->peteTest();
	cout << "      outputJointFile: " << _outputJointFileName << endl;
	cout << "      outputMuscleFile: " << _outputMuscleFileName << endl;
	cout << "      outputModelFile: " << _outputModelFileName << endl;
	cout << "      outputMarkerFile: " << _outputMarkerFileName << endl;
	cout << "      outputMotionFile: " << _outputMotionFileName << endl;
	cout << "      maxMarkerMovement: " << _maxMarkerMovement << endl;
}
/**
 * Check if the values in an instance of SimmMarkerPlacementParams are different from those put by default.
 */
bool SimmMarkerPlacementParams::isDefault() const
{
	return (_markerFileName=="Unassigned");
}
/**
 * Write output simm and xml files if desired.
 */

void SimmMarkerPlacementParams::writeOutputFiles(SimmModel* aModel, Storage& aStorage, const char* aPath) const
{
	string path=(aPath==0)?"":string(aPath);
	// write output files, if names specified by the user
	if (!_outputJointFileNameProp.getUseDefault())
		aModel->writeSIMMJointFile(path+_outputJointFileName);

	if (!_outputMuscleFileNameProp.getUseDefault())
		aModel->writeSIMMMuscleFile(path+_outputMuscleFileName);

	if (!_outputModelFileNameProp.getUseDefault())
	{
		aModel->print(path+_outputModelFileName);
		cout << "Wrote model file " << _outputModelFileName << " from model " << aModel->getName() << endl;
	}

	if (!_outputMarkerFileNameProp.getUseDefault())
	{
		aModel->writeMarkerFile(path+_outputMarkerFileName);
		cout << "Wrote marker file " << _outputMarkerFileName << " from model " << aModel->getName() << endl;
	}
	SimmMotionData motionData(aStorage);
	if (!_outputMotionFileNameProp.getUseDefault())
		motionData.writeSIMMMotionFile(path+_outputMotionFileName, aModel->getName());


}
//_____________________________________________________________________________
/**
 * Manually add a SimmCoordinate to a coordinateSet used for marker placement.
 */
void SimmMarkerPlacementParams::addCoordinate(SimmCoordinate* aCoordinate)
{
	_coordinateSet.append(aCoordinate);
}
