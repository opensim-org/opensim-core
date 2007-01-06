// SimmIKTrial.cpp
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
#include <OpenSim/Tools/rdMath.h>
#include "SimmIKTrial.h"
#include <OpenSim/Simulation/SIMM/SimmMacros.h>
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/SIMM/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/SIMM/SimmMotionData.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerData.h>
#include <OpenSim/Simulation/SIMM/AbstractCoordinate.h>
#include <OpenSim/Simulation/SIMM/CoordinateSet.h>
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
SimmIKTrial::SimmIKTrial() :
   _markerFileName(_markerFileNameProp.getValueStr()),
   _coordinateFileName(_coordinateFileNameProp.getValueStr()),
   _analogFileName(_analogFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
   _kinematicsSmoothing(_kinematicsSmoothingProp.getValueDbl()),
   _groundReactionSmoothing(_groundReactionSmoothingProp.getValueDbl()),
   _includeMarkers(_includeMarkersProp.getValueBool()),
	_outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
	_notes(_notesProp.getValueStr())
{
	setNull();
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmIKTrial::SimmIKTrial(DOMElement *aElement) :
   Object(aElement),
   _markerFileName(_markerFileNameProp.getValueStr()),
   _coordinateFileName(_coordinateFileNameProp.getValueStr()),
   _analogFileName(_analogFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
   _kinematicsSmoothing(_kinematicsSmoothingProp.getValueDbl()),
   _groundReactionSmoothing(_groundReactionSmoothingProp.getValueDbl()),
   _includeMarkers(_includeMarkersProp.getValueBool()),
	_outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
	_notes(_notesProp.getValueStr())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmIKTrial::~SimmIKTrial()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMarkerPlacementParams SimmIKTrial to be copied.
 */
SimmIKTrial::SimmIKTrial(const SimmIKTrial &aIKTrialParams) :
   Object(aIKTrialParams),
   _markerFileName(_markerFileNameProp.getValueStr()),
   _coordinateFileName(_coordinateFileNameProp.getValueStr()),
   _analogFileName(_analogFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
   _kinematicsSmoothing(_kinematicsSmoothingProp.getValueDbl()),
   _groundReactionSmoothing(_groundReactionSmoothingProp.getValueDbl()),
   _includeMarkers(_includeMarkersProp.getValueBool()),
	_outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
	_notes(_notesProp.getValueStr())
{
	setNull();
	setupProperties();
	copyData(aIKTrialParams);
}
//_____________________________________________________________________________
/**
 * Copy this IK Trial params and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmIKTrial.
 */
Object* SimmIKTrial::copy() const
{
	SimmIKTrial *IKTrialParams = new SimmIKTrial(*this);
	return(IKTrialParams);
}
//_____________________________________________________________________________
/**
 * Copy this SimmIKTrial and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmIKTrial::SimmIKTrial(DOMElement*) in order to establish the
 * relationship of the SimmIKTrial object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmIKTrial object. Finally, the data members of the
 * copy are updated using SimmIKTrial::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmIKTrial.
 */
Object* SimmIKTrial::copy(DOMElement *aElement) const
{
	SimmIKTrial *IKTrialParams = new SimmIKTrial(aElement);
	*IKTrialParams = *this;
	IKTrialParams->updateFromXMLNode();
	return(IKTrialParams);
}

void SimmIKTrial::copyData(const SimmIKTrial &aIKTrialParams)
{
	_markerFileName = aIKTrialParams._markerFileName;
	_coordinateFileName = aIKTrialParams._coordinateFileName;
	_analogFileNameProp = aIKTrialParams._analogFileNameProp;
	_timeRange = aIKTrialParams._timeRange;
	_kinematicsSmoothing = aIKTrialParams._kinematicsSmoothing;
	_groundReactionSmoothing = aIKTrialParams._groundReactionSmoothing;
	_includeMarkers = aIKTrialParams._includeMarkers;
	_outputMotionFileName = aIKTrialParams._outputMotionFileName;
	_notes = aIKTrialParams._notes;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmIKTrial to their null values.
 */
void SimmIKTrial::setNull()
{
	setType("SimmIKTrial");

	_coordinateFileName = "";
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmIKTrial::setupProperties()
{
	_markerFileNameProp.setName("marker_file");
	_propertySet.append(&_markerFileNameProp);

	_coordinateFileNameProp.setName("coordinate_file");
	_propertySet.append(&_coordinateFileNameProp);

	_analogFileNameProp.setName("analog_file");
	_propertySet.append(&_analogFileNameProp);

	const double defaultTimeRange[] = {-1.0, -1.0};
	_timeRangeProp.setName("time_range");
	_timeRangeProp.setValue(2, defaultTimeRange);
	_propertySet.append(&_timeRangeProp);

	_kinematicsSmoothingProp.setName("kinematics_smoothing");
	_kinematicsSmoothingProp.setValue(-1.0);
	_propertySet.append(&_kinematicsSmoothingProp);

	_groundReactionSmoothingProp.setName("ground_reaction_smoothing");
	_groundReactionSmoothingProp.setValue(-1.0);
	_propertySet.append(&_groundReactionSmoothingProp);

	_includeMarkersProp.setName("include_markers");
	_includeMarkersProp.setValue(false);
	_propertySet.append(&_includeMarkersProp);

	_outputMotionFileNameProp.setName("output_motion_file");
	_propertySet.append(&_outputMotionFileNameProp);

	_notesProp.setName("notes");
	_propertySet.append(&_notesProp);
}

SimmIKTrial& SimmIKTrial::operator=(const SimmIKTrial &aIKTrialParams)
{
	// BASE CLASS
	Object::operator=(aIKTrialParams);

	copyData(aIKTrialParams);

	return(*this);
}

bool SimmIKTrial::processTrialCommon(AbstractModel& aModel, CoordinateSet& aCoordinateSet, Array<std::string>& aCoordinatesFromFile, SimmMarkerData& aMarkerData, Storage& aOutputStorage)
{
	SimmMotionData* coordinateValues = NULL;

	// During the IK trial, *all* coordinates that have values specified
	// in the input coordinate file will use those values for the
	// trial (these values can be used either as starting values
	// or as final values, depending on the coordinate's locked
	// state). So you may have to change the names of some of the
	// data columns in the file to prevent them from being used to
	// drive the coordinates, because you want to use them only
	// if the user specified valueFromFile=true in the subject file.
	try
	{
		// Update the model with the coordinates specified
		// by the user in the params section.
		aModel.getDynamicsEngine().updateCoordinateSet(aCoordinateSet);

		// Now set the values of the coordinates listed in the
		// coordinates_from_file section of the SimmIKSolver parameters.
		if (_coordinateFileName != "" && aCoordinatesFromFile.getSize() > 0)
		{
			coordinateValues = new SimmMotionData(_coordinateFileName);
			coordinateValues->convertDegreesToRadians(aModel);

			if (coordinateValues->getNumColumns() > 0)
			{
				// Loop through every coordinate in the model, not just the ones in
				// the coordinates_from_file section. You need to do this because coordinates
				// not in that list must have their data removed from the input coordinate
				// file.
				int i;
				const CoordinateSet* coordinateSet = aModel.getDynamicsEngine().getCoordinateSet();

				for (i = 0; i < coordinateSet->getSize(); i++)
				{
					AbstractCoordinate* modelCoord = coordinateSet->get(i);
					int index = aCoordinatesFromFile.findIndex(modelCoord->getName());
					if (index >= 0) 
					{
						double value = coordinateValues->getValue(modelCoord->getName(), 0);
						if (value == rdMath::NAN)
						{
							// If the value is not found in the file, it is an error.
							string errorMessage = "Coordinate " + modelCoord->getName() +
								" was listed in coordinates_from_file section of SimmIKSolver, but no value found in " + _coordinateFileName;
							throw Exception(errorMessage);
						}
						else
						{
							// If the value is found in the file, set the coordinate in the
							// model to this value, regardless of its 'locked' state.
							bool lockedState = modelCoord->getLocked();
							modelCoord->setLocked(false);
							modelCoord->setValue(value);
							modelCoord->setLocked(lockedState);
							cout << "Values for coordinate " << modelCoord->getName() << " will be read from " <<	_coordinateFileName << endl;
						}
					}
					else
					{
						// The coordinate is not in the coordinates_from_file list,
						// so delete its data from the input coordinate file, if it exists.
						coordinateValues->deleteColumn(modelCoord->getName());
					}
				}
			}
		}

		/* Convert marker data to an Storage object. */
		Storage inputStorage;
		aMarkerData.makeRdStorage(inputStorage);

		if (coordinateValues)
		{
			// Adjust the user-defined start and end times to make sure they are in the
			// range of the marker data. This must be done so that you only look in the
			// coordinate data for rows that will actually be solved.
			double firstStateTime = inputStorage.getFirstTime();
			double lastStateTime = inputStorage.getLastTime();
			double startTime = MAX(firstStateTime, getStartTime());
			double endTime = MIN(lastStateTime, getEndTime());

			// Add the coordinate data to the marker data. There must be a row of
			// corresponding coordinate data for every row of marker data that will
			// be solved, or it is a fatal error.
			coordinateValues->addToRdStorage(inputStorage, startTime, endTime);
		}

		inputStorage.print("markers_coords_ik.sto");

		// Create target
		SimmInverseKinematicsTarget *target = new SimmInverseKinematicsTarget(aModel, inputStorage);
		// Create solver
		SimmIKSolverImpl *ikSolver = new SimmIKSolverImpl(*target);
		// Solve
		ikSolver->solveFrames(*this, inputStorage, aOutputStorage);

		delete target;
		delete ikSolver;
	}
	catch (Exception &x)
	{
		x.print(cout);
		delete coordinateValues;
		return false;
	}

	delete coordinateValues;
	return true;
}

bool SimmIKTrial::processTrial(AbstractModel& aModel, CoordinateSet& aCoordinateSet, Array<std::string>& aCoordinatesFromFile)
{
	cout << endl << "Processing IK trial: " << getName() << endl;

	SimmMarkerData markerData(_markerFileName);
	/* Convert the marker data into the model's units. */
	markerData.convertToUnits(aModel.getLengthUnits());

	Storage outputStorage;
	if(!processTrialCommon(aModel,aCoordinateSet,aCoordinatesFromFile,markerData,outputStorage))
		return false;

	if (!_outputMotionFileNameProp.getUseDefault())
	{
		outputStorage.setWriteSIMMHeader(true);
		aModel.getDynamicsEngine().convertRadiansToDegrees(&outputStorage);
		outputStorage.print(_outputMotionFileName.c_str());
	}

	return true;
}

/* Find the range of frames that is between start time and end time
 * (inclusive). Return the indices of the bounding frames.
 */
void SimmIKTrial::findFrameRange(const Storage& aData, int& oStartFrame, int& oEndFrame) const
{
	if (_timeRange[0] > _timeRange[1])
	{
		double tmp = _timeRange[0];
		_timeRange[0] = _timeRange[1];
		_timeRange[1] = tmp;
	}

	oStartFrame = aData.findIndex(0, _timeRange[0]);
	oEndFrame = aData.findIndex(aData.getSize()-1, _timeRange[1]);
}

void SimmIKTrial::peteTest() const
{
	cout << "   SimmIKTrial: " << getName() << endl;
	cout << "      markerFileName: " << _markerFileName << endl;
	cout << "      coordinateFileName: " << _coordinateFileName << endl;
	cout << "      analogFileName: " << _analogFileName << endl;
	cout << "      timeRange: " << _timeRange << endl;
	cout << "      kinematicsSmoothing: " << _kinematicsSmoothing << endl;
	cout << "      groundReactionSmoothing: " << _groundReactionSmoothing << endl;
	cout << "      includeMarkers: " << _includeMarkers << endl;
	cout << "      outputMotionFile: " << _outputMotionFileName << endl;
	cout << "      notes: " << _notes << endl;
}
