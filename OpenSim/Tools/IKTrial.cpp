// IKTrial.cpp
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
#include <OpenSim/Common/rdMath.h>
#include "IKTrial.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include "IKTaskSet.h"
#include "IKSolverImpl.h"
#include "IKTarget.h"

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
IKTrial::IKTrial() :
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
 * Destructor.
 */
IKTrial::~IKTrial()
{
	delete _inputStorage;
	delete _outputStorage;
	delete _target;
	delete _ikSolver;
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMarkerPlacementParams IKTrial to be copied.
 */
IKTrial::IKTrial(const IKTrial &aIKTrialParams) :
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
 * @return Pointer to a copy of this IKTrial.
 */
Object* IKTrial::copy() const
{
	IKTrial *IKTrialParams = new IKTrial(*this);
	return(IKTrialParams);
}

void IKTrial::copyData(const IKTrial &aIKTrialParams)
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
	_printResultFiles = aIKTrialParams._printResultFiles;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this IKTrial to their null values.
 */
void IKTrial::setNull()
{
	setType("IKTrial");

	_coordinateFileName = "";
	_inputStorage = 0;
	_outputStorage = 0;
	_target = 0;
	_ikSolver = 0;
	_printResultFiles = true;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void IKTrial::setupProperties()
{
	_markerFileNameProp.setComment("TRC file (.trc) containing the time history of experimental marker positions.");
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

	_analogFileNameProp.setComment("Analog file (.ana) containing emg and reaction force data. "
		"Not currently used.");
	_analogFileNameProp.setName("analog_file");
	_propertySet.append(&_analogFileNameProp);

	const double defaultTimeRange[] = {-1.0, -1.0};
	_timeRangeProp.setComment("Time range over which the IK problem is solved.");
	_timeRangeProp.setName("time_range");
	_timeRangeProp.setValue(2, defaultTimeRange);
	_propertySet.append(&_timeRangeProp);

	_kinematicsSmoothingProp.setComment("Smoothing factor for preprocessing the kinematic data. "
		"Not currently used.");
	_kinematicsSmoothingProp.setName("kinematics_smoothing");
	_kinematicsSmoothingProp.setValue(-1.0);
	_propertySet.append(&_kinematicsSmoothingProp);

	_groundReactionSmoothingProp.setComment("Smoothing factor for preprocessing reaction force data. "
		"Not currently used.");
	_groundReactionSmoothingProp.setName("ground_reaction_smoothing");
	_groundReactionSmoothingProp.setValue(-1.0);
	_propertySet.append(&_groundReactionSmoothingProp);

	_includeMarkersProp.setComment("Flag (true or false) indicating whether or not to include marker "
		"positions in the output motion (.mot) file.");
	_includeMarkersProp.setName("include_markers");
	_includeMarkersProp.setValue(false);
	_propertySet.append(&_includeMarkersProp);

	_outputMotionFileNameProp.setComment("Name of the motion file (.mot) to which the results should be written.");
	_outputMotionFileNameProp.setName("output_motion_file");
	_propertySet.append(&_outputMotionFileNameProp);

	_notesProp.setComment("Notes for the trial.");
	_notesProp.setName("notes");
	_propertySet.append(&_notesProp);
}

IKTrial& IKTrial::operator=(const IKTrial &aIKTrialParams)
{
	// BASE CLASS
	Object::operator=(aIKTrialParams);

	copyData(aIKTrialParams);

	return(*this);
}
//_____________________________________________________________________________
/**
 * initializeTrialCommon -- must call before solveTrial
 */
bool IKTrial::initializeTrialCommon(Model& aModel, IKTaskSet& aIKTaskSet, MarkerData& aMarkerData)
{
	// Initialize input storage
	delete _inputStorage;
	_inputStorage = new Storage();

	// Initialize output storage
	delete _outputStorage;
	_outputStorage = new Storage();
	_outputStorage->setName(getName());

	// In case we have left over
	delete _target;
	_target = 0;
	delete _ikSolver;
	_ikSolver = 0;

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
		/* Convert marker data to an Storage object. */
		aMarkerData.makeRdStorage(*_inputStorage);

		if (!_coordinateFileName.empty() && _coordinateFileName!=PropertyStr::getDefaultStr())
		{
			Storage coordinateValues(_coordinateFileName);
			aModel.getDynamicsEngine().convertDegreesToRadians(coordinateValues);

			// Adjust the user-defined start and end times to make sure they are in the
			// range of the marker data. This must be done so that you only look in the
			// coordinate data for rows that will actually be solved.
			double firstStateTime = _inputStorage->getFirstTime();
			double lastStateTime = _inputStorage->getLastTime();
			double startTime = MAX(firstStateTime, getStartTime());
			double endTime = MIN(lastStateTime, getEndTime());

			// Add the coordinate data to the marker data. There must be a row of
			// corresponding coordinate data for every row of marker data that will
			// be solved, or it is a fatal error.
			coordinateValues.addToRdStorage(*_inputStorage, startTime, endTime);
		}

		_inputStorage->print("markers_coords_ik.sto");

		// Create target
		_target = new IKTarget(aModel, aIKTaskSet, *_inputStorage);
		_target->printTasks();

		// Create solver
		_ikSolver = new IKSolverImpl(*_target);
		_ikSolver->initializeSolver(*this, *_inputStorage, *_outputStorage);
	}
	catch (Exception &x)
	{
		x.print(cout);
		delete _inputStorage; _inputStorage = 0;
		delete _outputStorage; _outputStorage = 0;
		delete _ikSolver; _ikSolver = 0;
		delete _target; _target = 0;
		return false;
	}

	return true;
}
//_____________________________________________________________________________
/**
 */
bool IKTrial::initializeTrial(Model& aModel, IKTaskSet& aIKTaskSet)
{
	cout << endl << "Initializing IK trial: " << getName() << endl;

	MarkerData markerData(_markerFileName);
	/* Convert the marker data into the model's units. */
	markerData.convertToUnits(aModel.getLengthUnits());

	if(!initializeTrialCommon(aModel, aIKTaskSet, markerData))
		return false;

	return true;
}
//_____________________________________________________________________________
/**
 */
bool IKTrial::solveTrial(Model& aModel, IKTaskSet& aIKTaskSet)
{
	cout << endl << "Solving IK trial: " << getName() << endl;

	// Check if trial was initialized
	if(!_ikSolver) throw Exception("IKTrial.solveTrial: ERR- IKTrial not initialized before solveTrial called",__FILE__,__LINE__);

	bool success = false;
	try 
	{
		_ikSolver->solveFrames(*this, *_inputStorage, *_outputStorage);

		if (_printResultFiles && !_outputMotionFileNameProp.getUseDefault())
		{
			// TODO: avoid converting units in-place (as this means that suddenly as soon as IK is done the storage
			// (available to others through getOutputStorage() changes units)
			_outputStorage->setWriteSIMMHeader(true);
			aModel.getDynamicsEngine().convertRadiansToDegrees(*_outputStorage);
			_outputStorage->print(_outputMotionFileName.c_str());
		}

		success = true;
	}
	catch (Exception &x)
	{
		x.print(cout);
	}

	// We're done with the IK solver, but we keep around _inputStorage/_outputStorage
	delete _ikSolver; _ikSolver = 0;
	delete _target; _target = 0;

	return success;
}
