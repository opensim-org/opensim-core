#ifndef __MarkerPlacer_h__
#define __MarkerPlacer_h__

// MarkerPlacer.h
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


// INCLUDE
#include <iostream>
#include <string>
#include <math.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyStr.h>
#include "IKTaskSet.h"
#include "osimToolsDLL.h"

namespace OpenSim {

class Model;
class MarkerData;
class IKTrial;
class Storage;

//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing how to place markers
 * on a model (presumably after it has been scaled to fit a subject).
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMTOOLS_API MarkerPlacer : public Object  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	// whether or not to apply marker placer
	PropertyBool _applyProp;
	bool &_apply;

	// name of marker file that contains marker locations in the static pose
	PropertyStr _markerFileNameProp;
	std::string &_markerFileName;

	// range of frames to average in static pose marker file, specified by time
	PropertyDblArray _timeRangeProp;
	Array<double> &_timeRange;

	// tasks used to specify IK weights
	PropertyObj _ikTaskSetProp;
	IKTaskSet &_ikTaskSet;

	// name of SIMM motion file that contains [optional] coordinates for the static pose
	PropertyStr _coordinateFileNameProp;
	std::string &_coordinateFileName;

	// name of SIMM joint file to write when done placing markers
	PropertyStr _outputJointFileNameProp;
	std::string &_outputJointFileName;

	// name of SIMM muscle file to write when done placing markers
	PropertyStr _outputMuscleFileNameProp;
	std::string &_outputMuscleFileName;

	// name of XML model file to write when done placing markers
	PropertyStr _outputModelFileNameProp;
	std::string &_outputModelFileName;

	// name of marker file to write when done placing markers
	PropertyStr _outputMarkerFileNameProp;
	std::string &_outputMarkerFileName;

	// name of motion file (containing solved static pose) when done placing markers
	PropertyStr _outputMotionFileNameProp;
	std::string &_outputMotionFileName;

	// amount of allowable motion for each marker when averaging frames of the static trial
	PropertyDbl _maxMarkerMovementProp;
	double &_maxMarkerMovement;

	/** Preferred optimizer algorithm. */
	PropertyStr _optimizerAlgorithmProp;
	std::string &_optimizerAlgorithm;

	// Whether or not to write write to the designated output files (GUI will set this to false)
	bool _printResultFiles;
	// Whether to move the model markers (set to false if you just want to preview the static pose)
	bool _moveModelMarkers;

	IKTrial *_ikTrial;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	MarkerPlacer();
	MarkerPlacer(const MarkerPlacer &aMarkerPlacementParams);
	virtual ~MarkerPlacer();
	virtual Object* copy() const;

#ifndef SWIG
	MarkerPlacer& operator=(const MarkerPlacer &aMarkerPlacementParams);
#endif
   void copyData(const MarkerPlacer &aMarkerPlacementParams);

	bool processModel(Model* aModel, const std::string& aPathToSubject="");

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------

	bool getApply() const { return _apply; }
	void setApply(bool aApply) 
	{ 
		_apply = aApply; 
		_applyProp.setUseDefault(false); 
	}

	const std::string &getStaticPoseFileName() const { return _markerFileName; }
	void setStaticPoseFileName(const std::string &aFileName) 
	{
		_markerFileName = aFileName;
		_markerFileNameProp.setUseDefault(false);
	}

   const Array<double> &getTimeRange() const { return _timeRange; }
   void setTimeRange(const Array<double> &timeRange) 
	{
		_timeRange = timeRange; 
		_timeRangeProp.setUseDefault(false); 
	}

	IKTaskSet &getIKTaskSet() { return _ikTaskSet; }

	const std::string &getCoordinateFileName() const { return _coordinateFileName; }
	void setCoordinateFileName(const std::string& aCoordinateFileName)
	{
		_coordinateFileName = aCoordinateFileName;
		_coordinateFileNameProp.setUseDefault(false);
	}
    
	double getMaxMarkerMovement() const { return _maxMarkerMovement; }
	void setMaxMarkerMovement(double aMaxMarkerMovement)
	{
		_maxMarkerMovement=aMaxMarkerMovement;
		_maxMarkerMovementProp.setUseDefault(false);
	}

	const std::string& getOutputModelFileName() const { return _outputModelFileName; }
	void setOutputModelFileName(const std::string& aOutputModelFileName)
	{
		_outputModelFileName = aOutputModelFileName;
		_outputModelFileNameProp.setUseDefault(false);
	}

	const std::string& getOutputJointFileName() const { return _outputJointFileName; }
	void setOutputJointFileName(const std::string& outputJointFileName)
	{
		_outputJointFileName = outputJointFileName;
		_outputJointFileNameProp.setUseDefault(false);
	}

	const std::string& getOutputMuscleFileName() const { return _outputMuscleFileName; }
	void setOutputMuscleFileName(const std::string& outputMuscleFileName)
	{
		_outputMuscleFileName = outputMuscleFileName;
		_outputMuscleFileNameProp.setUseDefault(false);
	}

	const std::string& getOutputMarkerFileName() const { return _outputMarkerFileName; }
	void setOutputMarkerFileName(const std::string& outputMarkerFileName)
	{
		_outputMarkerFileName = outputMarkerFileName;
		_outputMarkerFileNameProp.setUseDefault(false);
	}

	const std::string& getOutputMotionFileName() const { return _outputMotionFileName; }
	void setOutputMotionFileName(const std::string& outputMotionFileName)
	{
		_outputMotionFileName = outputMotionFileName;
		_outputMotionFileNameProp.setUseDefault(false);
	}

	void setPrintResultFiles(bool aToWrite) { _printResultFiles = aToWrite; }

	bool getMoveModelMarkers() { return _moveModelMarkers; }
	void setMoveModelMarkers(bool aMove) { _moveModelMarkers = aMove; }

	Storage *getOutputStorage();

	IKTrial *getIKTrial() { return _ikTrial; }

private:
	void setNull();
	void setupProperties();
	void moveModelMarkersToPose(Model& aModel, MarkerData& aPose);
//=============================================================================
};	// END of class MarkerPlacer
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MarkerPlacer_h__


