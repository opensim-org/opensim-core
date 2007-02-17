#ifndef __SimmMarkerPlacer_h__
#define __SimmMarkerPlacer_h__

// SimmMarkerPlacer.h
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
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/PropertyObj.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Simulation/SIMM/IKTaskSet.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Applications/Workflow/workflowDLL.h>

namespace OpenSim {

class AbstractModel;
class SimmMarkerData;

//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing how to place markers
 * on a model (presumably after it has been scaled to fit a subject).
 *
 * @author Peter Loan
 * @version 1.0
 */
class workflow_API SimmMarkerPlacer : public Object  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
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

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmMarkerPlacer();
	SimmMarkerPlacer(const SimmMarkerPlacer &aMarkerPlacementParams);
	virtual ~SimmMarkerPlacer();
	virtual Object* copy() const;

#ifndef SWIG
	SimmMarkerPlacer& operator=(const SimmMarkerPlacer &aMarkerPlacementParams);
#endif
   void copyData(const SimmMarkerPlacer &aMarkerPlacementParams);

	bool processModel(AbstractModel* aModel, const std::string& aPathToSubject="");

	/**
	 * get/set StaticPoseFilename
	 */
	const std::string &getStaticPoseFilename() const
	{
		return _markerFileName;
	}

	void setStaticPoseFilename(const std::string &aFileName) 
	{
		_markerFileName = aFileName;
	}

	/**
	 * get/set timeRange in static trial file
	 */
   Array<double> getTimeRange()
	{
		return _timeRange;
   }
   void setTimeRange(Array<double> timeRange)
	{
		_timeRange = timeRange;
   }

	const std::string &getCoordinateFileName() const
   {
		return _coordinateFileName;
   }
    
	/**
	 * get/set MaxMarkerMovement
	 */
	double getMaxMarkerMovement() const
	{
		return _maxMarkerMovement;
	}

	void setMaxMarkerMovement(double aMaxMarkerMovement)
	{
		if (aMaxMarkerMovement!= -1.0)
		{
			_maxMarkerMovement=aMaxMarkerMovement;
			_maxMarkerMovementProp.setUseDefault(false);
		}
	}

	const std::string& getOutputModelFileName()
	{
		return _outputModelFileName;
	}

	void setOutputModelFileName(const std::string& aOutputModelFileName)
	{
		if (aOutputModelFileName != ""){
			_outputModelFileName = aOutputModelFileName;
			_outputModelFileNameProp.setUseDefault(false);
		}
	}

	const std::string& getOutputJointFileName()
	{
		return _outputJointFileName;
	}
	/**
	 * Expose file names for access from the GUI
	 */
	void setOutputJointFileName(const std::string& outputJointFileName)
	{
		if (outputJointFileName != "")
		{
			_outputJointFileName = outputJointFileName;
			_outputJointFileNameProp.setUseDefault(false);
		}
	}

	const std::string& getOutputMuscleFileName()
	{
		return _outputMuscleFileName;
	}

	void setOutputMuscleFileName(const std::string& outputMuscleFileName)
	{
		if (outputMuscleFileName != "")
		{
			_outputMuscleFileName = outputMuscleFileName;
			_outputMuscleFileNameProp.setUseDefault(false);
		}
	}

	const std::string& getOutputMarkerFileName()
	{
		return _outputMarkerFileName;
	}

	void setOutputMarkerFileName(const std::string& outputMarkerFileName)
	{
		if (outputMarkerFileName != "")
		{
			_outputMarkerFileName = outputMarkerFileName;
			_outputMarkerFileNameProp.setUseDefault(false);
		}
	}

	const std::string& getOutputMotionFileName()
	{
		return _outputMotionFileName;
	}

	void setOutputMotionFileName(const std::string& outputMotionFileName)
	{
		if (outputMotionFileName != "")
		{
			_outputMotionFileName = outputMotionFileName;
			_outputMotionFileNameProp.setUseDefault(false);
		}
	}

	void peteTest() const;

private:
	void setNull();
	void setupProperties();
	void moveModelMarkersToPose(AbstractModel& aModel, SimmMarkerData& aPose);
//=============================================================================
};	// END of class SimmMarkerPlacer
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmMarkerPlacer_h__


