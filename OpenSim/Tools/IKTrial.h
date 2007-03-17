#ifndef __SimmIKTrial_h__
#define __SimmIKTrial_h__

// SimmIKTrial.h
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
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyBool.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Applications/Workflow/workflowDLL.h>

namespace OpenSim {

class AbstractModel;
class CoordinateSet;
class SimmMarkerData;
class IKTaskSet;

//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing how to perform
 * a single inverse kinematics trial on a model and a marker set.
 *
 * @author Peter Loan
 * @version 1.0
 */
class workflow_API SimmIKTrial : public Object  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	// name of marker file that contains marker locations for IK solving
	PropertyStr _markerFileNameProp;
	std::string &_markerFileName;

	// name of motion file that contains coordinate values for IK solving
	PropertyStr _coordinateFileNameProp;
	std::string &_coordinateFileName;

	// name of input analog file that contains ground-reaction and EMG data
	PropertyStr _analogFileNameProp;
	std::string &_analogFileName;

	// range of frames to solve in marker file, specified by time
	PropertyDblArray _timeRangeProp;
	Array<double> &_timeRange;

	// cut-off frequency for low-pass smoothing of kinematics data
	PropertyDbl _kinematicsSmoothingProp;
	double &_kinematicsSmoothing;

	// cut-off frequency for low-pass smoothing of ground-reaction data
	PropertyDbl _groundReactionSmoothingProp;
	double &_groundReactionSmoothing;

	// whether or not to include marker trajectories in the output data
	PropertyBool _includeMarkersProp;
	bool &_includeMarkers;

	// name of motion file (containing solved static pose) when done placing markers
	PropertyStr _outputMotionFileNameProp;
	std::string &_outputMotionFileName;

	// comment string
	PropertyStr _notesProp;
	std::string &_notes;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmIKTrial();
	SimmIKTrial(const SimmIKTrial &aIKTrialParams);
	virtual ~SimmIKTrial();
	virtual Object* copy() const;

#ifndef SWIG
	SimmIKTrial& operator=(const SimmIKTrial &aIKTrialParams);
#endif
   void copyData(const SimmIKTrial &aIKTrialParams);

	double getStartTime() const { return _timeRange[0]; }
	double getEndTime() const { return _timeRange[1]; }
	double getKinematicsSmoothing() const { return _kinematicsSmoothing; }
	double getGroundReactionSmoothing() const { return _groundReactionSmoothing; }
	bool getIncludeMarkers() const { return _includeMarkers; }

	void setStartTime(double aTime) { _timeRange[0] = aTime; }
	void setEndTime(double aTime) { _timeRange[1] = aTime; }
	void setIncludeMarkers(bool aValue) { _includeMarkers = aValue; }
	void findFrameRange(const Storage& aData, int& oStartFrame, int& oEndFrame) const;

	bool processTrial(AbstractModel& aModel, IKTaskSet& aIKTaskSet);
	bool processTrialCommon(AbstractModel& aModel, IKTaskSet& aIKTaskSet, SimmMarkerData& aMarkerData, Storage& aOutputStorage);

	/*===== Set and Get ===============*/
	std::string getMarkerDataFilename() const
	{
		return _markerFileName;
	}
	std::string getOutputMotionFilename() const
	{
		return _outputMotionFileName;
	}
	void setOutputMotionFilename(const std::string& aOutputMotionFilename)
	{
		_outputMotionFileName = aOutputMotionFilename;
	}
	const std::string& getCoordinateFileName() const
	{
		return _coordinateFileName;
	}
	void setCoordinateFileName(const std::string& aFilename)
	{
		_coordinateFileName = aFilename;
	}

	void peteTest() const;

private:
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class SimmIKTrial
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmIKTrial_h__


