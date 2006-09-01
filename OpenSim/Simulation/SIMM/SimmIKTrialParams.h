#ifndef _SimmIKTrialParams_h_
#define _SimmIKTrialParams_h_

// SimmIKTrialParams.h
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
#include <OpenSim/Tools/XMLDocument.h>

namespace OpenSim { 

class SimmModel;
class SimmCoordinate;
class SimmMotionData;

//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing how to perform
 * a single inverse kinematics trial on a model and a marker set.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API SimmIKTrialParams : public Object  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	// name of marker file that contains marker locations for IK solving
	PropertyStr _inputMarkerFileNameProp;
	std::string &_inputMarkerFileName;

	// name of motion file that contains coordinate values for IK solving
	PropertyStr _inputCoordinateFileNameProp;
	std::string &_inputCoordinateFileName;

	// name of input analog file that contains ground-reaction and EMG data
	PropertyStr _inputAnalogFileNameProp;
	std::string &_inputAnalogFileName;

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
	SimmIKTrialParams();
	SimmIKTrialParams(DOMElement *aElement);
	SimmIKTrialParams(const SimmIKTrialParams &aIKTrialParams);
	virtual ~SimmIKTrialParams();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;

#ifndef SWIG
	SimmIKTrialParams& operator=(const SimmIKTrialParams &aIKTrialParams);
#endif
   void SimmIKTrialParams::copyData(const SimmIKTrialParams &aIKTrialParams);

	double getStartTime() const { return _timeRange[0]; }
	double getEndTime() const { return _timeRange[1]; }
	double getKinematicsSmoothing() const { return _kinematicsSmoothing; }
	double getGroundReactionSmoothing() const { return _groundReactionSmoothing; }
	bool getIncludeMarkers() const { return _includeMarkers; }

	void setStartTime(double aTime) { _timeRange[0] = aTime; }
	void setEndTime(double aTime) { _timeRange[1] = aTime; }
	void setIncludeMarkers(bool aValue) { _includeMarkers = aValue; }
	void findFrameRange(const Storage& aData, int& oStartFrame, int& oEndFrame) const;
	SimmMotionData *getCoordinateValues(SimmModel& aModel, const char*path=0) const;

	void peteTest() const;
	std::string getMarkerDataFilename() const
	{
		return _inputMarkerFileName;
	}
	std::string getOutputMotionFilename() const
	{
		return _outputMotionFileName;
	}
protected:

private:
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class SimmIKTrialParams

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimmIKTrialParams_h__


