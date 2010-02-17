#ifndef __IKTrial_h__
#define __IKTrial_h__

// IKTrial.h
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


// INCLUDE
#include <iostream>
#include <string>
#include <math.h>
#include "osimToolsDLL.h"
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/Storage.h>
#include "osimToolsDLL.h"

namespace OpenSim {

class Model;
class CoordinateSet;
class MarkerData;
class IKTaskSet;
class IKTarget;
class IKSolverImpl;

//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing how to perform
 * a single inverse kinematics trial on a model and a marker set.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMTOOLS_API IKTrial : public Object  
{
	OPENSIM_DECLARE_DERIVED(IKTrial, Object);

//=============================================================================
// DATA
//=============================================================================
private:

protected:

	//Accumulate the total cost of ike solution across all frames
	double _accumulatedCost;

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

	std::string _optimizerAlgorithm;

	// These are initialized in initializeTrial, and used in solveTrial
	Storage *_inputStorage;
	Storage *_outputStorage;
	IKTarget *_target;
	IKSolverImpl *_ikSolver;
	int _startFrame;
	int _endFrame;

	// Whether or not to write write to the designated output files (GUI will set this to false)
	bool _printResultFiles;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	IKTrial();
	IKTrial(const IKTrial &aIKTrialParams);
	virtual ~IKTrial();
	virtual Object* copy() const;

#ifndef SWIG
	IKTrial& operator=(const IKTrial &aIKTrialParams);
#endif
   void copyData(const IKTrial &aIKTrialParams);

	double getKinematicsSmoothing() const { return _kinematicsSmoothing; }
	double getGroundReactionSmoothing() const { return _groundReactionSmoothing; }

#ifndef SWIG
	bool initializeTrialCommon(const SimTK::State& s, Model& aModel, IKTaskSet& aIKTaskSet, MarkerData& aMarkerData);
	bool initializeTrial(const SimTK::State& s, Model& aModel, IKTaskSet& aIKTaskSet);
	bool solveTrial( SimTK::State& s, Model& aModel, IKTaskSet& aIKTaskSet);
#endif
	void interrupt();

	/*===== Set and Get ===============*/
	double getStartTime() const { return _timeRange[0]; }
	double getEndTime() const { return _timeRange[1]; }
	void setStartTime(double aTime) { _timeRange[0] = aTime; }
	void setEndTime(double aTime) { _timeRange[1] = aTime; }

	int getStartFrame() const { return _startFrame; }
	int getEndFrame() const { return _endFrame; }

	bool getIncludeMarkers() const { return _includeMarkers; }
	void setIncludeMarkers(bool aValue) { _includeMarkers = aValue; }

	const std::string& getMarkerDataFileName() const { return _markerFileName; }
	void setMarkerDataFileName(const std::string& aMarkerFileName) { _markerFileName = aMarkerFileName; }

	const std::string& getOutputMotionFileName() const { return _outputMotionFileName; }
	void setOutputMotionFileName(const std::string& aOutputMotionFileName) { _outputMotionFileName = aOutputMotionFileName; }

	const std::string& getCoordinateFileName() const { return _coordinateFileName; }
	void setCoordinateFileName(const std::string& aFileName) { _coordinateFileName = aFileName; }

	void setOptimizerAlgorithm(const std::string& aOptimizerAlgorithm) { _optimizerAlgorithm = aOptimizerAlgorithm; }
	std::string getOptimizerAlgorithm() const { return _optimizerAlgorithm; }

	//FIKSTool needs access to the Marker Map generated by the IKtarget
	IKTarget *getIKTarget() { return _target;}

	// Get the input experimental marker data 
	Storage *getExperimentalMarkerData() { return _inputStorage; }
	Storage *getOutputStorage() { return _outputStorage; }

	void setAccumulatedCost(double totCost) {_accumulatedCost = totCost;}
	double getAccumulatedCost() {return _accumulatedCost;}

	void setPrintResultFiles(bool aToWrite) { _printResultFiles = aToWrite; }

private:
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class IKTrial
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __IKTrial_h__


