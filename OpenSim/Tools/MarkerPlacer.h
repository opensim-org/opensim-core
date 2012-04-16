#ifndef __MarkerPlacer_h__
#define __MarkerPlacer_h__

// MarkerPlacer.h
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
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyStr.h>
#include "IKTaskSet.h"
#include "osimToolsDLL.h"
#include "SimTKsimbody.h"

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
class OSIMTOOLS_API MarkerPlacer : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MarkerPlacer, Object);

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

	// Whether or not to write write to the designated output files (GUI will set this to false)
	bool _printResultFiles;
	// Whether to move the model markers (set to false if you just want to preview the static pose)
	bool _moveModelMarkers;

	Storage* _outputStorage;
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

	void copyData(const MarkerPlacer &aMarkerPlacementParams);

#ifndef SWIG
	MarkerPlacer& operator=(const MarkerPlacer &aMarkerPlacementParams);
#endif
	bool processModel( SimTK::State& s, Model* aModel, const std::string& aPathToSubject="");

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


private:
	void setNull();
	void setupProperties();
	void moveModelMarkersToPose(SimTK::State& s, Model& aModel, MarkerData& aPose);
//=============================================================================
};	// END of class MarkerPlacer
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MarkerPlacer_h__


