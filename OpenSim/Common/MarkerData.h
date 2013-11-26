#ifndef __MarkerData_h__
#define __MarkerData_h__
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  MarkerData.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


// INCLUDE
#include <iostream>
#include <string>
#include "osimCommonDLL.h"
#include "Object.h"
#include "Storage.h"
#include "ArrayPtrs.h"
#include "MarkerFrame.h"
#include "Units.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a sequence of marker frames from a TRC/TRB file.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMCOMMON_API MarkerData : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MarkerData, Object);

//=============================================================================
// DATA
//=============================================================================
private:
	int _numFrames;
	int _numMarkers;
	int _firstFrameNumber;
	double _dataRate;
	double _cameraRate;
	double _originalDataRate;
	int _originalStartFrame;
	int _originalNumFrames;
	std::string _fileName;
	Units _units;
	Array<std::string> _markerNames;
	ArrayPtrs<MarkerFrame> _frames;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	MarkerData();
	explicit MarkerData(const std::string& aFileName) SWIG_DECLARE_EXCEPTION;
	virtual ~MarkerData();

	void findFrameRange(double aStartTime, double aEndTime, int& rStartFrame, int& rEndFrame) const;
	void averageFrames(double aThreshold = -1.0, double aStartTime = -SimTK::Infinity, double aEndTime = SimTK::Infinity);
	const std::string& getFileName() const { return _fileName; }
	void makeRdStorage(Storage& rStorage);
	const MarkerFrame& getFrame(int aIndex) const;
	int getMarkerIndex(const std::string& aName) const;
	const Units& getUnits() const { return _units; }
	void convertToUnits(const Units& aUnits);
	const Array<std::string>& getMarkerNames() const { return _markerNames; }
	int getNumMarkers() const { return _numMarkers; }
	int getNumFrames() const { return _numFrames; }
	double getStartFrameTime() const;
	double getLastFrameTime() const;
	double getDataRate() const { return _dataRate; }
	double getCameraRate() const { return _cameraRate; }

private:
	void readTRCFile(const std::string& aFileName, MarkerData& aSMD);
	void readTRCFileHeader(std::ifstream &in, const std::string& aFileName, MarkerData& aSMD);
	void readTRBFile(const std::string& aFileName, MarkerData& aSMD);
    void readStoFile(const std::string& aFileName);
    void buildMarkerMap(const Storage& storageToReadFrom, std::map<int, std::string>& markerNames);

//=============================================================================
};	// END of class MarkerData
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MarkerData_h__


