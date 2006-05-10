#ifndef _SimmMarkerData_h_
#define _SimmMarkerData_h_

// SimmMarkerData.h
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
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/ArrayPtrs.h>
#include "SimmMarkerFrame.h"
#include "SimmUnits.h"
#include "SimmMeasurement.h"

//=============================================================================
//=============================================================================
/**
 * A class implementing a sequence of marker frames from a TRC/TRB file.
 *
 * @author Peter Loan
 * @version 1.0
 */
#include <OpenSim/Simulation/rdSimulationDLL.h>

namespace OpenSim { 

class RDSIMULATION_API SimmMarkerData
{

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
	SimmUnits _units;
	Array<std::string> _markerNames;
	ArrayPtrs<SimmMarkerFrame> _frames;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmMarkerData();
	SimmMarkerData(std::string& aFileName);
	virtual ~SimmMarkerData();
	void findFrameRange(double aStartTime, double aEndTime, int& oStartFrame, int& oEndFrame) const;
	void averageFrames(double aThreshold = -1.0, double aStartTime = rdMath::MINUS_INFINITY, double aEndTime = rdMath::PLUS_INFINITY);
	double takeMeasurement(const SimmMeasurement& aMeasurement) const;
	const std::string& getFileName() const { return _fileName; }
	void makeRdStorage(Storage& aStorage);
	SimmMarkerFrame* getFrame(int aIndex) const;
	int getMarkerIndex(const std::string& aName) const;
	const SimmUnits& getUnits() const { return _units; }
	void convertToUnits(const SimmUnits& aUnits);
	const Array<std::string>& getMarkerNames() const { return _markerNames; }

	void peteTest() const;

private:
	void readTRCFile(std::string& aFileName, SimmMarkerData& data);
	void readTRCFileHeader(std::ifstream &in, std::string& aFileName, SimmMarkerData& data);
	void readTRBFile(std::string& aFileName, SimmMarkerData& data);

//=============================================================================
};	// END of class SimmMarkerData

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimmMarkerData_h__


