#ifndef __MarkerData_h__
#define __MarkerData_h__

// MarkerData.h
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
#include "osimCommonDLL.h"
#include "Object.h"
#include "Storage.h"
#include "rdMath.h"
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
class OSIMCOMMON_API MarkerData : public Object
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
	MarkerData(const std::string& aFileName);
	virtual ~MarkerData();
	void findFrameRange(double aStartTime, double aEndTime, int& rStartFrame, int& rEndFrame) const;
	void averageFrames(double aThreshold = -1.0, double aStartTime = rdMath::MINUS_INFINITY, double aEndTime = rdMath::PLUS_INFINITY);
	const std::string& getFileName() const { return _fileName; }
	void makeRdStorage(Storage& rStorage);
	MarkerFrame* getFrame(int aIndex) const;
	int getMarkerIndex(const std::string& aName) const;
	const Units& getUnits() const { return _units; }
	void convertToUnits(const Units& aUnits);
	const Array<std::string>& getMarkerNames() const { return _markerNames; }
	int getNumFrames() const { return _numFrames; }
	double getStartFrameTime() const;
	double getLastFrameTime() const;
	void peteTest() const;

private:
	void readTRCFile(const std::string& aFileName, MarkerData& aSMD);
	void readTRCFileHeader(std::ifstream &in, const std::string& aFileName, MarkerData& aSMD);
	void readTRBFile(const std::string& aFileName, MarkerData& aSMD);

//=============================================================================
};	// END of class MarkerData
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MarkerData_h__


