#ifndef _SimmMotionData_h_
#define _SimmMotionData_h_

// SimmMotionData.h
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
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/Array.h>
#include <OpenSim/Tools/ArrayPtrs.h>
#include "SimmMotionEvent.h"
#include "SimmUnits.h"

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM motion file.
 *
 * @author Peter Loan
 * @version 1.0
 */
namespace OpenSim { 

class RDSIMULATION_API SimmMotionData
{

//=============================================================================
// DATA
//=============================================================================
private:
	std::string _name;
	int _numRows;
	int _numColumns;
	double _rangeMin;
	double _rangeMax;
	SimmUnits _units;
	std::string _fileName;
	Array<std::string> _columnNames;
	Array<std::string> _keys;
	Array<double*> _rows;

	// read from file, but may or may not be used in the future
	double _timeStep;
	int _numOtherData;
	bool _wrap;
	bool _calcDerivatives;
	bool _enforceLoops;
	bool _enforceConstraints;
	double _cursorColor[3];
	bool _showCursor;
	bool _slidingTimeScale;
	ArrayPtrs<SimmMotionEvent> _events;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmMotionData();
	SimmMotionData(const std::string& aFileName);
	SimmMotionData(Storage& aData);
	virtual ~SimmMotionData();

	int getNumColumns() const { return _numColumns; }
	int getColumnIndex(const std::string& aName) const;
	void setColumnLabel(int aIndex, std::string& aLabel);
	double getValue(const std::string& aName, int aFrameIndex);
	double getRangeMin() const { return _rangeMin; }
	double getRangeMax() const { return _rangeMax; }
	void addToRdStorage(Storage& aStorage, double startTime, double endTime);

	void writeSIMMMotionFile(const std::string& aFileName, const std::string& aComment) const;

	void peteTest() const;

private:
	void readMotionFileHeader(std::ifstream &in, const std::string& aFileName, SimmMotionData& data);

//=============================================================================
};	// END of class SimmMotionData

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimmMotionData_h__


