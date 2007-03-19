#ifndef __SimmMotionData_h__
#define __SimmMotionData_h__

// SimmMotionData.h
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
#include "Array.h"
#include "ArrayPtrs.h"
#include "SimmMotionEvent.h"
#include "Units.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM motion file.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMCOMMON_API SimmMotionData : public Object
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
	Units _units;
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
	SimmMotionData(const SimmMotionData &aData);
	virtual ~SimmMotionData();
	virtual Object* copy() const;
#ifndef SWIG
	SimmMotionData& operator=(const SimmMotionData &aData);
#endif

	void copyData(const SimmMotionData &aData);

	int getNumColumns() const { return _numColumns; }
	int getColumnIndex(const std::string& aName) const;
	void setColumnLabel(int aIndex, const std::string& aLabel);
	double getValue(const std::string& aName, int aFrameIndex);
	double getValue(const int columnIndex, int aFrameIndex);
	double getRangeMin() const { return _rangeMin; }
	double getRangeMax() const { return _rangeMax; }
	int getNumberOfFrames() const { return _rows.getSize(); }
	void addToRdStorage(Storage& rStorage, double aStartTime, double aEndTime);
	bool deleteColumn(const std::string& aColumnName);
	void scaleColumn(int aColumnIndex, double aScaleFactor);

	int getFrameNumberForTime(double time) const;
	void writeSIMMMotionFile(const std::string& aFileName, const std::string& aName, const std::string& aComment) const;
	const Array<std::string>& getColumnNames()const
	{
		return _columnNames;
	}
	const char* getUnassignedColName() const { return "Unassigned"; }
	void peteTest() const;

private:
	void setNull();
	void readMotionFileHeader(std::ifstream &in, const std::string& aFileName, SimmMotionData& rData);

//=============================================================================
};	// END of class SimmMotionData
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmMotionData_h__


