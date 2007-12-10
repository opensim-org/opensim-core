#ifndef __SimmMotionData_h__
#define __SimmMotionData_h__

// SimmMotionData.h
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

private:
	void setNull();
	void readMotionFileHeader(std::ifstream &in, const std::string& aFileName, SimmMotionData& rData);

//=============================================================================
};	// END of class SimmMotionData
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmMotionData_h__


