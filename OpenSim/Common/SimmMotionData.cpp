// SimmMotionData.cpp
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <fstream>
#include <sstream>
#include "rdMath.h"
#include "SimmMotionData.h"
#include "SimmIO.h"
#include "SimmMacros.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
// dll export of static data is problematic for DevStudio. Rearrange code around it.
// char SimmMotionData::_unassignedColName[] = "Unassigned";

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmMotionData::SimmMotionData() :
	Object(),
	_numRows(0),
	_numColumns(0),
	_rows(NULL),
	_wrap(false),
	_calcDerivatives(false),
	_enforceLoops(false),
	_enforceConstraints(false),
	_showCursor(false),
	_slidingTimeScale(false)
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Constructor from a SIMM motion file.
 */
SimmMotionData::SimmMotionData(const string& aFileName) :
	Object(),
	_numRows(0),
	_numColumns(0),
	_rows(NULL),
	_wrap(false),
	_calcDerivatives(false),
	_enforceLoops(false),
	_enforceConstraints(false),
	_showCursor(false),
	_slidingTimeScale(false)
{
	setNull();

#if 0
   if (!aFileName)
		return;

   if (!lookForFile(aFileName, gWorkingDir.c_str(), actualFilename))
   {
      return smFileError;
   }
#endif

	int i;
   ifstream in;
   string line, buffer;

   in.open(aFileName.c_str());

   readMotionFileHeader(in, aFileName, *this);

	/* Read the column names. */
	for (i = 0; i < _numColumns; i++)
	{
		string* cName = new string();
		readStringFromStream(in, *cName);
		_columnNames.append(*cName);
	}

	/* Peek ahead to see if there are more column names specified. */
	// TODO

	/* Read the data. */
	for (i = 0; i < _numRows; i++)
	{
		double* row = new double [_numColumns];
		for (int j = 0; j < _numColumns; j++)
		{
			readStringFromStream(in, buffer);
			readDoubleFromString(buffer, &row[j]);
		}
		_rows.append(row);
	}

	_fileName = aFileName;
}

//_____________________________________________________________________________
/**
 * Constructor from a Storage object.
 */
SimmMotionData::SimmMotionData(Storage& aData) :
	Object(),
	_numRows(0),
	_numColumns(0),
	_rows(NULL),
	_wrap(false),
	_calcDerivatives(false),
	_enforceLoops(false),
	_enforceConstraints(false),
	_showCursor(false),
	_slidingTimeScale(false)
{
	setNull();

	/* Copy the column labels. */
	const Array<string>& columnLabels = aData.getColumnLabelsArray();
	//string* timeLabel = new string("time");
	//_columnNames.append(timeLabel);
	int i;
	for (i = 0; i < columnLabels.getSize(); i++)
	{
		string* name = new string(columnLabels[i]);
		_columnNames.append(*name);
	}
	_numColumns = _columnNames.getSize();

	/* Get the data. */
	for (i = 0; i < aData.getSize(); i++)
	{
		double* row = new double [_numColumns];
		aData.getTime(i, row[0]);
		aData.getData(i, _numColumns - 1, &row[1]);
		_rows.append(row);
	}
	_numRows = _rows.getSize();

	/* Set the range of the motion to the min/max time. */
	aData.getTime(0, _rangeMin);
	aData.getTime(aData.getSize() - 1, _rangeMax);
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmMotionData::~SimmMotionData()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aData SimmMotionData to be copied.
 */
SimmMotionData::SimmMotionData(const SimmMotionData &aData) :
   Object(aData),
	_numRows(0),
	_numColumns(0),
	_rows(NULL),
	_wrap(false),
	_calcDerivatives(false),
	_enforceLoops(false),
	_enforceConstraints(false),
	_showCursor(false),
	_slidingTimeScale(false)
{
	setNull();
	copyData(aData);
}

//_____________________________________________________________________________
/**
 * Copy this SimmMotionData and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmMotionData.
 */
Object* SimmMotionData::copy() const
{
	SimmMotionData *data = new SimmMotionData(*this);
	return(data);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmMotionData to their null values.
 */
void SimmMotionData::setNull()
{
	setType("SimmMotionData");
}

//_____________________________________________________________________________
/**
 * Copy data members from one SimmMotionEvent to another.
 *
 * @param aData SimmMotionEvent to be copied.
 */
void SimmMotionData::copyData(const SimmMotionData &aData)
{
	_name = aData._name;
	_numRows = aData._numRows;
	_numColumns = aData._numColumns;
	_rangeMin = aData._rangeMin;
	_rangeMax = aData._rangeMax;
	_units = aData._units;
	_fileName = aData._fileName;
	_columnNames = aData._columnNames;
	_keys = aData._keys;
	_rows = aData._rows;
	_timeStep = aData._timeStep;
	_numOtherData = aData._numOtherData;
	_wrap = aData._wrap;
	_calcDerivatives = aData._calcDerivatives;
	_enforceLoops = aData._enforceLoops;
	_enforceConstraints = aData._enforceConstraints;
	_cursorColor[0] = aData._cursorColor[0];
	_cursorColor[1] = aData._cursorColor[1];
	_cursorColor[2] = aData._cursorColor[2];
	_showCursor = aData._showCursor;
	_slidingTimeScale = aData._slidingTimeScale;
	_events = aData._events;
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
SimmMotionData& SimmMotionData::operator=(const SimmMotionData &aData)
{
	// BASE CLASS
	Object::operator=(aData);

	copyData(aData);

	return(*this);
}

//=============================================================================
// I/O
//=============================================================================
//_____________________________________________________________________________
/**
 * Read the header of a SIMM motion file.
 *
 * @param in stream of the file to read
 * @param aFileName name of the file to read
 * @param rData SimmMotionData to read the header into
 */
void SimmMotionData::readMotionFileHeader(ifstream &in, const string& aFileName, SimmMotionData& rData)
{
   string line, buffer, str, str2, str3;
	double eventColor[3];

   while (1)
   {
		if (!readNonCommentStringFromStream(in, buffer))
			break;

		if (buffer == "endheader")
			break;

      if (buffer == "name")
      {
         if (getline(in, line))
         {
				rData._name = buffer;
				setName(line);	// Set object name as well to be used by the GUI
         }
         else
         {
#if 0
            error(abort_action,"Error reading name of motion.");
            goto error_cleanup;
#endif
         }
      }
      else if (buffer == "wrap")
      {
			rData._wrap = true;
      }
      else if (buffer == "enforce_loops")
      {
			if (readStringFromStream(in, str))
			{
				if (str == "no" || str == "off" || str == "false")
					rData._enforceLoops = false;
				else if (str == "yes" || str == "on" || str == "true")
					rData._enforceLoops = true;
			}
      }
      else if (buffer == "enforce_constraints")
      {
			if (readStringFromStream(in, str))
			{
				if (str == "no" || str == "off" || str == "false")
					rData._enforceConstraints = false;
				else if (str == "yes" || str == "on" || str == "true")
					rData._enforceConstraints = true;
			}
      }
      else if (buffer == "datacolumns")
      {
			if (readStringFromStream(in, str))
			{
				if (!readIntegerFromString(str, &rData._numColumns))
				{
#if 0
					(void)sprintf(errorbuffer,"Error reading datacolumns in motion %s", motion->name);
					error(abort_action,errorbuffer);
					goto error_cleanup;
#endif
				}
			}
      }
      else if (buffer == "keys")
      {
         if (getline(in, line))
			{
				string* key1 = new string();
				string* key2 = new string();

				/* The user can specify either one or two keys. */
				if (readStringFromString(line, *key1))
					rData._keys.append(*key1);
				if (readStringFromString(line, *key2))
					rData._keys.append(*key2);
			}
      }
      else if (buffer == "datarows")
      {
			if (readStringFromStream(in, str))
			{
				if (!readIntegerFromString(str, &rData._numRows) || rData._numRows < 0)
				{
#if 0
					(void)sprintf(errorbuffer,"Error reading datarows in motion %s", motion->name);
					error(abort_action,errorbuffer);
					goto error_cleanup;
#endif
				}
			}
      }
      else if (buffer == "otherdata")
      {
         readStringFromStream(in, str);
         //error(none, "Keyword \'otherdata\' is no longer required in a motion header.");
      }
      else if (buffer == "range")
      {
			/* The next two strings will be interpreted as rangeMin and rangeMax. */
			readStringFromStream(in, str);
			readStringFromStream(in, str2);
			if (!readDoubleFromString(str, &rData._rangeMin) || !readDoubleFromString(str2, &rData._rangeMax))
			{
#if 0
            (void)sprintf(errorbuffer,"Error reading range for motion %s", motion->name);
            error(none,errorbuffer);
            error(none,"Using default values 0.0 to 100.0");
#endif
				rData._rangeMin = 0.0;
				rData._rangeMax = 100.0;
         }
      }
      else if (buffer == "units")
      {
			if (readStringFromStream(in, str))
				rData._units = Units(str);
      }
      else if (buffer == "cursor")
      {
			/* The next three strings will be interpreted as RGB values. */
			readStringFromStream(in, str);
			readStringFromStream(in, str2);
			readStringFromStream(in, str3);
			if (!readDoubleFromString(str, &rData._cursorColor[0]) ||
				 !readDoubleFromString(str2, &rData._cursorColor[1]) ||
				 !readDoubleFromString(str3, &rData._cursorColor[2]))
			{
#if 0
            (void)sprintf(errorbuffer,"Error reading cursor color for motion %s",
               motion->name);
            error(none,errorbuffer);
            error(none,"Using default values of 1.0 1.0 0.0");
#endif
            rData._cursorColor[0] = 1.0;
            rData._cursorColor[1] = 1.0;
            rData._cursorColor[2] = 0.0;
         }
			rData._showCursor = true;
      }
      else if (buffer == "event")
      {
			double time;
			SimmMotionEvent* anEvent = new SimmMotionEvent();

			/* The next two strings will be interpreted as 'time' and 'name'. */
			readStringFromStream(in, str);
			if (readDoubleFromString(str, &time))
			{
				anEvent->setTime(time);
			}
			else
			{
#if 0
            (void)sprintf(errorbuffer,"Error reading event time in motion %s", motion->name);
            error(recover,errorbuffer);
#endif
			}

			if (readStringFromStream(in, str))
			{
				anEvent->setName(str);
			}
			else
			{
#if 0
            (void)sprintf(errorbuffer,"Error reading event name in motion %s", motion->name);
            error(recover,errorbuffer);
#endif
			}
      }
      else if (buffer == "event_color")
      {
			/* The next three strings will be interpreted as RGB values. */
			readStringFromStream(in, str);
			readStringFromStream(in, str2);
			readStringFromStream(in, str3);
			if (!readDoubleFromString(str, &eventColor[0]) ||
				 !readDoubleFromString(str2, &eventColor[1]) ||
				 !readDoubleFromString(str3, &eventColor[2]))
			{
#if 0
            (void)sprintf(errorbuffer,"Error reading event color for motion %s",
               motion->name);
            error(none,errorbuffer);
            error(none,"Using default values of 1.0 0.0 1.0");
#endif
            eventColor[0] = 1.0;
            eventColor[1] = 0.0;
            eventColor[2] = 1.0;
         }
      }
      else if (buffer == "calc_derivatives")
      {
			readStringFromStream(in, str);
			if (!readDoubleFromString(str, &rData._timeStep))
         {
#if 0
            (void)sprintf(errorbuffer,"Error reading calc_derivatives for motion %s", motion->name);
            error(recover,errorbuffer);
#endif
         }
         else if (rData._timeStep <= 0.0)
			{
            //error(recover,"calc_derivatives must be greater than zero.");
			}
         else
			{
				rData._calcDerivatives = true;
			}
      }
      else if (buffer == "accept_realtime_motion")
		{
         // this parameter is no longer supported
		}
      else if (buffer == "sliding_time_scale")
		{
			rData._slidingTimeScale = true;
		}
      else
      {
#if 0
         (void)sprintf(errorbuffer,"Unrecognized string \"%s\" found in motion file.",
            buffer);
         error(recover,errorbuffer);
         num_errors++;
      }
      if (num_errors > 10)
      {
         error(none,"Too many errors to continue.");
         error(none,"Unable to load motion file.");
         goto error_cleanup;
#endif
      }
   }

	/* In the motion file a single color is specified for all the events,
	 * but in the SimmMotionData object each event stores its own color.
	 * So now you have to copy the one specified color into each event
	 * instance.
	 */
	for (int i = 0; i < rData._events.getSize(); i++)
		_events[i]->setColor(eventColor);
}

//_____________________________________________________________________________
/**
 * Write the SimmMotionData to a SIMM motion file.
 *
 * @param aFileName the name of the file to create
 * @param aComment "notes" text to add the file as a comment
 */
void SimmMotionData::writeSIMMMotionFile(const string& aFileName, const string& aName, const string& aComment) const
{
	int i;
	ofstream out;

	out.open(aFileName.c_str());
	out.setf(ios::fixed);
	out.precision(10);

   if (!out.good())
   {
      cout << "Unable to open motion file " << aFileName << endl;
      return;
   }

	out << "#" << aComment << endl;
	out << "name " << aName << endl;
	out << "datacolumns " << _numColumns << endl;
	out << "datarows " << _numRows << endl;
	out << "range " << _rangeMin << " " << _rangeMax << endl;
	out << "units " << _units.getLabel() << endl;
	if (_keys.getSize() > 0)
	{
		out << "keys ";
		for (i = 0; i < MIN(2, _keys.getSize()); i++)
			out << _keys[i] << " ";
		out << endl;
	}
	if (_wrap)
		out << "wrap" << endl;
	out << "enforce_loops " << ((_enforceLoops) ? ("yes") : ("no")) << endl;
	out << "enforce_constraints " << ((_enforceConstraints) ? ("yes") : ("no")) << endl;
	if (_calcDerivatives)
		out << "calc_derivatives " << _timeStep << endl;
	if (_showCursor)
		out << "cursor " << _cursorColor[0] << " " << _cursorColor[1] << " " << _cursorColor[2] << endl;
	for (i = 0; i < _events.getSize(); i++)
	{
		const double* color = _events[i]->getColor();
		out << "event " << _events[i]->getTime() << " " << color[0] << " " << color[1] << " " << color[2] << endl;
	}
	if (_slidingTimeScale)
		out << "sliding_time_scale" << endl;
	out << "endheader" << endl << endl;

	for (i = 0; i < _columnNames.getSize(); i++)
		out << _columnNames[i] << '\t';
	out << endl;

	for (i = 0; i < _numRows; i++)
	{
		int j;
		for (j = 0; j < _numColumns; j++)
			out << _rows[i][j] << '\t';
		out << endl;
	}

	out.close();

	cout << "Wrote SIMM motion file " << aFileName << endl;
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the value of one element in one frame.
 *
 * @param aName the name of the element
 * @param aFrameIndex the index of the frame to get the element from
 * @return The value of the element
 *
 * Note: Column names in .mot files are not unique due to:
 * - old fromat that had all experimental markers having the name ground_marker_..
 * - Ground reaction forces which will have same name for both feet
 */
double SimmMotionData::getValue(const string& aName, int aFrameIndex)
{
	for (int i = 0; i < _columnNames.getSize(); i++)
	{
		if (_columnNames[i] == aName)
			return _rows[aFrameIndex][i];
	}

	return rdMath::NAN;
}
//_____________________________________________________________________________
/**
 * Get the value of one element in one frame.
 *
 * @param aColumnIndex the index of the column
 * @param aFrameIndex the index of the frame to get the element from
 * @return The value of the element
 */
double SimmMotionData::getValue(const int aColumnIndex, int aFrameIndex)
{
	return _rows[aFrameIndex][aColumnIndex];
}

//_____________________________________________________________________________
/**
 * Get the index of a column, specified by name
 *
 * @param aName the name of the column
 * @return The index of the column
 */
int SimmMotionData::getColumnIndex(const string& aName) const
{
	for (int i = 0; i < _columnNames.getSize(); i++)
	{
		if (_columnNames[i] == aName)
			return i;
	}

	return -1;
}

//_____________________________________________________________________________
/**
 * Set the label of a column, specified by index
 *
 * @param aIndex the index of the column
 * @param aLabel the new name of the column
 */
void SimmMotionData::setColumnLabel(int aIndex, const std::string& aLabel)
{
	if (aIndex >= 0 && aIndex < _columnNames.getSize())
		_columnNames[aIndex] = aLabel;
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Add the coordinate data to an Storage object.
 * Assumptions:
 *   1. the first column in the SimmMotionData is 'time'
 *   2. the Storage object already contains data, so
 *      only the SimmMotionData frames whose time stamps
 *      match existing frames are added to it.
 *   3. the Storage object does not already contain any
 *      columns with the same labels as the SimmMotionData
 *      has. If it does, the Storage will end up with
 *      duplicate column labels.
 *
 * @param rStorage the storage to add the data to
 * @param aStartTime the time of the first frame to add
 * @param aEndTime the time of the last frame to add
 */
void SimmMotionData::addToRdStorage(Storage& rStorage, double aStartTime, double aEndTime)
{
	bool addedData = false;
	double time, stateTime;

	/* Loop through the rows in rStorage from aStartTime to aEndTime,
	 * looking for a match (by time) in the rows of SimmMotionData.
	 * If you find a match, add the columns in the SimmMotionData
	 * to the end of the state vector in the Storage. If you
	 * don't find one, it's a fatal error so throw an exception.
	 * Don't add a column if its name is 'unassigned'.
	 */
	int i, j, startIndex = rStorage.findIndex(0, aStartTime);
	int endIndex = rStorage.findIndex(rStorage.getSize() - 1, aEndTime);

	for (i = startIndex; i <= endIndex; i++)
	{
		rStorage.getTime(i, stateTime);
		for (j = 0; j < _numRows; j++)
		{
			/* Assume that the first column is 'time'. */
			time = _rows[j][0];
			if (EQUAL_WITHIN_TOLERANCE(time, stateTime, 0.0001))
			{
				Array<double>& states = rStorage.getStateVector(i)->getData();
				for (int k = 1; k < _numColumns; k++)	// Start at 1 to avoid duplicate time column
				{
					if (_columnNames[k] != getUnassignedColName())
					{
						states.append(_rows[j][k]);
						addedData = true;
					}
				}
				break;
			}
		}
		if (j == _numRows)
		{
			stringstream errorMessage;
			errorMessage << "Error: no coordinate data found at time " << stateTime << " in " << _fileName;
			throw (Exception(errorMessage.str()));
		}
	}

	/* Add the coordinate names to the Storage (if at least
	 * one row of data was added to the object).
	 */
	if (addedData)
	{
		string columnLabels(rStorage.getColumnLabels());
		for (int i = 1; i < _columnNames.getSize(); i++) // Start at 1 to avoid duplicate time label
		{
			if (_columnNames[i] != getUnassignedColName())
				columnLabels += _columnNames[i] + '\t';
		}
		rStorage.setColumnLabels(columnLabels.c_str());
	}
}

bool SimmMotionData::deleteColumn(const string& aColumnName)
{
	int i = getColumnIndex(aColumnName);
	if (i >= 0) 
	{
		for (int j = 0; j < _rows.getSize(); j++)
		{
			double *row = _rows.get(j);
			for (int k = i; k < _numColumns - 1; k++) row[k] = row[k+1];
		}
		_columnNames.remove(i);
		_numColumns--;
		return true;
	}
	else
		return false;
}

void SimmMotionData::scaleColumn(int aColumnIndex, double aScaleFactor)
{
	int i;
	for (i = 0; i < _rows.getSize(); i++)
		_rows.get(i)[aColumnIndex] *= aScaleFactor;
}
//_____________________________________________________________________________
/**
 * find frame number closest to passed in time.
 */
int SimmMotionData::getFrameNumberForTime(double time) const
{
	int i;
	if (time >= getRangeMax())
		return _rows.getSize()-1;
	// Here, it's between two rows. 
	bool done=false;
	for (i = 0; i < _rows.getSize() && !done; i++){
		done= (_rows.get(i)[0] > time);
	}
	// either i or i-1 if exists is closest
	if (i==0)
		return 0;
	else if (i==_rows.getSize())
		return i-1;
	else if (fabs(_rows.get(i)[0] - time) > fabs(_rows.get(i-1)[0] - time))
		return i-1;
	else
		return i;
}

void SimmMotionData::peteTest() const
{
	cout << "   MotionData: " << endl;
	cout << "      numRows: " << _numRows << endl;
	cout << "      numColumns: " << _numColumns << endl;
	cout << "      fileName: " << _fileName << endl;
	cout << "      range: " << _rangeMin << " to " << _rangeMax << endl;
	cout << "      units: " << _units.getLabel() << endl;

	int i;
	for (i = 0; i < _numColumns; i++)
		cout << "      column " << i << ": " << _columnNames[i].c_str() << endl;

	for (i = 0; i < _numRows; i++)
	{
		for (int j = 0; j < _numColumns; j++)
			cout << (_rows[i])[j] << " ";
		cout << endl;
	}
}
