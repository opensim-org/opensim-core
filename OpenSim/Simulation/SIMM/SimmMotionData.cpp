// SimmMotionData.cpp
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <fstream>
#include <OpenSim/Tools/rdMath.h>
#include "SimmMotionData.h"
#include "simmIO.h"
#include "simmMacros.h"

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmMotionData::SimmMotionData() :
	_numRows(0),
	_numColumns(0),
	_keys(""),
	_columnNames(""),
	_rows(NULL),
	_wrap(false),
	_calcDerivatives(false),
	_enforceLoops(false),
	_enforceConstraints(false),
	_showCursor(false),
	_slidingTimeScale(false)
{
}

SimmMotionData::SimmMotionData(const string& aFileName) :
	_numRows(0),
	_numColumns(0),
	_keys(""),
	_columnNames(""),
	_rows(NULL),
	_wrap(false),
	_calcDerivatives(false),
	_enforceLoops(false),
	_enforceConstraints(false),
	_showCursor(false),
	_slidingTimeScale(false)
{

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
		readString(in, *cName);
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
			readString(in, buffer);
			readDoubleFromString(buffer, &row[j]);
		}
		_rows.append(row);
	}

	_fileName = aFileName;
}

SimmMotionData::SimmMotionData(Storage& aData) :
	_numRows(0),
	_numColumns(0),
	_keys(""),
	_columnNames(""),
	_rows(NULL),
	_wrap(false),
	_calcDerivatives(false),
	_enforceLoops(false),
	_enforceConstraints(false),
	_showCursor(false),
	_slidingTimeScale(false)
{
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

void SimmMotionData::readMotionFileHeader(ifstream &in, const string& aFileName, SimmMotionData& data)
{
   string line, buffer, str, str2, str3;
	double eventColor[3];

   while (1)
   {
		if (!readNonCommentString(in, buffer))
			break;

		if (buffer == "endheader")
			break;

      if (buffer == "name")
      {
         if (getline(in, line))
         {
				data._name = buffer;
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
			data._wrap = true;
      }
      else if (buffer == "enforce_loops")
      {
			if (readString(in, str))
			{
				if (str == "no" || str == "off" || str == "false")
					data._enforceLoops = false;
				else if (str == "yes" || str == "on" || str == "true")
					data._enforceLoops = true;
			}
      }
      else if (buffer == "enforce_constraints")
      {
			if (readString(in, str))
			{
				if (str == "no" || str == "off" || str == "false")
					data._enforceConstraints = false;
				else if (str == "yes" || str == "on" || str == "true")
					data._enforceConstraints = true;
			}
      }
      else if (buffer == "datacolumns")
      {
			if (readString(in, str))
			{
				if (!readIntegerFromString(str, &data._numColumns))
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
				if (readStringFromLine(line, *key1))
					data._keys.append(*key1);
				if (readStringFromLine(line, *key2))
					data._keys.append(*key2);
			}
      }
      else if (buffer == "datarows")
      {
			if (readString(in, str))
			{
				if (!readIntegerFromString(str, &data._numRows) || data._numRows < 0)
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
         readString(in, str);
         //error(none, "Keyword \'otherdata\' is no longer required in a motion header.");
      }
      else if (buffer == "range")
      {
			/* The next two strings will be interpreted as rangeMin and rangeMax. */
			readString(in, str);
			readString(in, str2);
			if (!readDoubleFromString(str, &data._rangeMin) || !readDoubleFromString(str2, &data._rangeMax))
			{
#if 0
            (void)sprintf(errorbuffer,"Error reading range for motion %s", motion->name);
            error(none,errorbuffer);
            error(none,"Using default values 0.0 to 100.0");
#endif
				data._rangeMin = 0.0;
				data._rangeMax = 100.0;
         }
      }
      else if (buffer == "units")
      {
			if (readString(in, str))
				data._units = SimmUnits(str);
      }
      else if (buffer == "cursor")
      {
			/* The next three strings will be interpreted as RGB values. */
			readString(in, str);
			readString(in, str2);
			readString(in, str3);
			if (!readDoubleFromString(str, &data._cursorColor[0]) ||
				 !readDoubleFromString(str2, &data._cursorColor[1]) ||
				 !readDoubleFromString(str3, &data._cursorColor[2]))
			{
#if 0
            (void)sprintf(errorbuffer,"Error reading cursor color for motion %s",
               motion->name);
            error(none,errorbuffer);
            error(none,"Using default values of 1.0 1.0 0.0");
#endif
            data._cursorColor[0] = 1.0;
            data._cursorColor[1] = 1.0;
            data._cursorColor[2] = 0.0;
         }
			data._showCursor = true;
      }
      else if (buffer == "event")
      {
			double time;
			SimmMotionEvent* anEvent = new SimmMotionEvent();

			/* The next two strings will be interpreted as 'time' and 'name'. */
			readString(in, str);
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

			if (readString(in, str))
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
			readString(in, str);
			readString(in, str2);
			readString(in, str3);
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
			readString(in, str);
			if (!readDoubleFromString(str, &data._timeStep))
         {
#if 0
            (void)sprintf(errorbuffer,"Error reading calc_derivatives for motion %s", motion->name);
            error(recover,errorbuffer);
#endif
         }
         else if (data._timeStep <= 0.0)
			{
            //error(recover,"calc_derivatives must be greater than zero.");
			}
         else
			{
				data._calcDerivatives = true;
			}
      }
      else if (buffer == "accept_realtime_motion")
		{
         // this parameter is no longer supported
		}
      else if (buffer == "sliding_time_scale")
		{
			data._slidingTimeScale = true;
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
	for (int i = 0; i < data._events.getSize(); i++)
		_events[i]->setColor(eventColor);
}

double SimmMotionData::getValue(const string& aName, int aFrameIndex)
{
	for (int i = 0; i < _columnNames.getSize(); i++)
	{
		if (_columnNames[i] == aName)
			return _rows[aFrameIndex][i];
	}

	return rdMath::NAN;
}

int SimmMotionData::getColumnIndex(const string& aName) const
{
	for (int i = 0; i < _columnNames.getSize(); i++)
	{
		if (_columnNames[i] == aName)
			return i;
	}

	return -1;
}

void SimmMotionData::setColumnLabel(int aIndex, const std::string& aLabel)
{
	if (aIndex >= 0 && aIndex < _columnNames.getSize())
		_columnNames[aIndex] = aLabel;
}

/* Add the coordinate data to an Storage object.
 * Assumptions:
 *   1. the first column in the SimmMotionData is 'time'
 *   2. the Storage object already contains data, so
 *      only the SimmMotionData frames whose time stamps
 *      match existing frames are added to it.
 *   3. the Storage object does not already contain any
 *      columns with the same labels as the SimmMotionData
 *      has. If it does, the Storage will end up with
 *      duplicate column labels.
 */
void SimmMotionData::addToRdStorage(Storage& aStorage, double startTime, double endTime)
{
	bool addedData = false;
	double time, stateTime;

	/* Loop through the rows in aStorage from startTime to endTime,
	 * looking for a match (by time) in the rows of SimmMotionData.
	 * If you find a match, add the columns in the SimmMotionData
	 * to the end of the state vector in the Storage. If you
	 * don't find one, it's a fatal error so throw an exception.
	 */
	int startIndex = aStorage.findIndex(0, startTime);
	int endIndex = aStorage.findIndex(aStorage.getSize() - 1, endTime);
	int j=0;

	for (int i = startIndex; i <= endIndex; i++)
	{
		aStorage.getTime(i, stateTime);
		for (j = 0; j < _numRows; j++)
		{
			/* Assume that the first column is 'time'. */
			time = _rows[j][0];
			if(rdMath::IsEqual(time,stateTime,0.0001)) {
			//if(EQUAL_WITHIN_ERROR(time, stateTime))
			//{
				Array<double>& states = aStorage.getStateVector(i)->getData();
				for (int k = 1; k < _numColumns; k++)	// Start at 1 to avoid duplicate time column
					states.append(_rows[j][k]);
				addedData = true;
				break;
			}
		}
		if (j == _numRows)
		{
			char timeText[8];
			// TODO: use sprintf (recommended) instead of gcvt
#ifdef __linux__
			gcvt(stateTime, 6, timeText);
#else
			_gcvt(stateTime, 6, timeText);
#endif
			string errorMessage = "Error: no coordinate data found at time " + string(timeText) + " in " + _fileName;
			throw (Exception(errorMessage.c_str()));
		}
	}

	/* Add the coordinate names to the Storage (if at least
	 * one row of data was added to the object).
	 */
	if (addedData)
	{
		string columnLabels(aStorage.getColumnLabels());
		for (int i = 1; i < _columnNames.getSize(); i++) // Start at 1 to avoid duplicate time label
			columnLabels += _columnNames[i] + '\t';
		aStorage.setColumnLabels(columnLabels.c_str());
	}
}

void SimmMotionData::writeSIMMMotionFile(const string& aFileName, const string& aComment) const
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

	out << "#File generated from solving marker data for model " << aComment << endl;

	out << "name solved_motion" << endl; // TODO: come up with a better name than this
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
