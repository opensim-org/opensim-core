// SimmMarkerData.cpp
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
#include <iostream>
#include <fstream>
#include <math.h>
#include <float.h>
#include <OpenSim/Tools/rdMath.h>
#include "SimmMarkerData.h"
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
SimmMarkerData::SimmMarkerData() :
	_numFrames(0),
	_numMarkers(0),
	_markerNames("")
{
}

SimmMarkerData::SimmMarkerData(const string& aFileName) :
	_numFrames(0),
	_numMarkers(0),
	_markerNames("")
{

#if 0
   if (!aFileName)
		return;

   if (!lookForFile(aFileName, gWorkingDir.c_str(), actualFilename))
   {
      return smFileError;
   }
#endif

   /* Check if the suffix is TRC or TRB. Will read TRC by default */
	string suffix;
   int dot = aFileName.find_last_of(".");
   suffix.assign(aFileName, dot+1, 3);

   if ((suffix == "TRB") || (suffix == "trb"))
      readTRBFile(aFileName, *this);
   else
      readTRCFile(aFileName, *this);

	_fileName = aFileName;

	cout << "Loaded marker file " << _fileName << " (" << _numMarkers << " markers, " << _numFrames << " frames)" << endl;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmMarkerData::~SimmMarkerData()
{
}

void SimmMarkerData::readTRCFile(const string& aFileName, SimmMarkerData& data)
{
   ifstream in;
   string line, buffer;
   int frameNum, coordsRead;
   double time, coords[3];
   bool ok = true;

   in.open(aFileName.c_str());

	if (!in.good())
	{
		string errorMessage;
		errorMessage = "Unable to open marker file " + aFileName;
		throw Exception(errorMessage);
	}

   readTRCFileHeader(in, aFileName, data);

   /* read frame data */
   while (getline(in, line))
   {
      /* skip over any blank lines */
      if (findFirstNonWhiteSpace(line) == -1)
         continue;

		if (data._frames.getSize() == data._numFrames)
		{
#if 0
			if (gUseGlobalMessages)
			{
				gErrorBuffer += "Extra data found at end of tracked marker file. ";
				gErrorBuffer += "Header declared only " + intToString(trc->header.numFrames) + " frames.\n";
			}
			rc = smFileWarning;
#endif
			break;
		}

      if (!readIntegerFromString(line, &frameNum))
      {
#if 0
         if (gUseGlobalMessages)
            gErrorBuffer += "Could not read frame number in tracked marker file.\n";
         rc = smFileError;
         goto cleanup;
#endif
      }

      if (!readDoubleFromString(line, &time))
      {
#if 0
         if (gUseGlobalMessages)
            gErrorBuffer += "Could not read time in tracked marker file.\n";
         rc = smFileError;
         goto cleanup;
#endif
      }

		SimmMarkerFrame *frame = new SimmMarkerFrame(data._numMarkers, frameNum, time, data._units);

      /* keep reading sets of coordinates until the end of the line is
       * reached. If more coordinates were read than there are markers,
       * return an error.
       */
      coordsRead = 0;
      while (readCoordinatesFromLine(line, coords))
      {
         if (coordsRead >= data._numMarkers)
         {
            break;

#if 0  // Don't return an error because many TRC files have extra data at the ends of rows
            if (gUseGlobalMessages)
               gErrorBuffer += "Extra data found in frame number " + intToString(frameNum) +
                               " in tracked marker file.\n";
            rc = smFileError;
            // delete the current markerCoordList because framesRead has not been incremented yet.
            delete [] f->markerCoordList;
            goto cleanup;
#endif
         }
         if (coordsRead < data._numMarkers)
				frame->addMarker(coords);
         coordsRead++;
      }

      if (coordsRead < data._numMarkers)
      {
#if 0
         if (gUseGlobalMessages)
            gErrorBuffer += " Missing data in frame number " + intToString(frameNum) +
                            " in tracked marker file.\n";
         rc = smFileError;
         // delete the current markerCoordList because framesRead has not been incremented yet.
         delete [] f->markerCoordList;
         goto cleanup;
#endif
      }
		data._frames.append(frame);
   }

   if (data._frames.getSize() < data._numFrames)
   {
#if 0
      if (gUseGlobalMessages)
         gErrorBuffer += "Missing data in tracked marker file. Only " + intToString(framesRead) + " of " +
                         intToString(trc->header.numFrames) + " frames found.\n";
      rc = smFileError;
      goto cleanup;
#endif
		data._numFrames = data._frames.getSize();
   }

   /* If the user-defined frame numbers are not continguous from the first frame to the
    * last, reset them to a contiguous array. This is necessary because the user-defined
    * numbers are used to index the array of frames.
    */
	if (data._frames[data._numFrames-1]->getFrameNumber() - data._frames[0]->getFrameNumber() !=
		 data._numFrames - 1)
   {
		int firstIndex = data._frames[0]->getFrameNumber();
      for (int i = 1; i < data._numFrames; i++)
			data._frames[i]->setFrameNumber(firstIndex + i);
   }

#if 0
   if (gUseGlobalMessages)
   {
      gMessage += "TRC file " + actualFileName + "\n\t" + intToString(trc->header.numFrames)
                  + " frames\n\t" + intToString(trc->header.numMarkers) + " markers/frame\n";
      gMessage += "Read " + intToString(framesRead) + " frames.\n";
   }
#endif

//cleanup:
   in.close();
}

void SimmMarkerData::readTRCFileHeader(ifstream &in, const string& aFileName, SimmMarkerData& data)
{
   string line, buffer;
   int pathFileType, markersRead;
   bool ok = true;

   /* read first line in TRC header */
   getline(in, line);

   /* read "PathFileType" and path file type */
   readStringFromLine(line, buffer);
   readIntegerFromString(line, &pathFileType);
   if (pathFileType != 3 && pathFileType != 4)
   {
#if 0
      if (gUseGlobalMessages)
         gErrorBuffer += "Unknown file type " + intToString(pathFileType) + " in TRC file" + actualFileName;
      return smFileError;
#endif
   }

   /* read line 2 - header info column names */
   getline(in, line);

   /* read line 3 - header info */
   getline(in, line);
   
   /* read first 5 parameters from file */
   ok = readDoubleFromString(line, &data._dataRate);
   ok = ok && readDoubleFromString(line, &data._cameraRate);
   ok = ok && readIntegerFromString(line, &data._numFrames);
   ok = ok && readIntegerFromString(line, &data._numMarkers);
   ok = ok && readStringFromLine(line, buffer);

   if (pathFileType == 3)
   {
      if (!ok)
      {
#if 0
         if (gUseGlobalMessages)
            gErrorBuffer += "Could not read line 3 in TRC file " + actualFileName + ".\n";
         return smFormatError;
#endif
      }
      data._originalDataRate = data._dataRate;
      data._originalStartFrame = 1;
      data._originalNumFrames = data._numFrames;
   }
   else if (pathFileType == 4)
   {
      ok = ok && readDoubleFromString(line, &data._originalDataRate);
      ok = ok && readIntegerFromString(line, &data._originalStartFrame);
      ok = ok && readIntegerFromString(line, &data._originalNumFrames);
      if (!ok)
      {
#if 0
         if (gUseGlobalMessages)
            gErrorBuffer += "Could not read line3 in TRC file " + actualFileName + ".\n";
         return smFormatError;
#endif
      }
   }

   data._units = SimmUnits(buffer);

   /* read line 4 - trc data column names */
   getline(in, line);

   /* read Frame# and Time */
   readStringFromLine(line, buffer);
   readStringFromLine(line, buffer);

   /* read the marker names */
   markersRead = 0;
   while (!line.empty())
   {
      if (!readTabDelimitedStringFromLine(line, buffer))
         break;
      if (markersRead >= data._numMarkers)
      {
#if 0
         if (gUseGlobalMessages)
            gMessage += "More marker names in TRC file than in model. Ignoring extra marker names.\n";
         break;
#endif
      }
		data._markerNames.append(buffer);
      markersRead++;
   }

	/* If we don't read the header, we'll throw meaningful exception and abort rather than crash the machine!! */
  if (markersRead < data._numMarkers)
   {
		string errorMessage;
		errorMessage = "Could not read all marker names in TRC file " + aFileName + 
			". Make sure there's exactly one tab per column & that Marker names are tab separated in header.\n";
		throw Exception(errorMessage);

   }

   /* Store the position of the pointer into the file just before reading the
    * coordinate labels. This is done because the first frame is read incorrectly
    * in certain cases.
	 */
   long pos = in.tellg();
   /* read line 5 - coordinate labels (X1 Y1 Z1 X2 Y2 Z2 ...) */
   getline(in, line);

   /* The following code supports 0 or 1 blank lines before the first
    * frame of data.  Read the line of code - if there is a blank line of code,
    * read the next line which will contain the data.
    */
   getline(in, line);
   if (line.empty())
      getline(in, line);

   if (!readIntegerFromString(line, &data._firstFrameNumber))
      data._firstFrameNumber = 1;

   /* reposition the pointer into the file so it points to before the coordinate
    * labels */
   in.seekg(pos, ios::beg);

   /* reread the coordinate labels so pointer into file points to data */
   getline(in, line);
   if (line.empty())
      getline(in, line);
}

void SimmMarkerData::readTRBFile(const string& aFileName, SimmMarkerData& data)
{
#if 0
   int i, j, index, headerSize, numMarkersThisFrame;
   unsigned short header[6];
   long data[100];
   FILE* file;

   trc->filename = new char [actualFileName.size() + 1];
   strcpy(trc->filename, actualFileName.c_str());

   readTRBFileHeader(actualFileName, &trc->header, headerSize);

   file = fopen(actualFileName.c_str(), "rb");

   trc->frameList = new smTRCFrame [trc->header.numFrames];

   fseek(file, headerSize, SEEK_SET);

   for (i = 0; i < trc->header.numFrames; i++)
   {
      trc->frameList[i].frameNum = i;
      trc->frameList[i].time = (double) i / trc->header.dataRate;
      trc->frameList[i].units = trc->header.units;

      trc->frameList[i].numMarkers = trc->header.numMarkers;
      trc->frameList[i].markerCoordList = new smPoint3 [trc->frameList[i].numMarkers];

      // initialize all the markers to UNDEFINED
      for (j = 0; j < trc->frameList[i].numMarkers; j++)
      {
         trc->frameList[i].markerCoordList[j][0] = UNDEFINED_DOUBLE;
         trc->frameList[i].markerCoordList[j][1] = UNDEFINED_DOUBLE;
         trc->frameList[i].markerCoordList[j][2] = UNDEFINED_DOUBLE;
      }

      // now read the header to see how many markers are present
      for (j = 0; j < 6; j++)
         header[j] = _read_binary_unsigned_short(file);

      numMarkersThisFrame = (header[4] - 3) / 6;

      for (j = 0; j < numMarkersThisFrame; j++)
      {
         fread(data, 6*4, 1, file);

         // the index of this marker is stored in the first data element
         index = data[0] - 1;

         // if the index is good, copy the marker coordinates
         if (index >= 0 && index < trc->frameList[i].numMarkers)
         {
            trc->frameList[i].markerCoordList[index][0] = *(float *)(&data[1]);
            trc->frameList[i].markerCoordList[index][1] = *(float *)(&data[2]);
            trc->frameList[i].markerCoordList[index][2] = *(float *)(&data[3]);
         }
      }
   }

   goto finish;

//invalid:
   if (gUseGlobalMessages)
      gErrorBuffer += "Cannot read " + actualFileName + ". File format is invalid.";
   return smFormatError;

   smFreeTRCStruct(trc);

finish:
   fclose(file);
   return smNoError;
#endif
}

/* Find the range of frames that is between start time and end time
 * (inclusive). Return the indices of the bounding frames.
 */
void SimmMarkerData::findFrameRange(double aStartTime, double aEndTime, int& oStartFrame, int& oEndFrame) const
{
	int i;

	oStartFrame = 0;
	oEndFrame = _numFrames - 1;

	if (aStartTime > aEndTime)
	{
		double tmp = aStartTime;
		aStartTime = aEndTime;
		aEndTime = tmp;
	}

	for (i = _numFrames - 1; i >= 0 ; i--)
	{
		if (_frames[i]->getFrameTime() <= aStartTime + rdMath::ZERO)
		{
			oStartFrame = i;
			break;
		}
	}

	for (i = oStartFrame; i < _numFrames; i++)
	{
		if (_frames[i]->getFrameTime() >= aEndTime - rdMath::ZERO)
		{
			oEndFrame = i;
			break;
		}
	}
}

/* This method averages all frames between aStartTime and
 * aEndTime (inclusive) and stores the result in the first
 * frame. All other frames are deleted. The time and frame
 * number of this one remaining frame are copied from the
 * startIndex frame. The aThreshold parameter is for printing
 * a warning if any marker moves more than that amount in
 * the averaged frames. aThreshold should always be specified
 * in meters; it will be converted to the units of the
 * marker data.
 */
void SimmMarkerData::averageFrames(double aThreshold, double aStartTime, double aEndTime)
{
	if (_numFrames < 2)
		return;

	int startIndex = 0, endIndex = 1;
	double *minX = NULL, *minY = NULL, *minZ = NULL, *maxX = NULL, *maxY = NULL, *maxZ = NULL;

	findFrameRange(aStartTime, aEndTime, startIndex, endIndex);
	SimmMarkerFrame *averagedFrame = new SimmMarkerFrame(*_frames[startIndex]);

	/* If aThreshold is greater than zero, then calculate
	 * the movement of each marker so you can check if it
	 * is greater than aThreshold.
	 */
	if (aThreshold > 0.0)
	{
		minX = new double [_numMarkers];
		minY = new double [_numMarkers];
		minZ = new double [_numMarkers];
		maxX = new double [_numMarkers];
		maxY = new double [_numMarkers];
		maxZ = new double [_numMarkers];
		for (int i = 0; i < _numMarkers; i++)
		{
			minX[i] = minY[i] = minZ[i] = rdMath::INFINITY;
			maxX[i] = maxY[i] = maxZ[i] = rdMath::MINUS_INFINITY;
		}
	}

	/* Initialize all the averaged marker locations to 0,0,0. Then
	 * loop through the frames to be averaged, adding each marker location
	 * to averagedFrame. Keep track of the min/max XYZ for each marker
	 * so you can compare it to aThreshold when you're done.
	 */
	for (int i = 0; i < _numMarkers; i++)
	{
		int numFrames = 0;
		SimmPoint& avePt = averagedFrame->getMarker(i);
		avePt.set(0.0, 0.0, 0.0);

		for (int j = startIndex; j <= endIndex; j++)
		{
			SimmPoint& pt = _frames[j]->getMarker(i);
			if (pt.isVisible())
			{
				double* coords = pt.get();
				avePt += pt;
				numFrames++;
				if (aThreshold > 0.0)
				{
					if (coords[0] < minX[i])
						minX[i] = coords[0];
					if (coords[0] > maxX[i])
						maxX[i] = coords[0];
					if (coords[1] < minY[i])
						minY[i] = coords[1];
					if (coords[1] > maxY[i])
						maxY[i] = coords[1];
					if (coords[2] < minZ[i])
						minZ[i] = coords[2];
					if (coords[2] > maxZ[i])
						maxZ[i] = coords[2];
				}
			}
		}

		/* Now divide by the number of frames to get the average. */
		if (numFrames > 0)
			avePt /= (double)numFrames;
		else
			avePt.set(rdMath::NAN, rdMath::NAN, rdMath::NAN);
	}

	/* Store the indices from the file of the first frame and
	 * last frame that were averaged, so you can report them later.
	 */
	int startUserIndex = _frames[startIndex]->getFrameNumber();
	int endUserIndex = _frames[endIndex]->getFrameNumber();

	/* Now delete all the existing frames and insert the averaged one. */
	_frames.clearAndDestroy();
	_frames.append(averagedFrame);
	_numFrames = 1;
	_firstFrameNumber = _frames[0]->getFrameNumber();

	if (aThreshold > 0.0)
	{
		for (int i = 0; i < _numMarkers; i++)
		{
			SimmPoint& pt = _frames[0]->getMarker(i);

			if (!pt.isVisible())
			{
				cout << "___WARNING___: marker " << _markerNames[i] << " is missing in frames " << startUserIndex
					  << " to " << endUserIndex << ". Coordinates will be set to NAN." << endl;
			}
			else if (maxX[i] - minX[i] > aThreshold ||
				      maxY[i] - minY[i] > aThreshold ||
				      maxZ[i] - minZ[i] > aThreshold)
			{
				double maxDim = maxX[i] - minX[i];
				maxDim = MAX(maxDim, (maxY[i] - minY[i]));
				maxDim = MAX(maxDim, (maxZ[i] - minZ[i]));
				cout << "___WARNING___: movement of marker " << _markerNames[i] << " in " << _fileName
					  << " is " << maxDim << " (threshold = " << aThreshold << ")" << endl;
			}
		}
	}

	cout << "Averaged frames from time " << aStartTime << " to " << aEndTime << " in " << _fileName
		  << " (frames " << startUserIndex << " to " << endUserIndex << ")" << endl;

	if (aThreshold > 0.0)
	{
		delete [] minX;
		delete [] minY;
		delete [] minZ;
		delete [] maxX;
		delete [] maxY;
		delete [] maxZ;
	}
}

/* Measure a length in a marker set. The length is defined by the average distance
 * between 1 or more pairs of markers, as stored in a SimmMeasurement. This
 * method takes the measurement on the first frame in the SimmMarkerData.
 */
double SimmMarkerData::takeMeasurement(const SimmMeasurement& aMeasurement) const
{
	double length;
	const string *name1 = NULL, *name2 = NULL;
	int i, numPairs;

	/* For each pair of markers, calculate the distance between them
	 * and add it to the running total.
	 */
	for (i = 0, length = 0.0, numPairs = 0; i < aMeasurement.getNumMarkerPairs(); i++)
	{
		int marker1 = -1, marker2 = -1;
		const SimmMarkerPair& pair = aMeasurement.getMarkerPair(i);
		pair.getMarkerNames(name1, name2);
		for (int j = 0; j < _markerNames.getSize(); j++)
		{
			if (_markerNames[j] == *name1)
				marker1 = j;
			if (_markerNames[j] == *name2)
				marker2 = j;
		}
		if (marker1 >= 0 && marker2 >= 0)
		{
			double* p1 = _frames[0]->getMarker(marker1).get();
			double* p2 = _frames[0]->getMarker(marker2).get();
			length += sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) + (p2[1]-p1[1])*(p2[1]-p1[1]) + (p2[2]-p1[2])*(p2[2]-p1[2]));
			numPairs++;
		}
		else
		{
			if (marker1 < 0)
				cout << "___WARNING___: marker " << *name1 << " in " << aMeasurement.getName() << " measurement not found in " << _fileName << endl;
			if (marker2 < 0)
				cout << "___WARNING___: marker " << *name2 << " in " << aMeasurement.getName() << " measurement not found in " << _fileName << endl;
		}
	}

	/* Divide by the number of pairs to get the average length. */
	if (numPairs == 0)
	{
		cout << "___WARNING___: could not calculate " << aMeasurement.getName() << " measurement on file " << _fileName << endl;
		return rdMath::NAN;
	}
	else
	{
		return length / numPairs;
	}
}

/* Store the marker data in an Storage object. The object
 * is emptied before adding the marker data.
 */
void SimmMarkerData::makeRdStorage(Storage& aStorage)
{
	/* First clear any existing frames. */
	aStorage.reset(0);

	/* Make the column labels. */
	string columnLabels = "time\t";
	int i;
	for (i = 0; i < _numMarkers; i++)
	{
		columnLabels += _markerNames[i] + "_tx\t";
		columnLabels += _markerNames[i] + "_ty\t";
		columnLabels += _markerNames[i] + "_tz\t";
	}
	aStorage.setColumnLabels(columnLabels.c_str());

	/* Store the marker coordinates in an array of doubles
	 * and add it to the Storage.
	 */
	int numColumns = _numMarkers * 3;
	double* row = new double [numColumns];

	for (i = 0; i < _numFrames; i++)
	{
		for (int j = 0, index = 0; j < _numMarkers; j++)
		{
			double* marker = _frames[i]->getMarker(j).get();
			for (int k = 0; k < 3; k++)
				row[index++] = marker[k];
		}
		aStorage.append(_frames[i]->getFrameTime(), numColumns, row);
	}

	delete [] row;
}

SimmMarkerFrame* SimmMarkerData::getFrame(int aIndex) const
{
	if (aIndex < 0 || aIndex >= _numFrames)
		return NULL;

	return _frames[aIndex];
}

int SimmMarkerData::getMarkerIndex(const string& aName) const
{
	for (int i = 0; i < _markerNames.getSize(); i++)
	{
		if (_markerNames[i] == aName)
			return i;
	}

	return -1;
}

void SimmMarkerData::convertToUnits(const SimmUnits& aUnits)
{
	double scaleFactor = _units.convertTo(aUnits);

	if (scaleFactor != rdMath::NAN && scaleFactor != 1.0)
	{
		/* Scale all marker locations by the conversion factor. */
		for (int i = 0; i < _frames.getSize(); i++)
			_frames[i]->scale(scaleFactor);

		/* Change the units for this object to the new ones. */
		_units = aUnits;
	}
}

void SimmMarkerData::peteTest() const
{
	cout << "   MarkerData: " << endl;
	cout << "      numFrames: " << _numFrames << endl;
	cout << "      numMarkers: " << _numMarkers << endl;
	cout << "      firstFrameNumber: " << _firstFrameNumber << endl;
	cout << "      dataRate: " << _dataRate << endl;
	cout << "      cameraRate: " << _cameraRate << endl;
	cout << "      originalDataRate: " << _originalDataRate << endl;
	cout << "      originalStartFrame: " << _originalStartFrame << endl;
	cout << "      originalNumFrames: " << _originalNumFrames << endl;
	cout << "      fileName: " << _fileName << endl;
	cout << "      units: " << _units.getLabel() << endl;

	int i;
	for (i = 0; i < _numMarkers; i++)
		cout << "      marker " << i << ": " << _markerNames[i].c_str() << endl;

	for (i = 0; i < _numFrames; i++)
		_frames[i]->peteTest();
}
