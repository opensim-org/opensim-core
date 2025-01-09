/* -------------------------------------------------------------------------- *
 *                          OpenSim:  MarkerData.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <fstream>
#include <math.h>
#include <float.h>
#include "MarkerData.h"
#include "SimmIO.h"
#include "SimmMacros.h"
#include "Storage.h"
#include "OpenSim/Auxiliary/auxiliaryTestFunctions.h"
#include "OpenSim/Common/STOFileAdapter.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// HELPER FUNCTIONS
//=============================================================================
// Add number of rows (nRows) and number of columns (nColumns) to the header of
// the STO file. Note that nColumns will include time, so it will be number of
// columns in the matrix plus 1 (for time).
inline void addNumRowsNumColumns(const std::string& filenameOld,
    const std::string& filenameNew) {
    TimeSeriesTable table(filenameOld);
    std::regex endheader{ R"( *endheader *)" };
    std::ifstream fileOld{ filenameOld };
    std::ofstream fileNew{ filenameNew };
    std::string line{};
    while (std::getline(fileOld, line)) {
        if (std::regex_match(line, endheader))
            fileNew << "nRows=" << table.getNumRows() << "\n"
            << "nColumns=" << table.getNumColumns() + 1 << "\n"
            << "endheader\n";
        else
            fileNew << line << "\n";
    }
}

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
MarkerData::MarkerData() :
    _numFrames(0),
    _numMarkers(0),
    _markerNames("")
{
}

//_____________________________________________________________________________
/**
 * Constructor from a TRB/TRC file.
 */
MarkerData::MarkerData(const string& aFileName) :
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
   int dot = (int)aFileName.find_last_of(".");
   suffix.assign(aFileName, dot+1, 3);
   SimTK::String sExtension(suffix);
   if (sExtension.toLower() == "trc") 
      readTRCFile(aFileName, *this);
   else if (sExtension.toLower() == "sto")
       readStoFile(aFileName);
   else
       throw Exception("MarkerData: ERROR- Marker file type is unsupported",__FILE__,__LINE__);

   _fileName = aFileName;

   log_info("Loaded marker file {} ({} markers, {} frames)",
           _fileName, _numMarkers, _numFrames);
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
MarkerData::~MarkerData()
{
}

//=============================================================================
// I/O
//=============================================================================
//_____________________________________________________________________________
/**
 * Read TRC file.
 *
 * @param aFilename name of TRC file.
 * @param aSMD MarkerData object to hold the file contents
 */
void MarkerData::readTRCFile(const string& aFileName, MarkerData& aSMD)
{
   ifstream in;
   string line, buffer;
   int frameNum, coordsRead;
   double time;
    SimTK::Vec3 coords;

    if (aFileName.empty())
        throw Exception("MarkerData.readTRCFile: ERROR- Marker file name is empty",__FILE__,__LINE__);

   in.open(aFileName.c_str());

    if (!in.good())
    {
        string errorMessage;
        errorMessage = "Unable to open marker file " + aFileName;
        throw Exception(errorMessage);
    }

   readTRCFileHeader(in, aFileName, aSMD);

   /* read frame data */
   while (getline(in, line))
   {
      /* skip over any blank lines */
      if (findFirstNonWhiteSpace(line) == -1)
         continue;

        if (aSMD._frames.getSize() == aSMD._numFrames)
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

        MarkerFrame *frame = new MarkerFrame(aSMD._numMarkers, frameNum, time, aSMD._units);

      /* keep reading sets of coordinates until the end of the line is
       * reached. If more coordinates were read than there are markers,
       * return an error.
       */
      coordsRead = 0;
      bool allowNaNs = true;
      while (readCoordinatesFromString(line, &coords[0], allowNaNs))
      {
         if (coordsRead >= aSMD._numMarkers)
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
         if (coordsRead < aSMD._numMarkers)
                frame->addMarker(coords);
         coordsRead++;
      }

      if (coordsRead < aSMD._numMarkers)
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
        aSMD._frames.append(frame);
   }

   if (aSMD._frames.getSize() < aSMD._numFrames)
   {
#if 0
      if (gUseGlobalMessages)
         gErrorBuffer += "Missing data in tracked marker file. Only " + intToString(framesRead) + " of " +
                         intToString(trc->header.numFrames) + " frames found.\n";
      rc = smFileError;
      goto cleanup;
#endif
        aSMD._numFrames = aSMD._frames.getSize();
   }

   /* If the user-defined frame numbers are not contiguous from the first frame to the
    * last, reset them to a contiguous array. This is necessary because the user-defined
    * numbers are used to index the array of frames.
    */
    if (aSMD._frames[aSMD._numFrames-1]->getFrameNumber() - aSMD._frames[0]->getFrameNumber() !=
         aSMD._numFrames - 1)
   {
        int firstIndex = aSMD._frames[0]->getFrameNumber();
      for (int i = 1; i < aSMD._numFrames; i++)
            aSMD._frames[i]->setFrameNumber(firstIndex + i);
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

//_____________________________________________________________________________
/**
 * Read TRC header.
 *
 * @param aStream stream to read from.
 * @param aFileName name of file that stream is from.
 * @param aSMD MarkerData object to hold the file contents
 */
void MarkerData::readTRCFileHeader(ifstream &aStream, const string& aFileName, MarkerData& aSMD)
{
   string line, buffer;
   int pathFileType, markersRead;
   bool ok = true;

   /* read first line in TRC header */
   getline(aStream, line);

   /* read "PathFileType" and path file type */
   readStringFromString(line, buffer);
   readIntegerFromString(line, &pathFileType);
   if (buffer != "PathFileType" || (pathFileType != 3 && pathFileType != 4))
   {
        throw Exception("MarkerData: ERR- File "+aFileName+" does not appear to be a valid TRC file",__FILE__,__LINE__);
#if 0
      if (gUseGlobalMessages)
         gErrorBuffer += "Unknown file type " + intToString(pathFileType) + " in TRC file" + actualFileName;
      return smFileError;
#endif
   }

   /* read line 2 - header info column names */
   getline(aStream, line);

   /* read line 3 - header info */
   getline(aStream, line);
   
   /* read first 5 parameters from file */
   ok = readDoubleFromString(line, &aSMD._dataRate);
   ok = ok && readDoubleFromString(line, &aSMD._cameraRate);
   ok = ok && readIntegerFromString(line, &aSMD._numFrames);
   ok = ok && readIntegerFromString(line, &aSMD._numMarkers);
   ok = ok && readStringFromString(line, buffer);

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
      aSMD._originalDataRate = aSMD._dataRate;
      aSMD._originalStartFrame = 1;
      aSMD._originalNumFrames = aSMD._numFrames;
   }
   else if (pathFileType == 4)
   {
      ok = ok && readDoubleFromString(line, &aSMD._originalDataRate);
      ok = ok && readIntegerFromString(line, &aSMD._originalStartFrame);
      ok = ok && readIntegerFromString(line, &aSMD._originalNumFrames);
      if (!ok)
      {
#if 0
         if (gUseGlobalMessages)
            gErrorBuffer += "Could not read line3 in TRC file " + actualFileName + ".\n";
         return smFormatError;
#endif
      }
   }

   aSMD._units = Units(buffer);

   /* read line 4 - trc data column names */
   getline(aStream, line);

   /* read Frame# and Time */
   readStringFromString(line, buffer);
   readStringFromString(line, buffer);

   /* read the marker names */
   markersRead = 0;
   while (!line.empty())
   {
      if (!readTabDelimitedStringFromString(line, buffer))
         break;
      if (markersRead >= aSMD._numMarkers)
      {
#if 0
         if (gUseGlobalMessages)
            gMessage += "More marker names in TRC file than in model. Ignoring extra marker names.\n";
         break;
#endif
      }
        aSMD._markerNames.append(buffer);
      markersRead++;
   }

    /* If we don't read the header, we'll throw meaningful exception and abort rather than crash the machine!! */
  if (markersRead < aSMD._numMarkers)
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
   /////long pos = aStream.tellg();
   /* read line 5 - coordinate labels (X1 Y1 Z1 X2 Y2 Z2 ...) */
   getline(aStream, line);

   /* The following code supports 0 or 1 blank lines before the first
    * frame of data.  Read the line of code - if there is a blank line of code,
    * read the next line which will contain the data.
    */
   /////getline(aStream, line);
   /////if (line.empty())
      /////getline(aStream, line);

   /////if (!readIntegerFromString(line, &aSMD._firstFrameNumber))
      aSMD._firstFrameNumber = 1;

   /* reposition the pointer into the file so it points to before the coordinate
    * labels */
   /////aStream.seekg(pos, ios::beg);

   /* reread the coordinate labels so pointer into file points to data */
   /////getline(aStream, line);
   /////if (line.empty())
      /////getline(aStream, line);
}

//_____________________________________________________________________________
/**
 * Read TRB file.
 *
 * @param aFileName name of file to read.
 * @param aSMD MarkerData object to hold the file contents
 */
void MarkerData::readTRBFile(const string& aFileName, MarkerData& aSMD)
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

//_____________________________________________________________________________
/**
 * Read sto file.
 *
 * @param aFilename name of sto file.
 */
void MarkerData::readStoFile(const string& aFileName)
{
    if (aFileName.empty())
        throw Exception("MarkerData.readStoFile: ERROR- Marker file name is empty",__FILE__,__LINE__);

    // If the file was written by STOFileAdapter, make the file readable by
    // Storage. Calls below have no effect otherwise.
    std::string tmpFileName{"tmp.sto"};
    bool versionChanged{revertToVersionNumber1(aFileName, tmpFileName)};
    if(versionChanged)
        addNumRowsNumColumns(tmpFileName, aFileName);
    std::remove(tmpFileName.c_str());

    Storage store(aFileName);

    // populate map between marker names and column numbers 
    std::map<int, std::string>  markerIndices;
    buildMarkerMap(store, markerIndices);

    if (markerIndices.size()==0){
        throw Exception("MarkerData.readStoFile: ERROR- No markers were identified. Markers should appear on consecutive columns as Marker1.x Marker1.y Marker1.z Marker2.x... etc.",__FILE__,__LINE__);
    }
    std::map<int, std::string>::iterator iter;

    for (iter = markerIndices.begin(); iter != markerIndices.end(); iter++) {
        SimTK::String markerNameWithSuffix = iter->second;
        size_t dotIndex =
            SimTK::String::toLower(markerNameWithSuffix).find_last_of(".x");
        SimTK::String candidateMarkerName = markerNameWithSuffix.substr(0, dotIndex-1);
        _markerNames.append(candidateMarkerName);
    }
    // use map to populate data for MarkerData header
    _numMarkers = (int) markerIndices.size();
    _numFrames = store.getSize();
    _firstFrameNumber = 1;
    _dataRate = 250;
    _cameraRate = 250;
    _originalDataRate = 250;
    _originalStartFrame = 1;
    _originalNumFrames = _numFrames;
    _fileName = aFileName;
    _units = Units(Units::Meters);

    double time;
    int sz = store.getSize();
    for (int i=0; i < sz; i++){
        StateVector* nextRow = store.getStateVector(i);
        time = nextRow->getTime();
        int frameNum = i+1;
        MarkerFrame *frame = new MarkerFrame(_numMarkers, frameNum, time, _units);
        const Array<double>& rowData = nextRow->getData();
        // Cycle through map and add Marker coordinates to the frame. Same order as header.
        for (iter = markerIndices.begin(); iter != markerIndices.end(); iter++) {
            int startIndex = iter->first; // startIndex includes time but data doesn't!
            frame->addMarker(SimTK::Vec3(rowData[startIndex-1], rowData[startIndex], rowData[startIndex+1]));
        }
        _frames.append(frame);
   }
}
/**
 * Helper function to check column labels of passed in Storage for possibly being a MarkerName, and if true
 * add the start index and corresponding name to the passed in std::map
 */
void MarkerData::buildMarkerMap(const Storage& storageToReadFrom, std::map<int, std::string>& markerNames)
{
    const Array<std::string> & labels = storageToReadFrom.getColumnLabels();
    for (int i=1; i < labels.getSize()-2; i++){
        // if label ends in .X, check that two labels that follow are .Y, .Z (case insensitive) with common prefix
        // if so, add to map
        SimTK::String nextLabel(labels.get(i));
        size_t dotIndex = nextLabel.toLower().find_last_of(".x");
        if (dotIndex > 1){  // possible marker
            SimTK::String candidateMarkerName = nextLabel.substr(0, dotIndex-1);
            // this may be replaced with getColumnIndicesForIdentifier(candidateMarkerName) but this will be more permissive
            // as it allows for non-consecutive columns, could be non-triplet,...etc.
            SimTK::String nextLabel2 = labels.get(i+1);
            SimTK::String nextLabel3 = labels.get(i+2);
            if ((nextLabel2.toLower() == candidateMarkerName+".y") && (nextLabel3.toLower() == candidateMarkerName+".z")){
                markerNames[i] = labels.get(i); // this includes trailing .x
                i+= 2;
            }
        }
    }
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Find the range of frames that is between start time and end time
 * (inclusive).
 *
 * @param aStartTime start time.
 * @param aEndTime end time.
 * @param rStartFrame index of start frame is returned here.
 * @param rEndFrame index of end frame is returned here.
 */
void MarkerData::findFrameRange(double aStartTime, double aEndTime, int& rStartFrame, int& rEndFrame) const
{
    int i;

    rStartFrame = 0;
    rEndFrame = _numFrames - 1;

    if (aStartTime > aEndTime)
    {
        throw Exception("MarkerData: findFrameRange start time is past end time.");
    }

    for (i = _numFrames - 1; i >= 0 ; i--)
    {
        if (_frames[i]->getFrameTime() <= aStartTime)
        {
            rStartFrame = i;
            break;
        }
    }

    for (i = rStartFrame; i < _numFrames; i++)
    {
        if (_frames[i]->getFrameTime() >= aEndTime - SimTK::Zero)
        {
            rEndFrame = i;
            break;
        }
    }
}
//_____________________________________________________________________________
/**
 * Utilities to support the GUI
 *
 * getStartFrameTime: Exposes the time for first frame
 */

double MarkerData::getStartFrameTime() const
{
    if (_numFrames<=0)
        return SimTK::NaN;

    return(_frames[0]->getFrameTime());

}
/**
 * Utilities to support the GUI
 *
 * getLastFrameTime: Expose the time for the last frame
 */
double MarkerData::getLastFrameTime() const
{
    if (_numFrames<=0)
        return SimTK::NaN;

    return(_frames[_numFrames-1]->getFrameTime());
}

//_____________________________________________________________________________
/**
 * Average all the frames between aStartTime and
 * aEndTime (inclusive) and store the result in the first
 * frame. All other frames are deleted. The time and frame
 * number of this one remaining frame are copied from the
 * startIndex frame. The aThreshold parameter is for printing
 * a warning if any marker moves more than that amount in
 * the averaged frames. aThreshold is specified by the user,
 * and is assumed to be in the units of the marker data.
 *
 * @param aThreshold amount of marker movement that is allowed for averaging.
 * @param aStartTime start time of frame range to average.
 * @param aEndTime end time of frame range to average.
 */
void MarkerData::averageFrames(double aThreshold, double aStartTime, double aEndTime)
{
    if (_numFrames < 2)
        return;

    int startIndex = 0, endIndex = 1;
    double *minX = NULL, *minY = NULL, *minZ = NULL, *maxX = NULL, *maxY = NULL, *maxZ = NULL;

    findFrameRange(aStartTime, aEndTime, startIndex, endIndex);
    MarkerFrame *averagedFrame = new MarkerFrame(*_frames[startIndex]);

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
            minX[i] = minY[i] = minZ[i] =  SimTK::Infinity;
            maxX[i] = maxY[i] = maxZ[i] = -SimTK::Infinity;
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
        Vec3& avePt = averagedFrame->updMarker(i);
        avePt = Vec3(0);

        for (int j = startIndex; j <= endIndex; j++)
        {
            Vec3& pt = _frames[j]->updMarker(i);
            if (!pt.isNaN())
            {
                Vec3& coords = pt; //.get();
                avePt += coords;
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
            avePt = Vec3(SimTK::NaN) ;//(SimTK::NaN, SimTK::NaN, SimTK::NaN);
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
            Vec3& pt = _frames[0]->updMarker(i);

            if (pt.isNaN())
            {
                log_warn("Marker {} is missing in frames {} to {}. Coordinate "
                         "will be set to NAN.", _markerNames[i], startUserIndex,
                        endUserIndex);
            }
            else if (maxX[i] - minX[i] > aThreshold ||
                      maxY[i] - minY[i] > aThreshold ||
                      maxZ[i] - minZ[i] > aThreshold)
            {
                double maxDim = maxX[i] - minX[i];
                maxDim = MAX(maxDim, (maxY[i] - minY[i]));
                maxDim = MAX(maxDim, (maxZ[i] - minZ[i]));
                log_warn("Movement of marker {} in {} is {} (threshold = {})",
                        _markerNames[i], _fileName, maxDim, aThreshold);
            }
        }
    }

    log_info("Averaged frames from time {} to {} in {} (frames {} to {})",
            aStartTime, aEndTime, _fileName, startUserIndex, endUserIndex);

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

//_____________________________________________________________________________
/**
 * Store the marker data in an Storage object. The object
 * is emptied before adding the marker data.
 *
 * @param aStorage storage block to fill in with marker data.
 */
void MarkerData::makeRdStorage(Storage& rStorage)
{
    /* First clear any existing frames. */
    rStorage.reset(0);

    /* Make the column labels. */
    Array<string> columnLabels;
    columnLabels.append("time");
    for (int i = 0; i < _numMarkers; i++)
    {
        columnLabels.append(_markerNames[i] + "_tx");
        columnLabels.append(_markerNames[i] + "_ty");
        columnLabels.append(_markerNames[i] + "_tz");
    }
    rStorage.setColumnLabels(columnLabels);

    /* Store the marker coordinates in an array of doubles
     * and add it to the Storage.
     */
    int numColumns = _numMarkers * 3;
    double* row = new double [numColumns];

    for (int i = 0; i < _numFrames; i++)
    {
        for (int j = 0, index = 0; j < _numMarkers; j++)
        {
            SimTK::Vec3& marker = _frames[i]->updMarker(j);
            for (int k = 0; k < 3; k++)
                row[index++] = marker[k];
        }
        rStorage.append(_frames[i]->getFrameTime(), numColumns, row);
    }

    delete [] row;
}

//_____________________________________________________________________________
/**
 * Convert all marker coordinates to the specified units.
 *
 * @param aUnits units to convert to.
 */
void MarkerData::convertToUnits(const Units& aUnits)
{
    double scaleFactor = _units.convertTo(aUnits);

    if (fabs(scaleFactor-1.0)<SimTK::Eps) return;

    if (!SimTK::isNaN(scaleFactor))
    {
        /* Scale all marker locations by the conversion factor. */
        for (int i = 0; i < _frames.getSize(); i++)
            _frames[i]->scale(scaleFactor);

        /* Change the units for this object to the new ones. */
        _units = aUnits;
    }
    else
        throw Exception("MarkerData.convertToUnits: ERROR- Model has unspecified units",__FILE__,__LINE__);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get a frame of marker data.
 *
 * @param aIndex index of the row to get.
 * @return Pointer to the frame of data.
 */
const MarkerFrame& MarkerData::getFrame(int aIndex) const
{
    if (aIndex < 0 || aIndex >= _numFrames)
        throw Exception("MarkerData::getFrame() invalid frame index.");

    return *_frames[aIndex];
}

//_____________________________________________________________________________
/**
 * Get the index of a marker, given its name.
 *
 * @param aName name of marker.
 * @return Index of the named marker.
 */
int MarkerData::getMarkerIndex(const string& aName) const
{
    for (int i = 0; i < _markerNames.getSize(); i++)
    {
        if (_markerNames[i] == aName)
            return i;
    }

    return -1;
}
