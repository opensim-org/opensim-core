/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Storage.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "Storage.h"

#include "CommonUtilities.h"
#include "GCVSpline.h"
#include "GCVSplineSet.h"
#include "IO.h"
#include "Logger.h"
#include "STOFileAdapter.h"
#include "Signal.h"
#include "SimTKcommon.h"
#include "SimmMacros.h"
#include "StateVector.h"
#include "TableUtilities.h"
#include "TimeSeriesTable.h"
#include <iostream>

using namespace OpenSim;
using namespace std;

void convertTableToStorage(const AbstractDataTable* table, Storage& sto)
{
    sto.purge();
    TimeSeriesTable out;

    if (auto td = dynamic_cast<const TimeSeriesTable*>(table))
        // Table is already flattened, so clone for further processing
        out = TimeSeriesTable{ *td };
    else if (auto tst = dynamic_cast<const TimeSeriesTable_<SimTK::Vec2>*>(table))
        out = tst->flatten();
    else if (auto tst = dynamic_cast<const TimeSeriesTable_<SimTK::Vec3>*>(table))
        out = tst->flatten({ "_x", "_y", "_z" });
    else if (auto tst = dynamic_cast<const TimeSeriesTable_<SimTK::Vec4>*>(table))
        out = tst->flatten();
    else if (auto tst = dynamic_cast<const TimeSeriesTable_<SimTK::Vec5>*>(table))
        out = tst->flatten();
    else if (auto tst = dynamic_cast<const TimeSeriesTable_<SimTK::Vec6>*>(table))
        out = tst->flatten();
    else if (auto tst = dynamic_cast<const TimeSeriesTable_<SimTK::Vec7>*>(table))
        out = tst->flatten();
    else if (auto tst = dynamic_cast<const TimeSeriesTable_<SimTK::Vec8>*>(table))
        out = tst->flatten();
    else if (auto tst = dynamic_cast<const TimeSeriesTable_<SimTK::Vec9>*>(table))
        out = tst->flatten();
    else if (auto tst = dynamic_cast<const TimeSeriesTable_<SimTK::Vec<10>>*>(table))
        out = tst->flatten();
    else if (auto tst = dynamic_cast<const TimeSeriesTable_<SimTK::Vec<11>>*>(table))
        out = tst->flatten();
    else if (auto tst = dynamic_cast<const TimeSeriesTable_<SimTK::Vec<12>>*>(table))
        out = tst->flatten();
    else if (auto tst = dynamic_cast<const TimeSeriesTable_<SimTK::UnitVec3>*>(table))
        out = tst->flatten({ "_x", "_y", "_z" });
    else if (auto tst = dynamic_cast<const TimeSeriesTable_<SimTK::Quaternion>*>(table))
        out = tst->flatten();
    else if (auto tst = dynamic_cast<const TimeSeriesTable_<SimTK::SpatialVec>*>(table))
        out = tst->flatten({ "_rx", "_ry", "_rz", "_tx", "_ty", "_tz" });
    else {
        OPENSIM_THROW( STODataTypeNotSupported, typeid(table).name());
    }

    OpenSim::Array<std::string> labels("", (int)out.getNumColumns() + 1);
    labels[0] = "time";
    for (int i = 0; i < (int)out.getNumColumns(); ++i) {
        labels[i + 1] = out.getColumnLabel(i);
    }
    sto.setColumnLabels(labels);

    const auto& times = out.getIndependentColumn();
    for (unsigned i_time = 0; i_time < out.getNumRows(); ++i_time) {
        const SimTK::Vector rowVector =
            out.getRowAtIndex(i_time).transpose().getAsVector();
        sto.append(times[i_time], rowVector);
    }
}


//============================================================================
// DEFINES
//============================================================================


//============================================================================
// CONSTANTS
//============================================================================
//const double Storage::LARGE_NEGATIVE = -1.0e-30;
//const double Storage::LARGE_POSITIVE =  1.0e-30;
const char* Storage::DEFAULT_HEADER_TOKEN = "endheader";
const char* Storage::DEFAULT_HEADER_SEPARATOR = " \t\r\n";
const int Storage::MAX_RESAMPLE_SIZE = 100000;
//============================================================================
// STATICS
//============================================================================
string Storage::simmReservedKeys[] = {
                                  "#",
                                  "range", "Range",
                                  "wrap", "Wrap", "WRAP",
                                  "event",
                                  "enforce_loops",
                                  "enforce_constraints",
                                  "calc_derivatives"};
int numSimmReservedKeys=10; // Keep this number in sync with above array size

// up version to 20301 for separation of RRATool, CMCTool
const int Storage::LatestVersion = 1;

//=============================================================================
// DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 *
 * The stored StateVectors are deleted during destruction.
 */
Storage::~Storage()
{
}

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Storage::Storage(int aCapacity,const string &aName) :
    StorageInterface(aName),
    _storage(StateVector())
{
    // SET NULL STATES
    setNull();

    // CAPACITY
    _storage.ensureCapacity(aCapacity);
    _storage.setCapacityIncrement(-1);

    _fileVersion = Storage::LatestVersion;
    // SET THE STATES
    setName(aName);
}
//_____________________________________________________________________________
/*
 * Construct an Storage instance from file.
 *
 */
Storage::Storage(const string &fileName, bool readHeadersOnly) :
    StorageInterface(fileName),
    _storage(StateVector())
{
    // SET NULL STATES
    setNull();

    // OPEN FILE
    std::unique_ptr<ifstream> fp{IO::OpenInputFile(fileName)};
    OPENSIM_THROW_IF(fp == nullptr, Exception,
            "Storage: Failed to open file '" + fileName +
            "'. Verify that the file exists at the specified location." );

    bool isMotFile = SimTK::String::toLower(fileName).rfind(".mot") != string::npos;
    bool isStoFile = SimTK::String::toLower(fileName).rfind(".sto") != string::npos;
    bool useFileAdpater = true;

    int nr = 0, nc = 0;

    if (isMotFile || isStoFile) {
        parseHeaders(*fp, nr, nc);
        if (_fileVersion <= 1) { // If an old .sto or .mot format
            // Must have valid number of rows and columns
            OPENSIM_THROW_IF(nr < 1 && nc < 1, Exception,
                "Storage: Failed to parse headers of file "
                + fileName);
            // Checks out as a valid old format, so use legacy code to process
            useFileAdpater = false;
        }
    }

    if (useFileAdpater) { // For new .sto files and others that are not .mot
        try {
            // Try using FileAdpater to read all file types
            OPENSIM_THROW_IF(readHeadersOnly, Exception,
                "Cannot read headers only if not a STO file or its "
                "version is greater than 1.");
            auto dataAdapter = FileAdapter::createAdapterFromExtension(fileName);
            FileAdapter::OutputTables tables = dataAdapter->read(fileName);
            if (tables.size() > 1) {
                log_warn(
                        "Storage: cannot read data files with multiple tables. "
                        "Only the first table '{}' will be loaded as Storage.",
                        tables.begin()->first);
            }
            convertTableToStorage(tables.begin()->second.get(), *this);
            return;
        }
        catch (const std::exception& x) {
            log_warn("Storage: FileAdpater failed to read data file.\n{}",
                    x.what());
            if (isStoFile)
                log_warn("Reverting to use conventional Storage reader.");
            else
                throw x;
        }
    }

    // Process file as if it were a .mot file
    log_info("Storage: read data file = {} (nr={} nc={})", fileName, nr, nc);

    // Motion files from SIMM are in degrees
    if (_fileVersion < 1 && isMotFile) {
        _inDegrees = true;
    }
    if (_fileVersion < 1) {
        log_info(".. assuming rotations in {}.",
                _inDegrees ? "Degrees." : "Radians.");
    }

    // IGNORE blank lines after header -- treat \r and \n as end of line chars
    while(fp->good()) {
        int c = fp->peek();
        if(c!='\n' && c!='\r' && c!='\t' && c!=' ') break;
        fp->get();
    }
    // Ayman: Support for oldStyleDescription dropped. 04/11/07

    // COLUMN LABELS
    string line;
    getline(*fp, line);
    parseColumnLabels(line.c_str());

    if (_columnLabels.getSize()!= nc){
        log_warn("Storage: Inconsistent headers in file {}. nColumns={} but {} "
                 "were found",
                fileName, nc, _columnLabels.getSize());
    }
    // CAPACITY
    _storage.ensureCapacity(nr);
    _storage.setCapacityIncrement(-1);

    // There are situations where we don't want to read the whole file in advance just header
    if (readHeadersOnly) return;

    //MM using the occurrence of time and range in the column labels to distinguish between
    //SIMM and non SIMMOtion files.
    Array<std::string> currentLabels = getColumnLabels();
    int indexTime = currentLabels.findIndex("time");
    int indexRange = currentLabels.findIndex("range");


    // DATA
    if(indexTime != -1 || indexRange != -1){ //MM edit
        int ny = nc-1;
        double time;
        double *y = new double[ny];
        for(int r=0;r<nr;r++) {
                (*fp)>>time;
                for(int i=0;i<ny;i++)
                    (*fp)>>y[i];
                append(time,ny,y);
        }
        delete[] y;
    }else{  //MM the modifications below are to make the Storage class
            //well behaved when it is given data that does not contain a
            //time or a range column
        int ny = nc;
        double time;
        double *y = new double[ny];
        for(int r=0;r<nr;r++) {
                time=(double)r;
                for(int i=0;i<ny;i++)
                    (*fp)>>y[i];
                append(time,ny,y);
        }
        delete[] y;
    }
    // If what we read was really a sIMM motion file, adjust the data
    // to account for different assumptions between SIMM.mot OpenSim.sto

    //MM if this is a SIMM Motion file, post process it as one. Else don't touch the data
    if(indexTime == -1){
        postProcessSIMMMotion();
    }
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 */
Storage::Storage(const Storage &aStorage,bool aCopyData) :
    StorageInterface(aStorage),
    _storage(StateVector())
{
    // NULL THE DATA
    setNull();

    // CAPACITY
    _storage.ensureCapacity(aStorage._storage.getCapacity());
    _storage.setCapacityIncrement(aStorage._storage.getCapacityIncrement());

    // SET STATES
    setName(aStorage.getName());
    setDescription(aStorage.getDescription());
    setHeaderToken(aStorage.getHeaderToken());
    setColumnLabels(aStorage.getColumnLabels());
    setStepInterval(aStorage.getStepInterval());
    setInDegrees(aStorage.isInDegrees());
    _fileVersion = aStorage._fileVersion;
    // COPY STORED DATA
    if(aCopyData) copyData(aStorage);
}
//_____________________________________________________________________________
/**
 * Construct a copy of a specified storage taking only a subset of the states.
 *
 * @param aStorage Storage to be copied.
 * @param aStateIndex Index of the state (column) at which to start the copy.
 * @param aN Number of states to copy.
 * @param aDelimiter Delimiter used to separate state labels (i.e., column
 * labels).  The delimiter is assumed to be a tab by default.
 */
Storage::
Storage(const Storage &aStorage,int aStateIndex,int aN,
             const char *aDelimiter) :
     StorageInterface(aStorage),
    _storage(StateVector())
{
    // NULL THE DATA
    setNull();

    // CAPACITY
    _storage.ensureCapacity(aStorage._storage.getCapacity());
    _storage.setCapacityIncrement(aStorage._storage.getCapacityIncrement());

    // SET STATES
    setName(aStorage.getName());
    setDescription(aStorage.getDescription());
    setHeaderToken(aStorage.getHeaderToken());
    setStepInterval(aStorage.getStepInterval());
    setInDegrees(aStorage.isInDegrees());
    _fileVersion = aStorage._fileVersion;
    // ERROR CHECK
    if(aStateIndex<0) return;
    if(aN<=0) return;

    // SET THE DATA
    int i,n;
    double time,*data = new double[aN];
    for(i=0;i<aStorage._storage.getSize();i++) {
        aStorage.getTime(i,time);
        n = aStorage.getData(i,aStateIndex,aN,data);
        append(time,n,data);
    }
    delete[] data;

    // COPY THE FIRST COLUMN AND COLUMNS aStateIndex+1 through aStateIndex+aN (corresponding to states aStateIndex - aStateIndex+aN-1)
    int originalNumCol = aStorage.getColumnLabels().getSize();
    _columnLabels.setSize(0);
    if(originalNumCol) {
        _columnLabels.append(aStorage.getColumnLabels()[0]);
        for(int i=0;i<aN && aStateIndex+1+i<originalNumCol;i++)
            _columnLabels.append(aStorage.getColumnLabels()[aStateIndex+1+i]);
    }
}


/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
Storage& Storage::operator=(const Storage &aStorage)
{
    // BASE CLASS
    StorageInterface::operator=(aStorage);

    // Copy Members
    copyData(aStorage);
    return(*this);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all states to their null or default values.
 */
void Storage::
setNull()
{
    _writeSIMMHeader = false;
    setHeaderToken(DEFAULT_HEADER_TOKEN);
    _stepInterval = 1;
    _lastI = 0;
    _fp = 0;
    _inDegrees = false;
}
//_____________________________________________________________________________
/**
 * Copy the data stored by another storage instance.
 *
 * Note that this method only copies the stored data.  It does not copy
 * other members of aStorage such as the name and the description.  To get
 * a complete copy, the copy constructor should be used.
 *
 * If this instance does not have enough capacity to hold the states
 * of the specified storage (aStorage), the capacity is increased.
 */
void Storage::
copyData(const Storage &aStorage)
{
    _units = aStorage._units;
    setInDegrees(aStorage.isInDegrees());

    // ENSURE CAPACITY
    _storage.ensureCapacity(aStorage._storage.getCapacity());

    // COPY
    //rdPtrArray::reset();
    _storage.setSize(0);
    for(int i=0;i<aStorage._storage.getSize();i++) {
        _storage.append(aStorage._storage[i]);
    }
}



//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// SIMM HEADER
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the whether or not to write a header appropriate for a SIMM motion
 * file.
 *
 * @param aTrueFalse Whether (true) or not (false) to write a SIMM header.
 */
void Storage::
setWriteSIMMHeader(bool aTrueFalse)
{
    _writeSIMMHeader = aTrueFalse;
    if (_writeSIMMHeader) setInDegrees(true);
}
//_____________________________________________________________________________
/**
 * Get the whether or not to write a header appropriate for a SIMM motion
 * file.
 *
 * @param aTrueFalse Whether (true) or not (false) to write a SIMM header.
 */
bool Storage::
getWriteSIMMHeader() const
{
    return(_writeSIMMHeader);
}

//-----------------------------------------------------------------------------
// HEADER TOKEN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the header token.
 * The header token is used to mark the end of the header
 * portion of an Storage when an Storage is saved in a file.
 *
 * If the header token is NULL, a default header token is used.
 *
 * @param aToken Header token.
 */
void Storage::
setHeaderToken(const std::string &aToken)
{
    if(aToken.empty()) _headerToken = DEFAULT_HEADER_TOKEN;
    else _headerToken = aToken;
}
//_____________________________________________________________________________
/**
 * Get the header token of this storage.
 *
 * @return Header token.
 * @see setHeaderToken()
 */
const string &Storage::
getHeaderToken() const
{
    return(_headerToken);
}

//-----------------------------------------------------------------------------
// COLUMN LABELS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
// added a default Parameter for startIndex. -Ayman
// TODO startIndex is being ignored.
int Storage::
getStateIndex(const std::string &aColumnName, int startIndex) const
{
    int thisColumnIndex =
            TableUtilities::findStateLabelIndex(_columnLabels, aColumnName);
    if (thisColumnIndex == -1) {
        return -1;
    }
    // Subtract 1 because time is included in the labels but not
    // in the "state vector".
    return thisColumnIndex - 1;
}


//_____________________________________________________________________________
/**
 * Set a labels string for the columns in this Storage instance.
 *
 * A character string is used to label the columns.  Each separate column
 * label is usually delimited by a tab ("\t"), but any delimiter may
 * be used.
 *
 * The first column is almost always "Time."  The other columns
 * correspond to the separate elements of a state vector (StateVector).
 *
 * If the labels string is set to NULL, the following default labels
 * will be used when the Storage instance is saved to file:
 *
 * time state_0 state_1 state_2 ...
 *
 * @param aLabels Character string containing labels for the columns.
 */
void Storage::
parseColumnLabels(const char *aLabels)
{
    _columnLabels.setSize(0);

    // HANDLE NULL POINTER
    if(aLabels==NULL) return;

    // HANDLE ZERO LENGTH STRING
    int len = (int)strlen(aLabels);
    if(len==0) return;

    // SET NEW
    char *labelsCopy = new char[len+1];
    if(aLabels[len-1]=='\n') {
        // strip carriage return
        strncpy(labelsCopy,aLabels,len-1);
        labelsCopy[len-1] = 0;
    } else {
        strcpy(labelsCopy,aLabels);
    }

    // Parse
    char *token = strtok(labelsCopy,DEFAULT_HEADER_SEPARATOR );
    while(token!=NULL)
    {
        // Append column label
        _columnLabels.append(token);

        // Get next label
        token = strtok( NULL, DEFAULT_HEADER_SEPARATOR );
    }

    delete[] labelsCopy;
}

//_____________________________________________________________________________
/**
 * Set column labels array.
 */
void Storage::
setColumnLabels(const Array<std::string> &aColumnLabels)
{
    _columnLabels = aColumnLabels;
}

//_____________________________________________________________________________
/**
 * Get column labels array.
 *
 * @return Character string of column labels.
 */
const Array<string> &Storage::
getColumnLabels() const
{
    return(_columnLabels);
}

//-----------------------------------------------------------------------------
// STEP INTERVAL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the step interval.
 */
void Storage::
setStepInterval(int aStepInterval)
{
    _stepInterval = aStepInterval;
    if(_stepInterval<0) _stepInterval = 0;
}
//_____________________________________________________________________________
/**
 * Get the step interval.
 */
int Storage::
getStepInterval() const
{
    return(_stepInterval);
}

//-----------------------------------------------------------------------------
// CAPACITY INCREMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the capacity increment of this storage object.  For details on what
 * the capacity increment does see Array::setCapacityIncrement().
 *
 * @param aIncrement Capacity increment.
 * @see Array
 */
void Storage::
setCapacityIncrement(int aIncrement)
{
    _storage.setCapacityIncrement(aIncrement);
}
//_____________________________________________________________________________
/**
 * Get the capacity increment of this storage object.
 *
 * @return Capacity increment of this storage.  For details on what
 * the capacity increment does see Array::setCapacityIncrement().
 */
int Storage::
getCapacityIncrement() const
{
    return(_storage.getCapacityIncrement());
}

//-----------------------------------------------------------------------------
// STATEVECTORS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the smallest number of states.
 *
 * Ordinarily, the number of states is the same in each state vector;
 * however, this is not required.
 * @return Smallest number of states.
 */
int Storage::
getSmallestNumberOfStates() const
{
    int n,nmin=0;
    for(int i=0;i<_storage.getSize();i++) {
        n = _storage[i].getSize();
        if(i==0) {
            nmin = n;
        } else if(n<nmin) {
            nmin = n;
        }
    }

    return(nmin);
}
//_____________________________________________________________________________
/**
 * Get the last states stored.
 *
 * @return Statevector.  If no state vector is stored, NULL is returned.
 */
StateVector* Storage::
getLastStateVector() const
{
    StateVector *vec = NULL;
    try {
        vec = &_storage.updLast();
    } catch(const Exception&) {
        //x.print(cout);
    }
    return(vec);
}
//_____________________________________________________________________________
/**
 * Get the StateVector at a specified time index.
 *
 * @param aTimeIndex Time index at which to get the state vector:
 * 0 <= aTimeIndex < _storage.getSize().
 * @return Statevector. If no valid statevector exists at aTimeIndex, NULL
 * is returned.
 */
StateVector* Storage::
getStateVector(int aTimeIndex) const
{
    return(&_storage.updElt(aTimeIndex));
}

//-----------------------------------------------------------------------------
// TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the time of the first stored states.
 *
 * @return Time of the first stored states.  If there is no stored state,
 * the constant SimTK::NaN (not a number) is returned.
 */
double Storage::
getFirstTime() const
{
    if(_storage.getSize()<=0) {
        return(SimTK::NaN);
    }
    return(_storage[0].getTime());
}
//_____________________________________________________________________________
/**
 * Get the time of the last states.
 *
 * @return Time of the first stored states.  If there is no stored state,
 * the constant SimTK::NaN (not a number) is returned.
 */
double Storage::
getLastTime() const
{
    if(_storage.getSize()<=0) {
        return(SimTK::NaN);
    }
    return(_storage.getLast().getTime());
}
//_____________________________________________________________________________
/**
 * Get the smallest time step.
 *
 * @return Smallest time step.  If there are less than 2 state vectors,  SimTK::Infinity  is returned.
 */
double Storage::
getMinTimeStep() const
{
    double *time=NULL;
    int n = getTimeColumn(time);
    double dtmin =  SimTK::Infinity;
    for(int i=1; i<n; i++) {
        double dt = time[i] - time[i-1];
        if(dt<dtmin) dtmin = dt;
    }
    delete[] time;
    return dtmin;
}
//_____________________________________________________________________________
/**
 * Get the time at a specified time index for a specified state.
 *
 * @param aTimeIndex Time index (row) for which to get the time.
 * @param rTime Time value.
 * @param aStateIndex Index of the state for which to get the time.
 * By default, aStateIndex has a value of -1, which means disregard whether
 * or not there is a valid state- just get the time at aTimeIndex.  If
 * aStateIndex is non-negative, the time is returned only if there is a valid
 * state at aStateIndex.
 * @return true when the time was set, false when there was no valid state.
 */
bool Storage::
getTime(int aTimeIndex,double &rTime,int aStateIndex) const
{
    if(aTimeIndex<0) return false;
    if(aTimeIndex>_storage.getSize()) return false;

    // GET STATEVECTOR
    StateVector &vec = _storage[aTimeIndex];

    // CHECK FOR VALID STATE
    if(aStateIndex >= vec.getSize()) return false;

    // ASSIGN TIME
    rTime = vec.getTime();
    return true;
}
//_____________________________________________________________________________
/**
 * Get the times for a specified state.
 *
 * @param rTime Array where times are set.  If rTime is sent in as NULL,
 * memory is allocated.  If rTime is setn in as non-NULL, it is assumed that
 * enough memory has been allocated at rTime to hold _storage.getSize() doubles.
 * @param aStateIndex Index of the state for which to get the times.
 * By default, aStateIndex has a value of -1, which means disregard whether
 * or not there is a valid state- just get the times.  If aStateIndex is
 * non-negative, the time is set only if there is a valid state at aStateIndex.
 * @return Number of times set.  This can be less than _storage.getSize() if
 * a state does not exist for all or a subset of the stored statevectors.
 */
int Storage::
getTimeColumn(double *&rTimes,int aStateIndex) const
{
    if(_storage.getSize()<=0) return(0);

    // ALLOCATE MEMORY
    if(rTimes==NULL) {
        rTimes = new double[_storage.getSize()];
    }

    // LOOP THROUGH STATEVECTORS
    int i,nTimes;
    StateVector *vec;
    for(i=nTimes=0;i<_storage.getSize();i++) {
        vec = getStateVector(i);
        if(vec==NULL) continue;
        if(aStateIndex >= vec->getSize()) continue;
        rTimes[nTimes] = vec->getTime();
        nTimes++;
    }

    return(nTimes);
}
int Storage::
getTimeColumn(Array<double> &rTimes,int aStateIndex) const
{
    if(_storage.getSize()<=0) return(0);

    rTimes.setSize(_storage.getSize());

    // LOOP THROUGH STATEVECTORS
    int i,nTimes;
    for(i=nTimes=0;i<_storage.getSize();i++) {
        StateVector *vec = getStateVector(i);
        if(vec==NULL) continue;
        if(aStateIndex >= vec->getSize()) continue;
        rTimes[nTimes] = vec->getTime();
        nTimes++;
    }

    rTimes.setSize(nTimes);

    return(nTimes);
}
/**
 * Get the time column starting at aTime. Return it in rTimes
 * rTimes is preallocated by the caller.
 */
void Storage::
getTimeColumnWithStartTime(Array<double>& rTimes,double aStartTime) const
{
    if(_storage.getSize()<=0) return;

    int startIndex = findIndex(aStartTime);

    double *timesVec=0;
    int size = getTimeColumn(timesVec);

    for(int i=startIndex; i<size; i++)
        rTimes.append(timesVec[i]);
    delete[] timesVec;
}
//-----------------------------------------------------------------------------
// DATA
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get a data value of a specified state at a specified time index.
 *
 * @param aTimeIndex Index that identifies the time (row) at which to get the
 * data value:  0 <= aTimeIndex < _storage.getSize().
 * @param aStateIndex Index of the state (column) for which to get the value.
 * @param rValue Value of the state.
 * @return 1 on success, 0 on failure.
 */
int Storage::
getData(int aTimeIndex,int aStateIndex,double &rValue) const
{
    if(aTimeIndex<0) return(0);
    if(aTimeIndex>=_storage.getSize()) return(0);

    // ASSIGNMENT
    StateVector *vec = getStateVector(aTimeIndex);
    if(vec==NULL) return(0);
    return( vec->getDataValue(aStateIndex,rValue) );
}
//_____________________________________________________________________________
/**
 * Helper function for getData
 */
int Storage::
getData(int aTimeIndex,int aStateIndex,int aN,double **rData) const
{
    if(aN<=0) return(0);
    if(aStateIndex<0) return(0);
    if(aTimeIndex<0) return(0);
    if(aTimeIndex>=_storage.getSize()) return(0);

    // GET STATEVECTOR
    StateVector *vec = getStateVector(aTimeIndex);
    if(vec==NULL) return(0);
    if(vec->getSize()<=0) return(0);

    // NUMBER OF STATES TO GET
    int size = vec->getSize();
    if(aStateIndex>=size) return(0);
    int n = aStateIndex + aN;
    if(n>size) n = size;
    int N = n - aStateIndex;

    // ALLOCATE MEMORY
    if(*rData==NULL) *rData = new double[N];

    // ASSIGN DATA
    int i,j;
    Array<double> &data = vec->getData();
    double *pData = *rData;
    for(i=0,j=aStateIndex;j<n;i++,j++) pData[i] = data[j];

    return(N);
}
//_____________________________________________________________________________
/**
 * At a specified time index, get a number of state values starting at
 * a specified state.  The method simply gets part of a row of data
 * from adjacent columns in the storage object.
 *
 * @param aTimeIndex Index that identifies the time (row) at which to get the
 * data value:  0 <= aTimeIndex < _storage.getSize().
 * @param aStateIndex Index of the state (column) at which to start getting
 * the data.
 * @param aN Number of states (columns) to get.
 * @param rData Data values. rData should be able to hold at least N values.
 * @return Number of states that were gotten.
 */
int Storage::
getData(int aTimeIndex,int aStateIndex,int aN,double *rData) const
{
    if(rData==NULL) return(0);
    else return getData(aTimeIndex,aStateIndex,aN,&rData);
}
//_____________________________________________________________________________
/**
 * At a specified time index, get a number of state values starting at
 * a specified state.  The method simply gets part of a row of data
 * from adjacent columns in the storage object.
 *
 *  @param aTimeIndex Time index at which to get the states.
 * @param aStateIndex Index of the state (column) at which to start getting
 * the data.
 * @param aN Number of states to get.
 * @param rData Pointer to an array where the returned data will be set.  The
 * size of *rData is assumed to be at least aN.  If rData comes in as NULL,
 * memory is allocated.
 * @return Number of states.
 */
int Storage::
getData(int aTimeIndex,int aN,double **rData) const
{
    return getData(aTimeIndex,0,aN,rData);
}
//_____________________________________________________________________________
/**
 * Get the first aN states at a specified time index.
 *
 *  @param aTimeIndex Time index at which to get the states.
 * @param aN Number of states to get.
 * @param rData Array where the returned data will be set.  The
 * size of rData is assumed to be at least aN.
 * @return Number of states that were set.
 */
int Storage::
getData(int aTimeIndex,int aN,double *rData) const
{
    if(rData==NULL) return(0);
    else return getData(aTimeIndex,0,aN,&rData);
}
//_____________________________________________________________________________
/**
 * Get the first aN states at a specified time index.
 *
 *  @param aTimeIndex Time index at which to get the states.
 * @param aN Number of states to get.
 * @param rData Array where the returned data will be set.  The
 * size of rData is assumed to be at least aN.
 * @return Number of states that were set.
 */
int Storage::
getData(int aTimeIndex,int aN,Array<double> &rData) const
{
    if(0 == rData.size()) return(0);
    else return getData(aTimeIndex,0,aN,&rData[0]);
}
//_____________________________________________________________________________
/**
 * Get the first aN states at a specified time index.
 *
 *  @param aTimeIndex Time index at which to get the states.
 * @param aN Number of states to get.
 * @param rData Array where the returned data will be set.  The
 * size of rData is assumed to be at least aN.
 * @return Number of states that were set.
 */
int Storage::
getData(int aTimeIndex,int aN,SimTK::Vector& v) const
{
    Array<double> rData;
    rData.setSize(aN);
    int r = getData(aTimeIndex,0,aN,&rData[0]);
    for (int i=0; i<aN; ++i)
        v[i] = rData[i];
    return r;
}
//_____________________________________________________________________________
/**
 * Get the first aN states at a specified time.
 * The values of the states are determined by linear interpolation.
 *
 *  @param aT Time at which to get the states.
 * @param aN Number of states to get.
 * @param rData Pointer to an array where the returned data will be set.  The
 * size of *rData is assumed to be at least aN.  If rData comes in as NULL,
 * memory is allocated.
 * @return Number of states that were set.
 */
int Storage::
getDataAtTime(double aT,int aN,double **rData) const
{

    // FIND THE CORRECT INTERVAL FOR aT
    int i = findIndex(_lastI,aT);
    if((i<0)||(_storage.getSize()<=0)) {
        *rData = NULL;
        return(0);
    }

    // CHECK FOR i AT END POINTS
    int i1=i,i2=i+1;

    if(i2==_storage.getSize()) {
        i1--;  if(i1<0) i1=0;
        i2--;  if(i2<0) i2=0;
    }

    // STATES AT FIRST INDEX
    int n1 = getStateVector(i1)->getSize();
    double t1 = getStateVector(i1)->getTime();
    Array<double> &y1 = getStateVector(i1)->getData();

    // STATES AT NEXT INDEX
    int n2 = getStateVector(i2)->getSize();
    double t2 = getStateVector(i2)->getTime();
    Array<double> &y2 = getStateVector(i2)->getData();

    // GET THE SMALLEST N TO PREVENT MEMORY OVER-RUNS
    int ns = (n1<n2) ? n1 : n2;

    // ALLOCATE MEMORY?
    double *y;
    if(*rData==NULL) {
        y = new double[ns];
    } else {
        y = *rData;
        if(aN<ns)  ns = aN;
    }

    // ASSIGN VALUES
    double pct;
    double num = aT-t1;
    double den = t2-t1;
    if(den<SimTK::Eps) {
        pct = 0.0;
    } else {
        pct = num/den;
    }


    for(i=0;i<ns;i++) {
        if(pct==0.0) {
            y[i] = y1[i];
        } else {
            y[i] = y1[i] + pct*(y2[i]-y1[i]);
        }
    }

    // ASSIGN FOR RETURN
    *rData = y;

    return(ns);
}
//_____________________________________________________________________________
/**
 * Get the first aN states at a specified time.
 * The values of the states are determined by linear interpolation.
 *
 *  @param aT Time at which to get the states.
 * @param aN Number of states to get.
 * @param rData Array where the returned data will be set.  The
 * size of rData is assumed to be at least aN.
 * @return Number of states that were set.
 */
int Storage::
getDataAtTime(double aT,int aN,double *rData) const
{
    if(rData==NULL) return(0);
    else return getDataAtTime(aT,aN,&rData);
}
int Storage::
getDataAtTime(double aT,int aN,Array<double> &rData) const
{
    double *data=&rData[0];
    return getDataAtTime(aT,aN,&data);
}
int Storage::
getDataAtTime(double aT,int aN,SimTK::Vector& v) const
{
    Array<double> rData;
    rData.setSize(aN);
    int r = getDataAtTime(aT,aN,rData);
    for (int i=0; i<aN; ++i)
        v[i] = rData[i];
    return r;
}
//_____________________________________________________________________________
/**
 * Get the data corresponding to a specified state.  This call is equivalent
 * to getting a column of data from the storage file.
 *
 * @param aStateIndex Index of the state (column) for which to get the data.
 * @param rData Array containing the desired data.  If rData is sent in as
 * NULL, memory is allocated.  However, if rData is sent in as a non-NULL, it
 * is assumed that rData points to a memory block that is large enough to
 * hold getSize() doubles.
 * @return Number of values set in rData.  The number of values set may be
 * less than getSize() because not all stored state vectors are
 * required to have the same number of states.
 */
int Storage::
getDataColumn(int aStateIndex,double *&rData) const
{
    int n = _storage.getSize();
    if(n<=0) return(0);

    // ALLOCATION
    if(rData==NULL) {
        rData = new double[n];
    }

    // ASSIGNMENT
    int i,nData;
    for(i=nData=0;i<n;i++) {
        StateVector *vec = getStateVector(i);
        if(vec==NULL) continue;
        if(vec->getDataValue(aStateIndex,rData[nData])) nData++;
    }

    return(nData);
}
int Storage::
getDataColumn(int aStateIndex,Array<double> &rData) const
{
    int n = _storage.getSize();
    if(n<=0) return(0);

    rData.setSize(n);

    // ASSIGNMENT
    int i,nData;
    for(i=nData=0;i<n;i++) {
        StateVector *vec = getStateVector(i);
        if(vec==NULL) continue;
        if(vec->getDataValue(aStateIndex,rData[nData])) nData++;
    }

    rData.setSize(nData);

    return(nData);
}

/**
 * Get the data column starting at aTime. Return it in rData
 * rData is preallocated by the caller.
 */
void Storage::
getDataColumn(const std::string& columnName, Array<double>& rData, double aStartTime)
{
    if(_storage.getSize()<=0) return;

    int startIndex = findIndex(aStartTime);
    int colIndex = getStateIndex(columnName);
    double *dataVec=0;
    getDataColumn(colIndex, dataVec);
    for(int i=startIndex; i<_storage.getSize(); i++)
        rData.append(dataVec[i]);
    delete[] dataVec;
}

//_____________________________________________________________________________
/**
 * Set the data corresponding to a specified state.  This call is equivalent
 * to setting a column of data from the storage file.
 *
 * @param aStateIndex Index of the state (column) for which to get the data.
 * @param aData Array containing the new data.
 * @return Number of values set in rData.  The number of values set may be
 * less than getSize() because not all stored state vectors are
 * required to have the same number of states.
 */
void Storage::
setDataColumn(int aStateIndex,const Array<double> &aData)
{
    int n = _storage.getSize();
    if(n!=aData.getSize()) {
        log_error("Storage.setDataColumn: sizes don't match.");
        return;
    }

    // ASSIGNMENT
    for(int i=0;i<n;i++) {
        StateVector *vec = getStateVector(i);
        if(vec==NULL) continue;
        vec->setDataValue(aStateIndex,aData[i]);
    }
}
/**
 * set values in the column specified by columnName to newValue
 */
void Storage::setDataColumnToFixedValue(const std::string& columnName, double newValue) {
    int n = _storage.getSize();
    int aStateIndex = getStateIndex(columnName);
    if(aStateIndex==-1) {
        log_error("Storage.setDataColumnToFixedValue: column not found.");
        return;
    }

    // ASSIGNMENT
    for(int i=0;i<n;i++) {
        StateVector *vec = getStateVector(i);
        if(vec==NULL) continue;
        vec->setDataValue(aStateIndex,newValue);
    }

}

//_____________________________________________________________________________
/**
 * Get the data corresponding to a state specified by name.  This call is equivalent
 * to getting a column of data from the storage file.
 *
 * @param aColumnName name in header of column for which to get the data.
 * @param rData Array containing the desired data.  If rData is sent in as
 * NULL, memory is allocated.  However, if rData is sent in as a non-NULL, it
 * is assumed that rData points to a memory block that is large enough to
 * hold getSize() doubles.
 * @return Number of values set in rData.  The number of values set may be
 * less than getSize() because not all stored state vectors are
 * required to have the same number of states.
 */
int Storage::
getDataColumn(const std::string& aColumnName,double *&rData) const
{
    if (aColumnName.c_str()[0]=='#'){
        int columnNumber=-1;
        sscanf(aColumnName.c_str(), "#%d", &columnNumber);
        return getDataColumn(columnNumber-1, rData);
    }
    else
        return getDataColumn(getStateIndex(aColumnName), rData);
}

/** It is desirable to access the block as a single entity provided an identifier that is common
    to all components (such as prefix in the column label).
     @param identifier  string identifying a single block of data
     @param rData       Array<Array<double>> of data belonging to the identifier */
void Storage::getDataForIdentifier(const std::string& identifier, Array<Array<double> >& rData, double startTime) const
{

    Array<int> found = getColumnIndicesForIdentifier(identifier);

    if(found.getSize() == 0){
        log_warn("Storage {} could not locate data for identifier {}.",
                getName(), identifier);
        return;
    }
    /* a row of "data" can be shorter than number of columns if time is the first column, since
       that is not considered a state by storage. Need to fix this! -aseth */
    int nd = getLastStateVector()->getSize();
    int off = _columnLabels.getSize()-nd;


    for(int i=0; i<found.getSize(); ++i){
        Array<double> data{};
        getDataColumn(found[i]-off, data);
        rData.append(data);
    }
}

OpenSim::Array<int>
Storage::getColumnIndicesForIdentifier(const std::string& identifier) const
{
    Array<int> found;
    const size_t lid = identifier.length();

    if (lid < 1)  // Identifier is empty; return empty Array.
        return found;

    for (int i = 0; i < _columnLabels.getSize(); ++i) {
        if (_columnLabels[i].compare(0,lid, identifier) == 0)
            found.append(i);
    }
    return found;
}

TimeSeriesTable Storage::exportToTable() const {
    TimeSeriesTable table{};

    table.addTableMetaData("header", getName());
    table.addTableMetaData("inDegrees", std::string{_inDegrees ? "yes" : "no"});
    table.addTableMetaData("nRows", std::to_string(_storage.getSize()));
    table.addTableMetaData("nColumns", std::to_string(_columnLabels.getSize()));
    if(!getDescription().empty())
        table.addTableMetaData("description", getDescription());

    // Exclude the first column label. It is 'time'. Time is a separate column
    // in TimeSeriesTable and column label is optional.
    if (_columnLabels.size() > 1) {
        table.setColumnLabels(_columnLabels.get() + 1,
                _columnLabels.get() + _columnLabels.getSize());
    }

    for(int i = 0; i < _storage.getSize(); ++i) {
        const auto& row = getStateVector(i)->getData();
        const auto time = getStateVector(i)->getTime();
        // Exclude the first column. It is 'time'. Time is a separate column in
        // TimeSeriesTable.
        table.appendRow(time, row.get(), row.get() + row.getSize());
    }

    return table;
}


//=============================================================================
// RESET
//=============================================================================
//_____________________________________________________________________________
/**
 * Reset the storage to start storing at a specified index.  All
 * statevectors at and following the specified index are discarded.
 *
 * If aIndex is less than or equal to zero, the storage object is
 * emptied (i.e., its size is set to 0).
 *
 * If aIndex is larger than the current size of the storage object, no
 * action is taken.
 *
 * @param aIndex Index at which to start storing new statevectors.
 * @return Index at which the next appended statevector will be stored.
 */
int Storage::
reset(int aIndex)
{
    if(aIndex>=_storage.getSize()) return(_storage.getSize());
    if(aIndex<0) aIndex = 0;
    _storage.setSize(aIndex);

    return(_storage.getSize());
}
//_____________________________________________________________________________
/**
 * Reset the storage to start storing after a specified time.  If no valid
 * statevector exists at that specified time, the storage is set to occur after
 * the first valid statevector that immediately precedes the specified
 * time.
 *
 * @param aT Time after which to start storing states.  If aT doesn't exist
 * exactly in the storage, the time is rounded down to the first valid time.
 * @return Index at which the next appended statevector will be stored.
 */
int Storage::
reset(double aTime)
{
    // 1 is added so that the states at or just prior to aT are kept.
    int index = findIndex(aTime) + 1;

    return( reset(index) );
}


//_____________________________________________________________________________
/**
 * Crop the storage object to the specified start and final time
 */
void Storage::
crop(const double newStartTime, const double newFinalTime)
{
    int startindex = findIndex(newStartTime);
    int finalindex = findIndex(newFinalTime);
    // Since underlying Array is packed we'll just move what we need up then
    // delete remaining rows in reverse order.
    int numRowsToKeep=finalindex-startindex+1;
    if (numRowsToKeep <=0){
        log_warn("Storage.crop: No rows will be left.");
        numRowsToKeep=0;
    }
    if (startindex!=0){
        for(int i=0; i<finalindex-startindex+1; i++)
            _storage[i]=_storage[startindex+i];
    }
    _storage.setSize(numRowsToKeep);
}

//=============================================================================
// STORAGE
//=============================================================================
//_____________________________________________________________________________
/**
 * Append an StateVector.
 *
 * @param aStateVector Statevector to be appended.
 * @return Size of the storage after the append.
 */
int Storage::
append(const StateVector &aStateVector,bool aCheckForDuplicateTime)
{
    // TODO: use some tolerance when checking for duplicate time?
    if(aCheckForDuplicateTime && _storage.getSize() && _storage.getLast().getTime()==aStateVector.getTime())
        _storage.updLast() = aStateVector;
    else
        _storage.append(aStateVector);

    if (_fp!=0){
        aStateVector.print(_fp);
        fflush(_fp);
    }
    return(_storage.getSize());
}
//_____________________________________________________________________________
/**
 * Append copies of all state vectors in an Storage object.
 *
 * This method overrides Array::append(Array).
 * Currently, there is no difference.  The override is done for completeness.
 *
 * @param aStorage Storage to be appended.
 * @return The index of the first empty storage element.
 */
int Storage::
append(const Array<StateVector> &aStorage)
{
    for(int i=0; i<aStorage.getSize(); i++)
        _storage.append(aStorage[i]);
    return(_storage.getSize());
}
//_____________________________________________________________________________
/**
 * Append an array of data that occurred at a specified time.
 *
 * @param aT Time stamp of the data.
 * @param aN Length of the array.
 * @param aY Array.
 * @return Index of the first empty storage element.
 */
int Storage::
append(double aT,int aN,const double *aY,bool aCheckForDuplicateTime)
{
    if(aY==NULL) return(_storage.getSize());
    if(aN<0) return(_storage.getSize());

    // APPEND
    StateVector vec(aT, SimTK::Vector_<double>(aN, aY));
    append(vec,aCheckForDuplicateTime);
    // TODO: use some tolerance when checking for duplicate time?
    /*
    if(aCheckForDuplicateTime && _storage.getSize() && _storage.getLast().getTime()==vec.getTime())
        _storage.getLast() = vec;
    else
        _storage.append(vec);
    */
    return(_storage.getSize());
}
//_____________________________________________________________________________
/**
 * Append an array of data that occurred at a specified time.
 *
 * @param aT Time stamp of the data.
 * @param aY Vector.
 * @return Index of the first empty storage element.
 */
int Storage::
append(double aT,const SimTK::Vector& aY,bool aCheckForDuplicateTime)
{
    // APPEND
    return( append ( aT, aY.size(), &aY[0], aCheckForDuplicateTime ));
}
//_____________________________________________________________________________
/**
 * Append an array of data that occurred at a specified time.
 *
 * @param aT Time stamp of the data.
 * @param aY Array<double>.
 * @return Index of the first empty storage element.
 */
int Storage::
append(double aT,const Array<double>& aY,bool aCheckForDuplicateTime)
{
    // APPEND
    return( append ( aT, aY.getSize(), &aY[0], aCheckForDuplicateTime ));
}

//_____________________________________________________________________________
//_____________________________________________________________________________
/**
 * Store a simulation vector.
 *
 * This method differs from append in that, if the integration step of
 * a simulation is not a multiple of the step interval of this
 * Storage class, the state is not appended.
 * Note that if the step storage interval is 0, storage is turned off.
 *
 * The first empty storage location is returned.
 */
int Storage::
store(int aStep,double aT,int aN,const double *aY)
{
    if(_stepInterval==0) return(_storage.getSize());
    if((aStep%_stepInterval) == 0) {
        append(aT,aN,aY);
    }

    return(_storage.getSize());
}


//=============================================================================
// OPERATIONS
//=============================================================================
//-----------------------------------------------------------------------------
// TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Shift the times of all state vectors.
 *
 * @param aValue Value by which to shift the times.
 */
void Storage::
shiftTime(double aValue)
{
    for(int i=0;i<_storage.getSize();i++) {
        _storage[i].shiftTime(aValue);
    }
}
//_____________________________________________________________________________
/**
 * Scale the times of all state vectors.
 *
 * @param aValue Value by which to scale the times.
 */
void Storage::
scaleTime(double aValue)
{
    for(int i=0;i<_storage.getSize();i++) {
        _storage[i].scaleTime(aValue);
    }
}

//-----------------------------------------------------------------------------
// ADD
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Add a value to all state vectors in this storage instance.
 *
 * @param aValue Value to add to the state vectors.
 * @see StateVector::add(double)
 */
void Storage::
add(double aValue)
{
    for(int i=0;i<_storage.getSize();i++) {
        _storage[i].add(aValue);
    }
}
//_____________________________________________________________________________
/**
 * Add a value to all entries in one column of the storage.
 *
 * @param aN Index of the column that the value is to be added to.
 * @param aValue Value to add to the column.
 * @see StateVector::add(int,double)
 */
void Storage::
add(int aN, double aValue)
{
    for(int i=0;i<_storage.getSize();i++) {
        _storage[i].add(aN,aValue);
    }
}
//_____________________________________________________________________________
/**
 * Add an array to all state vectors in this storage instance.
 *
 * Only the first aN states of each state vector are altered.
 *
 * @param values Array of values to add to the state vectors.
 * @see StateVector::add(int,double[])
 */
void Storage::add(const SimTK::Vector_<double>& values) {
    for(int i = 0; i < _storage.getSize(); ++i) {
        _storage[i].add(values);
    }
}
//_____________________________________________________________________________
/**
 * Add a state vector to the all state vectors in this storage instance.
 *
 * @param aStateVector State vector to add to the state vectors.
 * @see StateVector::add(int,double[])
 */
void Storage::
add(StateVector *aStateVector)
{
    for(int i=0;i<_storage.getSize();i++) {
        _storage[i].add(aStateVector);
    }
}
//_____________________________________________________________________________
/**
 * Add a storage instance to this storage instance.
 *
 * Linear interpolation or extrapolation is used to get the values of the
 * states that correspond in time to the states held in this storage
 * instance.
 *
 * @param aStorage Storage to add to this storage.
s */
void Storage::
add(Storage *aStorage)
{
    if(aStorage==NULL) return;

    int n,N=0,nN;
    double t,*Y=NULL;
    for(int i=0;i<_storage.getSize();i++) {

        // GET INFO ON THIS STORAGE INSTANCE
        n = getStateVector(i)->getSize();
        t = getStateVector(i)->getTime();

        // GET DATA FROM ARGUMENT
        N = aStorage->getDataAtTime(t,N,&Y);

        // SET SIZE TO SMALLER
        nN = (n<N) ? n : N;

        // ADD
        _storage[i].add(SimTK::Vector_<double>(nN, Y));
    }

    // CLEANUP
    if(Y!=NULL) delete[] Y;
}

//-----------------------------------------------------------------------------
// SUBTRACT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Subtract a value from all state vectors in this storage instance.
 *
 * @param aValue Value to subtract from the state vectors.
 * @see StateVector::subtract(double)
 */
void Storage::
subtract(double aValue)
{
    for(int i=0;i<_storage.getSize();i++) {
        _storage[i].subtract(aValue);
    }
}
//_____________________________________________________________________________
/**
 * Subtract an array from all state vectors in this storage instance.
 *
 * Only the first aN states of each state vector are altered.
 *
 * @param values Array of values to subtract from the state vectors.
 * @see StateVector::subtract(int,double[])
 */
void Storage::subtract(const SimTK::Vector_<double>& values) {
    for(int i = 0; i < _storage.getSize(); ++i) {
        _storage[i].subtract(values);
    }
}
//_____________________________________________________________________________
/**
 * Subtract a state vector from all state vectors in this storage instance.
 *
 * @param aStateVector State vector to subtract from the state vectors.
 * @see StateVector::subtract(int,double[])
 */
void Storage::
subtract(StateVector *aStateVector)
{
    for(int i=0;i<_storage.getSize();i++) {
        _storage[i].subtract(aStateVector);
    }
}
//_____________________________________________________________________________
/**
 * Subtract a storage instance to this storage instance.
 *
 * Linear interpolation or extrapolation is used to get the values of the
 * states that correspond in time to the states held in this storage
 * instance.
 *
 * @param aStorage Storage to subtract from this storage.
s */
void Storage::
subtract(Storage *aStorage)
{
    if(aStorage==NULL) return;

    int n,N=0,nN;
    double t,*Y=NULL;
    for(int i=0;i<_storage.getSize();i++) {

        // GET INFO ON THIS STORAGE INSTANCE
        n = getStateVector(i)->getSize();
        t = getStateVector(i)->getTime();

        // GET DATA FROM ARGUMENT
        N = aStorage->getDataAtTime(t,N,&Y);

        // SET SIZE TO SMALLER
        nN = (n<N) ? n : N;

        // SUBTRACT
        _storage[i].subtract(SimTK::Vector_<double>(nN, Y));
    }

    // CLEANUP
    if(Y!=NULL) delete[] Y;
}

//-----------------------------------------------------------------------------
// MULTIPLY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Multiply all state vectors in this storage instance by a value.
 *
 * @param aValue Value by which to multiply the state vectors.
 * @see StateVector::multiply(double)
 */
void Storage::
multiply(double aValue)
{
    for(int i=0;i<_storage.getSize();i++) {
        _storage[i].multiply(aValue);
    }
}
//_____________________________________________________________________________
/**
 * Multiply all state vectors in this storage instance by an array.
 *
 * Only the first aN states of each state vector are altered.
 *
 * @param values Array of values the states are to be multiplied by.
 * @see StateVector::multiply(int,double[])
 */
void Storage::multiply(const SimTK::Vector_<double>& values) {
    for(int i = 0; i < _storage.getSize(); ++i) {
        _storage[i].multiply(values);
    }
}

//_____________________________________________________________________________
/**
 * Multiply all state vectors in this storage instance by a state vector.
 *
 * @param aStateVector State vector by which to multiply the state vectors.
 * @see StateVector::multiply(StateVector)
 */
void Storage::
multiply(StateVector *aStateVector)
{
    for(int i=0;i<_storage.getSize();i++) {
        _storage[i].multiply(aStateVector);
    }
}
//_____________________________________________________________________________
/**
 * Multiply this storage instance by a storage instance.
 *
 * Linear interpolation or extrapolation is used to get the values of the
 * states that correspond in time to the states held in this storage
 * instance.
 *
 * @param aStorage Storage instance by which to multiply.
s */
void Storage::
multiply(Storage *aStorage)
{
    if(aStorage==NULL) return;

    int n,N=0,nN;
    double t,*Y=NULL;
    for(int i=0;i<_storage.getSize();i++) {

        // GET INFO ON THIS STORAGE INSTANCE
        n = getStateVector(i)->getSize();
        t = getStateVector(i)->getTime();

        // GET DATA FROM ARGUMENT
        N = aStorage->getDataAtTime(t,N,&Y);

        // SET SIZE TO SMALLER
        nN = (n<N) ? n : N;

        // MULTIPLY
        _storage[i].multiply(SimTK::Vector_<double>(nN, Y));
    }

    // CLEANUP
    if(Y!=NULL) delete[] Y;
}
//_____________________________________________________________________________
/**
 * Multiply entries at column aIndex by a value.
 *
 * @param aIndex is the index of the column to multiply
 * @param aValue Value by which to multiply the column.
 */
void Storage::
multiplyColumn(int aIndex, double aValue)
{
    double newValue;
    for(int i=0;i<_storage.getSize();i++) {
        _storage[i].getDataValue(aIndex, newValue);
        newValue *= aValue;
        _storage[i].setDataValue(aIndex, newValue);
    }
}

//-----------------------------------------------------------------------------
// DIVIDE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Divide all state vectors in this storage instance by a value.
 *
 * @param aValue Value by which to divide the state vectors.
 */
void Storage::
divide(double aValue)
{
    for(int i=0;i<_storage.getSize();i++) {
        _storage[i].divide(aValue);
    }
}
//_____________________________________________________________________________
/**
 * Divide all state vectors in this storage instance by an array.
 *
 * Only the first aN states of each state vector are altered.
 *
 * @param values Array of values the states are to be divided by.
 */
void Storage::divide(const SimTK::Vector_<double>& values) {
    for(int i = 0; i < _storage.getSize(); ++i) {
        _storage[i].divide(values);
    }
}
//_____________________________________________________________________________
/**
 * Divide all state vectors in this storage instance by a state vector.
 *
 * @param aStateVector State vector by which to divide the state vectors.
 */
void Storage::
divide(StateVector *aStateVector)
{
    for(int i=0;i<_storage.getSize();i++) {
        _storage[i].divide(aStateVector);
    }
}
//_____________________________________________________________________________
/**
 * Divide this storage instance by a storage instance.
 *
 * Linear interpolation or extrapolation is used to get the values of the
 * states that correspond in time to the states held in this storage
 * instance.
 *
 * @param aStorage Storage instance by which to divide.
s */
void Storage::
divide(Storage *aStorage)
{
    if(aStorage==NULL) return;

    int i;
    int n,N=0,nN;
    double t,*Y=NULL;
    for(i=0;i<_storage.getSize();i++) {

        // GET INFO ON THIS STORAGE INSTANCE
        n = getStateVector(i)->getSize();
        t = getStateVector(i)->getTime();

        // GET DATA FROM ARGUMENT
        N = aStorage->getDataAtTime(t,N,&Y);

        // SET SIZE TO SMALLER
        nN = (n<N) ? n : N;

        // DIVIDE
        _storage[i].divide(SimTK::Vector_<double>(nN, Y));
    }

    // CLEANUP
    if(Y!=NULL) delete[] Y;
}

//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the average value of the first aN states stored for all state
 * vectors stored in this storage instance.
 *
 * This method uses computeArea() to compute the area (integral) and then
 * simply divides by the time interval (tf-ti).
 *
 * It is assumed that there is enough memory at aAve to hold aN states.
 * If aN exceeds the number of states held in storage, aN is disregarded.
 *
 * The number of valid states in aAve is returned.
 */
int Storage::
computeAverage(int aN,double *aAve) const
{
    int n = computeArea(aN,aAve);
    if(n==0) return(0);

    // DIVIDE BY THE TIME RANGE
    double ti = getFirstTime();
    double tf = getLastTime();
    if(tf<=ti) {
        log_error("Storage.computeAverage: time interval "
                  "invalid.\n\tfirstTime={}  lastTime={}",
                ti, tf);
        return(0);
    }
    double dt_recip = 1.0 / (tf-ti);
    for(int i=0;i<n;i++) {
        aAve[i] *= dt_recip;
    }

    return(n);
}
//_____________________________________________________________________________
/**
 * Compute the average value of the first aN states stored between the
 * times aTI and aTF.
 *
 * This method uses computeArea() to compute the area (integral) of each
 * state on the interval [aTI,aTF] and then simply divides by the (aTF-aTI).
 *
 * It is assumed that there is enough memory at aAve to hold aN states.
 * If aN exceeds the number of states held in storage, aN is disregarded.
 *
 * The number of valid states in aAve is returned.
 *
 * Note that if aTI and aTF do not fall exactly on the time stamp
 * of a stored state, the states are linearly interpolated to provide
 * an estimate of the state at aTI or at aTF.
 */
int Storage::
computeAverage(double aTI,double aTF,int aN,double *aAve) const
{
    int n = computeArea(aTI,aTF,aN,aAve);
    if(n==0) return(0);

    // DIVIDE BY THE TIME RANGE
    double dt_recip = 1.0 / (aTF-aTI);
    for(int i=0;i<n;i++) {
        aAve[i] *= dt_recip;
    }

    return(n);
}

//_____________________________________________________________________________
/**
 * Helper function for integrate/computeArea
 */
int Storage::
integrate(int aI1,int aI2,int aN,double *rArea,Storage *rStorage) const
{
    // CHECK THAT THERE ARE STATES STORED
    if(_storage.getSize()<=0) {
        log_error("Storage.integrate: no stored states.");
        return(0);
    }

    // CHECK INDICES
    if(aI1>=aI2) {
        log_error("Storage.integrate:  aI1 >= aI2.");
        return(0);
    }

    // GET THE SMALLEST NUMBER OF STATES
    int n = getSmallestNumberOfStates();
    if(n>aN) n = aN;
    if(n<=0) {
        log_error("Storage.computeArea: no stored states");
        return(0);
    }

    // SET THE INDICES
    if(aI1<0) aI1 = 0;
    if(aI2<0) aI2 = _storage.getSize()-1;

    // WORKING MEMORY
    double ti,tf;
    double *yi=NULL,*yf=NULL;

    bool functionAllocatedArea = false;
    if(!rArea) {
        rArea = new double [n];
        functionAllocatedArea = true;
    }

    // INITIALIZE AREA
    for(int i=0;i<n;i++) rArea[i]=0.0;

    // RECORD FIRST STATE
    if(rStorage) {
        ti = getStateVector(aI1)->getTime();
        rStorage->append(ti,n,rArea);
    }

    // INTEGRATE
    for(int I=aI1;I<aI2;I++) {

        // INITIAL
        ti = getStateVector(I)->getTime();
        yi = getStateVector(I)->getData().get();

        // FINAL
        tf = getStateVector(I+1)->getTime();
        yf = getStateVector(I+1)->getData().get();

        // AREA
        for(int i=0;i<n;i++) {
            rArea[i] += 0.5*(yf[i]+yi[i])*(tf-ti);
        }

        // APPEND
        if(rStorage) rStorage->append(tf,n,rArea);
    }

    // CLEANUP
    if(functionAllocatedArea) delete[] rArea;

    return(n);
}
//_____________________________________________________________________________
/**
 * Helper function for integrate/computeArea
 */
int Storage::
integrate(double aTI,double aTF,int aN,double *rArea,Storage *rStorage) const
{
    // CHECK THAT THERE ARE STATES STORED
    if(_storage.getSize()<=0) {
        log_error("Storage.integrate: no stored states.");
        return(0);
    }

    // CHECK INITIAL AND FINAL TIMES
    if(aTI>=aTF) {
        log_error("Storage.integrate: bad time range.\n\tInitial time ({}) is "
                  "not smaller than final time ({})",
                aTI, aTF);
        return(0);
    }

    // CHECK TIME RANGE
    double fstT = getFirstTime();
    double lstT = getLastTime();
    if((aTI<fstT)||(aTI>lstT)||(aTF<fstT)||(aTF>lstT)) {
        log_error("Storage.integrate: bad time range.\n\tThe specified range "
                  "({} to {}) is not covered by\ttime range of the stored "
                  "states ({} to {}).",
                aTI, aTF, fstT, lstT);
        return(0);
    }

    // CHECK FOR VALID OUTPUT ARRAYS
    if(aN<=0) return(0);

    // GET THE SMALLEST NUMBER OF STATES
    int n = getSmallestNumberOfStates();
    if(n>aN) n = aN;
    if(n<=0) {
        log_error("Storage.integrate: no stored states");
        return(0);
    }

    // WORKING MEMORY
    double ti,tf;
    double *yI = new double[n];
    double *yF = new double[n];

    bool functionAllocatedArea = false;
    if(!rArea) {
        rArea = new double [n];
        functionAllocatedArea = true;
    }

    // INITIALIZE AREA
    for(int i=0;i<n;i++) rArea[i]=0.0;

    // RECORD FIRST STATE
    if(rStorage) rStorage->append(aTI,n,rArea);

    // GET RELEVANT STATE INDICES
    int II = findIndex(aTI)+1;
    int FF = findIndex(aTF);

    // SAME INTERVAL
    if(II>FF) {
        getDataAtTime(aTI,n,&yI);
        getDataAtTime(aTF,n,&yF);
        for(int i=0;i<n;i++) {
            rArea[i] += 0.5*(yF[i]+yI[i])*(aTF-aTI);
        }
        if(rStorage) rStorage->append(aTF,n,rArea);

    // SPANS MULTIPLE INTERVALS
    } else {
        double *yi=NULL,*yf=NULL;

        // FIRST SLICE
        getDataAtTime(aTI,n,&yI);
        tf = getStateVector(II)->getTime();
        yf = getStateVector(II)->getData().get();
        for(int i=0;i<n;i++) {
            rArea[i] += 0.5*(yf[i]+yI[i])*(tf-aTI);
        }
        if(rStorage) rStorage->append(tf,n,rArea);

        // INTERVALS
        for(int I=II;I<FF;I++) {
            ti = getStateVector(I)->getTime();
            yi = getStateVector(I)->getData().get();
            tf = getStateVector(I+1)->getTime();
            yf = getStateVector(I+1)->getData().get();
            for(int i=0;i<n;i++) {
                rArea[i] += 0.5*(yf[i]+yi[i])*(tf-ti);
            }
            if(rStorage) rStorage->append(tf,n,rArea);
        }

        // LAST SLICE
        ti = getStateVector(FF)->getTime();
        yi = getStateVector(FF)->getData().get();
        getDataAtTime(aTF,n,&yF);
        for(int i=0;i<n;i++) {
            rArea[i] += 0.5*(yF[i]+yi[i])*(aTF-ti);
        }
        if(rStorage) rStorage->append(aTF,n,rArea);
    }

    // CLEANUP
    delete[] yI;
    delete[] yF;
    if(functionAllocatedArea) delete[] rArea;

    return(n);
}
//_____________________________________________________________________________
/**
 * Compute the area of the first aN states stored for all state vectors
 * stored in this storage instance.
 *
 * It is assumed that there is enough memory at aArea to hold aN states.
 * If aN exceeds the number of states held in storage, aN is disregarded.
 *
 * The number of valid states in aArea is returned.
 */
int Storage::
computeArea(int aN,double *aArea) const
{
    // CHECK FOR VALID OUTPUT ARRAYS
    if(aN<=0) return(0);
    else if(aArea==NULL) return(0);
    else return integrate(0,_storage.getSize()-1,aN,aArea,NULL);
}
//_____________________________________________________________________________
/**
 * Compute the area of the first aN states stored between the
 * times aTI and aTF.
 *
 * It is assumed that there is enough memory at aArea to hold aN states.
 * If aN exceeds the number of states held in storage, aN is disregarded.
 *
 * The number of valid states in aArea is returned.
 *
 * Note that if aTI and aTF do not fall exactly on the time stamp
 * of a stored state, the states are linearly interpolated to provide
 * an estimate of the state at aTI or at aTF.
 */
int Storage::
computeArea(double aTI,double aTF,int aN,double *aArea) const
{
    // CHECK FOR VALID OUTPUT ARRAYS
    if(aN<=0) return(0);
    else if(aArea==NULL) return(0);
    else return integrate(aTI,aTF,aN,aArea,NULL);
}
//_____________________________________________________________________________
/**
 * Integrate the state vectors between aI1 and aI2.
 *
 * The integration results are returned in an Storage instance that is a
 * copy of this instance except the name has been appended with "_integrated"
 * and, of course, the state vectors have been integrated.  The caller is
 * responsible for deleting the returned storage instance.
 *
 * If aI1 is negative, integrations starts at the first StateVector
 * held by this Storage instance.
 * If aI2 is negative, integration starts at the last StateVector held by
 * this Storage instance.
 *
 * Note that aI1 and aI2 have negative default values, so that this method
 * may be called without arguments to integrate all StateVectors held by
 * this Storage instance.
 *
 * @param aI1 Index of state vector at which to start integration.
 * @param aI2 Index of state vector at which to stop integration.
 * @return Storage instance of integrated results.  NULL is returned if an
 * error is encountered.
 */
Storage* Storage::
integrate(int aI1,int aI2) const
{
    // CREATE COPY
    Storage *integStore = new Storage(*this,false);
    integStore->setName(getName()+"_integrated");

    int n = getSmallestNumberOfStates();
    int result = integrate(aI1,aI2,n,NULL,integStore);
    if(result<=0) {
        delete integStore;
        return NULL;
    } else return integStore;
}
//_____________________________________________________________________________
/**
 * Integrate the state vectors between times aTI and aTF.
 *
 * The integration results are returned in an Storage instance that is a
 * copy of this instance except the name has been appended with "_integrated"
 * and, of course, the state vectors have been integrated.  The caller is
 * responsible for deleting the returned storage instance.
 *
 * Note that if aTI and aTF do not fall exactly on the time stamp
 * of a stored state, the states are linearly interpolated or extrapolated
 * to provide an estimate of the state at aTI or at aTF.
 *
 * @param aTI Time at which to start the integration.
 * @param aTF Time at which to stop the integration.
 * @return Storage instance of integrated results.  NULL is returned if an
 * error is encountered.
 */
Storage* Storage::
integrate(double aTI,double aTF) const
{
    // CREATE COPY
    Storage *integStore = new Storage(*this,false);
    integStore->setName(getName()+"_integrated");

    int n = getSmallestNumberOfStates();
    int result = integrate(aTI,aTF,n,NULL,integStore);
    if(result<=0) {
        delete integStore;
        return NULL;
    } else return integStore;
}

//_____________________________________________________________________________
/**
 * Pad each of the columns in a statevector by a specified amount.
 * Data is both prepended and appended by reflecting and negating.
 *
 * @param aPadSize Number of data points to prepend and append.
 */
void Storage::
pad(int aPadSize)
{
    if (aPadSize==0) return; //Nothing to do
    // PAD THE TIME COLUMN
    Array<double> paddedTime;
    int size = getTimeColumn(paddedTime);
    Signal::Pad(aPadSize,paddedTime);
    int newSize = paddedTime.getSize();

    // PAD EACH COLUMN
    int nc = getSmallestNumberOfStates();
    Array<double> paddedSignal(0.0,size);
    StateVector *vecs = new StateVector[newSize];
    for(int j=0;j<newSize;j++) {
        vecs[j].getData().setSize(nc);
        vecs[j].setTime(paddedTime[j]);
    }
    for(int i=0;i<nc;i++) {
        getDataColumn(i,paddedSignal);
        Signal::Pad(aPadSize,paddedSignal);
        for(int j=0;j<newSize;j++)
            vecs[j].setDataValue(i,paddedSignal[j]);
    }

    // APPEND THE STATEVECTORS
    _storage.setSize(0);
    for(int i=0;i<newSize;i++) _storage.append(vecs[i]);

    // CLEANUP
    delete[] vecs;
}

void Storage::
smoothSpline(int aOrder,double aCutoffFrequency)
{
    int size = getSize();
    double dtmin = getMinTimeStep();
    double avgDt = (_storage[size-1].getTime() - _storage[0].getTime()) / (size-1);

    if(dtmin<SimTK::Eps) {
        log_error("Storage.SmoothSpline: storage cannot be resampled.");
        return;
    }

    // RESAMPLE if the sampling interval is not uniform
    if ((avgDt - dtmin) > SimTK::Eps) {
        dtmin = resample(dtmin, aOrder);
        size = getSize();
    }

    if(size<(2*aOrder)) {
        log_error("Storage.SmoothSpline: too few data points to filter.");
        return;
    }

    // LOOP OVER COLUMNS
    double *times=NULL;
    int nc = getSmallestNumberOfStates();
    double *signal=NULL;
    Array<double> filt(0.0,size);
    getTimeColumn(times,0);
    for(int i=0;i<nc;i++) {
        getDataColumn(i,signal);
        Signal::SmoothSpline(aOrder,dtmin,aCutoffFrequency,size,times,signal,&filt[0]);
        setDataColumn(i,filt);
    }

    // CLEANUP
    delete[] times;
    delete[] signal;
}

void Storage::
lowpassIIR(double aCutoffFrequency)
{
    int size = getSize();
    double dtmin = getMinTimeStep();
    double avgDt = (_storage[size-1].getTime() - _storage[0].getTime()) / (size-1);

    if(dtmin<SimTK::Eps) {
        log_error("Storage.lowpassIIR: storage cannot be resampled.");
        return;
    }

    // RESAMPLE if the sampling interval is not uniform
    if ((avgDt - dtmin) > SimTK::Eps) {
        dtmin = resample(dtmin, 5);
        size = getSize();
    }

    if(size<(4)) {
        log_error("Storage.lowpassIIR: too few data points to filter.");
        return;
    }

    // LOOP OVER COLUMNS
    int nc = getSmallestNumberOfStates();
    double *signal=NULL;
    Array<double> filt(0.0,size);
    for(int i=0;i<nc;i++) {
        getDataColumn(i,signal);
        Signal::LowpassIIR(dtmin,aCutoffFrequency,size,signal,&filt[0]);
        setDataColumn(i,filt);
    }

    // CLEANUP
    delete[] signal;
}

void Storage::
lowpassFIR(int aOrder,double aCutoffFrequency)
{
    int size = getSize();
    double dtmin = getMinTimeStep();
    double avgDt = (_storage[size-1].getTime() - _storage[0].getTime()) / (size-1);

    if (dtmin<SimTK::Eps) {
        log_error("Storage.lowpassFIR: storage cannot be resampled.");
        return;
    }

    // RESAMPLE if the sampling interval is not uniform
    if ((avgDt - dtmin) > SimTK::Eps) {
        dtmin = resample(dtmin, 5);
        size = getSize();
    }

    if(size<(2*aOrder)) {
        log_error("Storage.lowpassFIR: too few data points to filter.");
        return;
    }

    // LOOP OVER COLUMNS
    int nc = getSmallestNumberOfStates();
    double *signal=NULL;
    Array<double> filt(0.0,size);
    for(int i=0;i<nc;i++) {
        getDataColumn(i,signal);
        Signal::LowpassFIR(aOrder,dtmin,aCutoffFrequency,size,signal,&filt[0]);
        setDataColumn(i,filt);
    }

    // CLEANUP
    delete[] signal;
}


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Find the index of the storage element that occurred immediately before
 * or at time aT ( aT <= getTime(index) ).
 *
 * This method can be much more efficient than findIndex(aT) if a good guess
 * is made for aI.
 * If aI corresponds to a state which occurred later than aT, an exhaustive
 * search is performed by calling findIndex(aT).
 *
 * @param aI Index at which to start searching.
 * @param aT Time.
 * @return Index preceding or at time aT.  If aT is less than the earliest
 * time, 0 is returned.
 */
int Storage::
findIndex(int aI,double aT) const
{
    // MAKE SURE aI IS VALID
    if(_storage.getSize()<=0) return(-1);
    if((aI>=_storage.getSize())||(aI<0)) aI=0;
    if(getStateVector(aI)->getTime()>aT) aI=0;

    // SEARCH
    //cout << "Storage.findIndex: starting at " << aI << endl;
    int i;
    for(i=aI;i<_storage.getSize();i++) {
        if(aT<getStateVector(i)->getTime()) break;
    }
    _lastI = i-1;
    if(_lastI<0) _lastI=0;
    return(_lastI);
}
//_____________________________________________________________________________
/**
 * Find the index of the storage element that occurred immediately before
 * or at a specified time ( getTime(index) <= aT ).
 *
 * This method is not very efficient because it always starts its search
 * with the first stored state.
 *
 * @param aT Time.
 * @return Index preceding or at time aT.  If aT is less than the earliest
 * time, 0 is returned.
 */
int Storage::
findIndex(double aT) const
{
    if(_storage.getSize()<=0) return(-1);
    int i;
    for(i=0;i<_storage.getSize();i++) {
        if(aT<getStateVector(i)->getTime()) break;
    }
    _lastI = i-1;
    if(_lastI<0) _lastI=0;
    return(_lastI);
}
//_____________________________________________________________________________
/**
 * Find the range of frames that is between start time and end time
 * (inclusive). Return the indices of the bounding frames.
 */
void Storage::findFrameRange(double aStartTime, double aEndTime, int& oStartFrame, int& oEndFrame) const
{
    SimTK_ASSERT_ALWAYS(aStartTime <= aEndTime, "Start time must be <= end time");

    oStartFrame = findIndex(0, aStartTime);
    oEndFrame = findIndex(getSize()-1, aEndTime);
}
//_____________________________________________________________________________
/**
 * Resample Storage columns to specified rate. This's done by fitting splines
 * to Storage columns and resampling
 *
 * @param aDT Time interval between adjacent statevectors.
 * @return Actual sampling time step (may be clamped)
 */
double Storage::
resample(double aDT, int aDegree)
{
    int numDataRows = _storage.getSize();

    if(numDataRows<=1) return aDT;

    // Limit aDT based on expected number of samples
    int maxSamples = MAX_RESAMPLE_SIZE;
    if((getLastTime()-getFirstTime())/aDT > maxSamples) {
        double newDT = (getLastTime()-getFirstTime())/maxSamples;
        log_warn("Storage.resample: resampling at time step {} (but minimum "
                 "time step is {}).",
                newDT, aDT);
        aDT = newDT;
    }

    GCVSplineSet *splineSet = new GCVSplineSet(aDegree,this);

    Array<std::string> saveLabels = getColumnLabels();
    // Free up memory used by Storage
    _storage.setSize(0);
    // For every column, collect data and fit spline to originalTimes, dataColumn.
    Storage *newStorage = splineSet->constructStorage(0,aDT);
    newStorage->setInDegrees(isInDegrees());
    copyData(*newStorage);

    setColumnLabels(saveLabels);

    delete newStorage;
    delete splineSet;

    return aDT;
}
//_____________________________________________________________________________
/**
 * Resample using linear interpolation
 */
double Storage::
resampleLinear(double aDT)
{
    int numDataRows = _storage.getSize();

    if(numDataRows<=1) return aDT;

    // Limit aDT based on expected number of samples
    int maxSamples = MAX_RESAMPLE_SIZE;
    if((getLastTime()-getFirstTime())/aDT > maxSamples) {
        double newDT = (getLastTime()-getFirstTime())/maxSamples;
        log_warn("Storage.resample: resampling at time step {} (but minimum "
                 "time step is {}).",
                newDT, aDT);
        aDT = newDT;
    }

    Array<std::string> saveLabels = getColumnLabels();

    // HOW MANY TIME STEPS?
    double ti = getFirstTime();
    double tf = getLastTime();
    int nr = IO::ComputeNumberOfSteps(ti,tf,aDT);

    Storage *newStorage = new Storage(nr);

    // LOOP THROUGH THE DATA
    int ny=0;
    double *y=NULL;
    StateVector vec;
    for(int i=0; i<nr; i++) {
        double t = ti+aDT*(double)i;
        // INTERPOLATE THE STATES
        ny = getDataAtTime(t,ny,&y);
        newStorage->append(t,ny,y);
    }

    copyData(*newStorage);

    delete newStorage;
    delete[] y;

    return aDT;
}

//_____________________________________________________________________________
/**
 * interpolateAt passed in list of time values. Tries to check if there is a data
 * row at the specified times to avoid introducing duplicates.
 */
void Storage::interpolateAt(const Array<double> &targetTimes)
{
    for(int i=0; i<targetTimes.getSize();i++){
        double t = targetTimes[i];
        // get index for t
        int tIndex = findIndex(t);
        // If within small number from t then pass
        double actualTime=0.0;
        if (tIndex < getSize()-1){
            getTime(tIndex+1, actualTime);
            if (fabs(actualTime - t)<1e-6)
                    continue;
        }
        // or could be the following one too
        getTime(tIndex, actualTime);
        if (fabs(actualTime - t)<1e-6)
                continue;
        // create a StateVector and add it
        double *y=NULL;
        int ny=0;
        StateVector vec;
        // INTERPOLATE THE STATES
        ny = getDataAtTime(t,ny,&y);
        vec.setStates(t, SimTK::Vector_<double>(ny, y));

        _storage.insert(tIndex+1, vec);
    }
}
//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Set name of output file to be written into.
 * This has the side effect of opening the file for writing. The header will not have the correct
 * number of rows but this may not be an issue for version 2 of the Storage class
 */
void Storage::
setOutputFileName(const std::string& aFileName)
{
    assert(_fileName=="");
    _fileName = aFileName;

    // OPEN THE FILE
    _fp = IO::OpenFile(aFileName,"w");
    if(_fp==NULL) throw(Exception("Could not open file "+aFileName));
    // WRITE THE HEADER
    writeHeader(_fp);
    writeDescription(_fp);
    // WRITE THE COLUMN LABELS
    writeColumnLabels(_fp);
}
//_____________________________________________________________________________
/**
 * Print the contents of this storage instance to a file.
 *
 * The argument aMode specifies whether the file is opened for writing, "w",
 * or appending, "a".  If a bad value for aMode is sent in, the file is opened
 * for writing.
 *
 * The total number of characters written is returned.  If an error occurred,
 * a negative number is returned.
 *
 * @param aFileName Name of file to which to save.
 * @param aMode Writing mode: "w" means write and "a" means append.  The
 * default is "w".
 * @param aComment string to be written to the file header (preceded by # per SIMM)
 * @return true on success
 */
bool Storage::
print(const string &aFileName,const string &aMode, const string& aComment) const
{
    // OPEN THE FILE
    FILE *fp = IO::OpenFile(aFileName,aMode);
    if(fp==NULL) return(false);

    // WRITE THE HEADER
    int n=0,nTotal=0;
    n = writeHeader(fp);
    if(n<0) {
        log_error("Storage.print: failed to write header to file {}.",
                aFileName);
        return(false);
    }

    // WRITE SIMM HEADER
    if(_writeSIMMHeader) {
        n = writeSIMMHeader(fp, -1, aComment.c_str());
        if(n<0) {
            log_error("Storage.print: failed to write SIMM header to file {}.",
                    aFileName);
            return(false);
        }
    }

    // WRITE THE DESCRIPTION
    n = writeDescription(fp);
    if(n<0) {
        log_error("Storage.print: failed to write description to file {}.",
                aFileName);
        return(false);
    }

    // WRITE THE COLUMN LABELS
    n = writeColumnLabels(fp);
    if(n<0) {
        log_error("Storage.print: failed to write column labels to file {}.",
                aFileName);
        return(false);
    }

//printf("Storage.cpp:print storage=%x  n=%d ",&_storage, _storage.getSize());
//std::cout << aFileName << endl;

    // VECTORS
    for(int i=0;i<_storage.getSize();i++) {
        n = getStateVector(i)->print(fp);
        if(n<0) {
            log_error("Storage.print: error printing to {}.", aFileName);
            return(false);
        }
        nTotal += n;
    }

    // CLOSE
    fclose(fp);

    return(nTotal!=0);
}
//_____________________________________________________________________________
/**
 * Print the contents of this storage instance to a file named by the argument
 * aFileaName using uniform time spacing.
 *
 * The argument aMode specifies whether the file is opened for writing, "w",
 * or appending, "a".  If a bad value for aMode is sent in, the file is opened
 * for writing.
 *
 * The argument aDT specifies the time spacing.
 *
 * The total number of characters written is returned.  If an error occurred,
 * a negative number is returned.
 */
int Storage::
print(const string &aFileName,double aDT,const string &aMode) const
{
    // CHECK FOR VALID DT
    if(aDT<=0) return(0);

    if (_fp!= NULL) fclose(_fp);
    // OPEN THE FILE
    FILE *fp = IO::OpenFile(aFileName,aMode);
    if(fp==NULL) return(-1);

    // HOW MANY TIME STEPS?
    double ti = getFirstTime();
    double tf = getLastTime();
    int nr = IO::ComputeNumberOfSteps(ti,tf,aDT);
//printf("Storage.cpp:print ti=%f tf=%f dt=%f nr=%d ", ti,tf,aDT,nr );
//std::cout << aFileName << endl;

    // WRITE THE HEADER
    int n,nTotal=0;
    n = writeHeader(fp,aDT);
    if(n<0) {
        log_error("Storage.print: failed to write header to file {}.",
                aFileName);
        return(n);
    }

    // WRITE SIMM HEADER
    if(_writeSIMMHeader) {
        n = writeSIMMHeader(fp,aDT);
        if(n<0) {
            log_error("Storage.print: failed to write SIMM header to file {}.",
                    aFileName);
            return(n);
        }
    }

    // WRITE THE DESCRIPTION
    n = writeDescription(fp);
    if(n<0) {
        log_error("Storage.print: failed to write description to file {}.",
                aFileName);
        return(n);
    }

    // WRITE THE COLUMN LABELS
    n = writeColumnLabels(fp);
    if(n<0) {
        log_error("Storage.print: failed to write column labels to file {}.",
                aFileName);
        return(n);
    }

    // LOOP THROUGH THE DATA
    int i,ny=0;
    double t,*y=NULL;
    StateVector vec;
    for(t=ti,i=0;i<nr;i++,t=ti+aDT*(double)i) {

        // INTERPOLATE THE STATES
        ny = getDataAtTime(t,ny,&y);
        vec.setStates(t, SimTK::Vector_<double>(ny, y));

        // PRINT
        n = vec.print(fp);
        if(n<0) {
            log_error("Storage.print: error printing to {}.", aFileName);
            return(n);
        }
        nTotal += n;
    }

    // CLEANUP
    fclose(fp);
    if(y!=NULL) { delete[] y;  y=NULL; }

    return(nTotal);
}

void Storage::printResult(const Storage* aStorage, const std::string& aName,
        const std::string& aDir, double aDT, const std::string& aExtension) {
    if (!aStorage) return;
    std::string path = (aDir == "") ? "." : aDir;
    std::string directory;
    bool dontApplySearchPath;
    std::string fileName, extension;
    SimTK::Pathname::deconstructPathname(
            aName, dontApplySearchPath, directory, fileName, extension);
    if (directory != "") {
        log_warn("Directory '{}' was specified where only file name '{}' is "
                 "expected. The directory will be ignored. Result files will "
                 "be written to directory '{}' instead.",
                    directory, fileName, path);
    }
    std::string name = (extension == "") ? (path + "/" + fileName + aExtension)
                                         : (path + "/" + fileName + extension);
    if(aDT<=0.0) aStorage->print(name);
    else aStorage->print(name,aDT);
}

//_____________________________________________________________________________
/**
 * Write the header.
 */
int Storage::
writeHeader(FILE *rFP,double aDT) const
{
    if(rFP==NULL) return(-1);

    // COMPUTE ATTRIBUTES
    int nr,nc;
    if(aDT<=0) {
        nr = _storage.getSize();
    } else {
        double ti = getFirstTime();
        double tf = getLastTime();
        nr = IO::ComputeNumberOfSteps(ti,tf,aDT);
    }
    nc = getSmallestNumberOfStates()+1;

    // ATTRIBUTES
    fprintf(rFP,"%s\n",getName().c_str());
    fprintf(rFP,"version=%d\n",LatestVersion);
    fprintf(rFP,"nRows=%d\n",nr);
    fprintf(rFP,"nColumns=%d\n",nc);
    fprintf(rFP,"inDegrees=%s\n",(_inDegrees?"yes":"no"));

    return(0);
}
//_____________________________________________________________________________
/**
 * Write a header appropriate for SIMM motion files.
 *
 * @return SIMM header.
 */
int Storage::
writeSIMMHeader(FILE *rFP,double aDT, const char *aComment) const
{
    if(rFP==NULL) return(-1);

    // COMMENT
    // NOTE: avoid writing empty comment because SIMM seems to screw up parsing
    // of a line with only a #
    if (aComment && aComment[0])
        fprintf(rFP,"\n# %s\n", aComment);
    else
        fprintf(rFP,"\n# SIMM Motion File Header:\n");

    // NAME
    fprintf(rFP,"name %s\n",getName().c_str());

    // COLUMNS
    fprintf(rFP,"datacolumns %d\n",getSmallestNumberOfStates()+1);

    // ROWS
    int nRows;
    if(aDT<=0) {
        nRows = _storage.getSize();
    } else {
        nRows = IO::ComputeNumberOfSteps(getFirstTime(),getLastTime(),aDT);
    }
    fprintf(rFP,"datarows %d\n",nRows);

    // OTHER DATA
    fprintf(rFP,"otherdata 1\n");

    // RANGE
    fprintf(rFP,"range %lf %lf\n",getFirstTime(),getLastTime());

    // Other data from the map
    MapKeysToValues::const_iterator iter;

    for(iter = _keyValueMap.begin(); iter != _keyValueMap.end(); iter++){
        fprintf(rFP,"%s %s\n",iter->first.c_str(), iter->second.c_str());
    }
    return(0);
}
//_____________________________________________________________________________
/**
 * Write the description.
 */
int Storage::
writeDescription(FILE *rFP) const
{
    if(rFP==NULL) return(-1);

    // DESCRIPTION
    string descrip = getDescription();
    size_t len = descrip.size();
    if((len>0)&&(descrip[len-1]!='\n')) {
        fprintf(rFP,"%s\n",descrip.c_str());
    } else {
        fprintf(rFP,"%s",descrip.c_str());
    }

    // DESCRIPTION TOKEN
    fprintf(rFP,"%s\n",_headerToken.c_str());

    return(0);
}
//_____________________________________________________________________________
/**
 * Write the column labels.
 *
 * @param rFP File pointer.
 */
int Storage::
writeColumnLabels(FILE *rFP) const
{
    if(rFP==NULL) return(-1);

    if(_columnLabels.getSize()) {
        fprintf(rFP,"%s",_columnLabels[0].c_str());
        for(int i=1;i<_columnLabels.getSize();i++) fprintf(rFP,"\t%s",_columnLabels[i].c_str());
        fprintf(rFP,"\n");
    } else {
        // Write default column labels
        fprintf(rFP,"time");
        int n=getSmallestNumberOfStates();
        for(int i=0;i<n;i++) fprintf(rFP,"\tstate_%d",i);
        fprintf(rFP,"\n");
    }

    return(0);
}
void Storage::addToRdStorage(Storage& rStorage, double aStartTime, double aEndTime)
{
    bool addedData = false;
    double time, stateTime;

    /* Loop through the rows in rStorage from aStartTime to aEndTime,
     * looking for a match (by time) in the rows of Storage.
     * If you find a match, add the columns in the Storage
     * to the end of the state vector in the rStorage. If you
     * don't find one, it's a fatal error so throw an exception.
     * Don't add a column if its name is 'unassigned'.
     */
    int i, j, startIndex, endIndex;
    rStorage.findFrameRange(aStartTime, aEndTime, startIndex, endIndex);
    int numColumns=getColumnLabels().getSize();
    for (i = startIndex; i <= endIndex; i++)
    {
        rStorage.getTime(i, stateTime);
        for (j = 0; j < getSize(); j++) {
            /* Assume that the first column is 'time'. */
            time = getStateVector(j)->getTime();
            // The following tolerance is a hack. Previously, it used 0.0001
            // which caused values to be duplicated in cases where time
            // steps were within the tolerance. This method should only be
            // used to concatenate data columns from the same simulation
            // or analysis results.
            if (EQUAL_WITHIN_TOLERANCE(time, stateTime, SimTK::SignificantReal)) {
                Array<double>& states = rStorage.getStateVector(i)->getData();
                // Start at 1 to avoid duplicate time column
                for (int k = 1; k < numColumns; k++)
                {
                    if (_columnLabels[k] != "Unassigned")
                    {
                        states.append(getStateVector(j)->getData().get(k-1));
                        addedData = true;
                    }
                }
                break;
            }
        }
        if (j == getSize()) {
            stringstream errorMessage;
            errorMessage << "Error: no data found at time " << stateTime
                << " in " << _fileName;
            throw (Exception(errorMessage.str()));
        }
    }

    /* Add the coordinate names to the Storage (if at least
     * one row of data was added to the object).
     */
    if (addedData)
    {
        const Array<std::string>& oldColumnLabels =rStorage.getColumnLabels();
        Array<std::string> newColumnLabels(oldColumnLabels);
        for (int i = 1; i < _columnLabels.getSize(); i++) // Start at 1 to avoid duplicate time label
        {
            if (!(_columnLabels[i] == "Unassigned"))
                newColumnLabels.append( _columnLabels[i]);
        }
        rStorage.setColumnLabels(newColumnLabels);
    }
}
//_____________________________________________________________________________
/**
 * Processing of special reserved words used by SIMM and corresponding values
 * The keys and their corresponding values are maintained in _keyValueMap
 */
// Add a new Pair
void Storage::
addKeyValuePair(const std::string& aKey, const std::string& aValue)
{
    if (_keyValueMap.find(aKey)!=_keyValueMap.end()){
        // Should we warn here? or append in case of comment?
    }
    _keyValueMap[aKey]=aValue;
    //cout << "key, value " << aKey << ", " << aValue << endl;

}
// Lookup the value for a key
void Storage::
getValueForKey(const std::string& aKey, std::string& rValue) const
{
    MapKeysToValues::const_iterator iter =_keyValueMap.find(aKey);
    if (iter!=_keyValueMap.end()){
        rValue=iter->second;
    }
    else
        rValue="";

}
// Check if the key exists
bool Storage::hasKey(const std::string& aKey) const
{
    return (_keyValueMap.find(aKey)!=_keyValueMap.end());
}

//_____________________________________________________________________________
/**
 * Check that a Token belongs to a list of reserved keys
 *
 * @returns true on success (meaningful values of rNumRows, rNumColumns)
 */
bool Storage::isSimmReservedToken(const std::string& aToken)
{
    for(int i=0; i<numSimmReservedKeys; i++){
        if (simmReservedKeys[i]==aToken)
            return true;
    }
    return false;
}
//_____________________________________________________________________________
/**
 * parse headers of OpenSim::Storage file or SIMM motion file into a Storage object
 * and populate rNumRows, rNumColumns
 * a Map between some keywords and their values for SIMM compatibility
 *
 * @returns true on success (meaningful values of rNumRows, rNumColumns)
 */
bool Storage::parseHeaders(std::ifstream& aStream, int& rNumRows, int& rNumColumns)
{
    bool done=false;
    bool firstLine=true;
    _fileVersion = 0;
    // Parse until the end of header
    while(!done){
        // NAME
        string line = IO::ReadLine(aStream);
        // Always Strip leading and trailing spaces and tabs
        IO::TrimLeadingWhitespace(line);
        IO::TrimTrailingWhitespace(line);
        if(line.empty() && !aStream.good()) {
            log_error("Storage: no more lines in storage file.");
            return false;
        }
        if (line.length()==0)
            continue;

        // Here we have a line. Should be one of:
        // name
        // nRows, nr, datarows
        // nColumns, nc, datacolumns
        // nd, nDescrip
        // # Comment
        size_t delim = line.find_first_of(" \t=");
        string key = line.substr(0, delim);
        size_t restidx = line.find_first_not_of(" \t=", delim);
        string rest = (restidx==string::npos) ? "" : line.substr(restidx);

        if (key== "name"){
            setName(rest);
        }
        else if (key== "nr" || key== "nRows" || key== "datarows"){
            rNumRows = atoi(rest.c_str());
        }
        else if (key== "nc" || key== "nColumns" || key== "datacolumns"){
            rNumColumns = atoi(rest.c_str());
        }
        else if (isSimmReservedToken(key)) {
                _keyValueMap[key]= rest;
        }
        else if (key== "units") {
                _units = Units(rest);
        }
        else if (key=="version") {
            _fileVersion = atoi(rest.c_str());
        }
        else if (key=="inDegrees") {
            string lower = IO::Lowercase(rest);
            bool inDegrees = (lower=="yes" || lower=="y");
            setInDegrees(inDegrees);
        }
        else if (key=="Angles" && _fileVersion==0){
            if (line == "Angles are in degrees.")
                setInDegrees(true);
            else if (line == "Angles are in radians.")
                setInDegrees(false);
        }
        else if(key== DEFAULT_HEADER_TOKEN){
            break;
        }
        else if (firstLine){    // Storage file have their names without "name prefix on first line"
            setName(line);
        }
        firstLine=false;
    }

    if (_fileVersion < 1) {
        log_info("Old version storage/motion file encountered");
    }

    if(rNumColumns==0 || rNumRows==0) {
        return false;
    }

    return true;
}
//_____________________________________________________________________________
/**
 * This function exchanges the time column (including the label) with the column    
 * at the passed in aColumnIndex. The index is zero based relative to the Data
 */
void Storage::
exchangeTimeColumnWith(int aColumnIndex)
{
    StateVector* vec;
    for(int i=0; i< _storage.getSize(); i++){
        vec = getStateVector(i);
        double swap = vec->getData().get(aColumnIndex);
        double time=vec->getTime();
        vec->setDataValue(aColumnIndex, time);
        vec->setTime(swap);
    }
    // Now column labels
    string swap = _columnLabels.get(0);
    _columnLabels.set(aColumnIndex+1, swap);
    _columnLabels.set(0, "time");

}
//_____________________________________________________________________________
/**
 * If that was a SIMM motion file post-process it to account for
 * lack of time column, assumption of uniform time
 * other kinds of processing can be added here to account for calc_derivatives, ...
 *
 * This is all untested since all motion files we deal with so far do not exhibit this behavior.
 */
void Storage::postProcessSIMMMotion() 
{
    Array<std::string> currentLabels = getColumnLabels();
    // If time is not first column check if it exists somewhere else and exchange
    if (!(currentLabels.get(0)=="time")){
        int timeColumnIndx = currentLabels.findIndex("time");
        if (timeColumnIndx!=-1){ // Exchange column timeColumnIndx with time
            exchangeTimeColumnWith(timeColumnIndx);
        }
        else {  
            // There was no time column altogether. make one based on
            // range (if specified) and number of entries
            MapKeysToValues::iterator iter;
            iter = _keyValueMap.find("range");  // Should we check "Range", "RANGE" too?
            if (iter !=_keyValueMap.end()){
                string rangeValue = iter->second;
                double start, end;
                sscanf(rangeValue.c_str(), "%lf %lf", &start, &end);
                if (_storage.getSize()<2){  // Something wrong throw exception unless start==end
                    if (start !=end){
                        stringstream errorMessage;
                        errorMessage << "Error: Motion file has inconsistent headers";
                        throw (Exception(errorMessage.str()));
                    }
                    else if (_storage.getSize()==1){
                        // Prepend a Time column
                        StateVector vec = _storage.get(0);
                        vec.getData().append(0.0);
                        _columnLabels.append("time");
                        exchangeTimeColumnWith(_columnLabels.findIndex("time"));
                    }
                    else
                        throw (Exception("File has no data"));
                }
                else {  // time  column from range, size
                    double timeStep = (end - start)/(_storage.getSize()-1);
                    _columnLabels.append("time");
                    for(int i=0; i<_storage.getSize(); i++){
                        Array<double>& data=_storage.updElt(i).getData();
                        data.append(i*timeStep);
                    }
                    int timeColumnIndex=_columnLabels.findIndex("time");
                    exchangeTimeColumnWith(timeColumnIndex-1);
                }
            }
            else {  // No time specified altogether
                    throw (Exception("Storage::postProcessSIMMMotion no time column found."));
            }
        }
    }
}
/**
 * Compare column named "aColumnName" in two storage objects
 * If endTime is not specified the comparison goes to the end of the file
 * @returns the difference or SimTK::Infinity if times do not match up.
 *
 * NOTE: This assumes same time sampling between both Storages.
 */
double Storage::compareColumn(Storage& aOtherStorage, const std::string& aColumnName, double startTime, double endTime)
{
    //Subtract one since, the data does not include the time column anymore.
    int thisColumnIndex=_columnLabels.findIndex(aColumnName)-1;
    int otherColumnIndex = aOtherStorage._columnLabels.findIndex(aColumnName)-1;

    double theDiff = SimTK::NaN;

    if ((thisColumnIndex==-2)||(otherColumnIndex==-2))// not found is now -2 since we subtracted 1 already
        return theDiff;

    // Now we have two columnNumbers. get the data and compare
    Array<double> thisData, otherData;
    Array<double> thisTime, otherTime;
    getDataColumn(thisColumnIndex, thisData);
    getTimeColumn(thisTime);

    aOtherStorage.getDataColumn(otherColumnIndex, otherData);
    aOtherStorage.getTimeColumn(otherTime);

    // make sure times match (we probably should make this more flexible to interpolate missing values
    // but that's not needed for now.
    theDiff = -SimTK::Infinity;
    int startIndex = findIndex(startTime);
    int startIndexOther = aOtherStorage.findIndex(startTime);
    int endIndex = (endTime==-1.0)?getSize():findIndex(endTime);
    int endIndexOther = (endTime==-1.0)?(aOtherStorage.getSize()):aOtherStorage.findIndex(endTime);
    // Make sure we have same number of rows
    if ((endIndex -startIndex)!= (endIndexOther -startIndexOther)) return (theDiff);

    for(int i=startIndex; i< endIndex; i++){
        if (abs(thisTime[i]-otherTime[startIndexOther+i-startIndex]) > 1E-3) 
            return SimTK::Infinity;
        theDiff = std::max(theDiff, fabs(thisData[i]-otherData[startIndexOther+i-startIndex]));
    }
    return theDiff;
}
/**
 * Compare column named "aColumnName" in two storage objects
 * If endTime is not specified the comparison goes to the end of the file
 * @returns the root mean square, using a spline to calculate values where the times do not match up.
 */
double Storage::compareColumnRMS(const Storage& aOtherStorage, const std::string& aColumnName, double startTime, double endTime) const
{
    int thisColumnIndex = getStateIndex(aColumnName);
    int otherColumnIndex = aOtherStorage.getStateIndex(aColumnName);

    if ((thisColumnIndex < 0) || (otherColumnIndex < 0))
        return SimTK::NaN;

    // Now we have two columnNumbers. get the data and compare
    Array<double> thisData, otherData;
    Array<double> thisTime, otherTime;
    getDataColumn(thisColumnIndex, thisData);
    getTimeColumn(thisTime);
    aOtherStorage.getDataColumn(otherColumnIndex, otherData);
    aOtherStorage.getTimeColumn(otherTime);

    // get start and end indices
    if (SimTK::isNaN(startTime))
        startTime = max(thisTime[0], otherTime[0]);
    int startIndex = findIndex(startTime);
    if (thisTime[startIndex] < startTime)
        ++startIndex;
    if (SimTK::isNaN(endTime))
        endTime = min(thisTime.getLast(), otherTime.getLast());
    int endIndex = findIndex(endTime);
    
    // create spline in case time values do not match up
    GCVSpline spline(3, otherTime.getSize(), &otherTime[0], &otherData[0]);

    double rms = 0.;

    for(int i = startIndex; i <= endIndex; i++) {
        SimTK::Vector inputTime(1, thisTime[i]);
        double diff = thisData[i] - spline.calcValue(inputTime);
        rms += diff * diff;
    }

    rms = sqrt(rms/(endIndex - startIndex));

    return rms;
}
/**
 * Compare this storage object with a standard storage object. Find RMS
 * errors for columns occurring in both storage objects, and record the
 * values and column names in the comparisons and columnsUsed Arrays.
 */
void Storage::compareWithStandard(const Storage& standard, std::vector<string>& columnsUsed, std::vector<double>& comparisons) const
{
    int maxColumns = _columnLabels.getSize();

    for (int i = 1; i < maxColumns; ++i) {
        double comparison = compareColumnRMS(standard, _columnLabels[i]);
        if (!SimTK::isNaN(comparison)) {
            comparisons.push_back(comparison);
            columnsUsed.push_back(_columnLabels[i]);
        }
    }
}

bool Storage::makeStorageLabelsUnique() {
    Array<std::string> lbls = getColumnLabels();
    std::string offending = "";
    bool changedLabels = false;
    for(int i = 0; i < lbls.getSize(); i++){
        bool isUnique = (lbls.findIndex(lbls[i]) == i);
        if (!isUnique) { // Make new names
            offending = lbls[i];
            bool exist = true;
            std::string newName = offending;
            changedLabels = true;
            int c = 1;
            while (exist) {
                char cString[20];
                sprintf(cString,"%d", c);
                newName = std::string(cString) + "_" + offending;
                exist = (lbls.findIndex(newName) != -1);
                c++;
            }
            lbls[i] = newName;
        }
    }
    if (changedLabels) setColumnLabels(lbls);
    const bool labelsWereUnique = (!changedLabels);
    return labelsWereUnique;
}

bool Storage::storageLabelsAreUnique() const {
    const auto& lbls = getColumnLabels();
    for(int i = 0; i < lbls.getSize(); i++) {
        const bool isUnique = (lbls.findIndex(lbls[i]) == i);
        if (!isUnique) return false;
    }
    return true;
}
