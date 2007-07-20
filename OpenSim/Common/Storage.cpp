// Storage.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "osimCommonDLL.h"
#include <sstream>
#include <iostream>
#include "rdMath.h"
#include "IO.h"
#include "Signal.h"
#include "Storage.h"
#include "GCVSplineSet.h"
#include "SimmIO.h"
#include "SimmMacros.h"


using namespace OpenSim;
using namespace std;


//============================================================================
// DEFINES
//============================================================================


//============================================================================
// CONSTANTS
//============================================================================
const double Storage::LARGE_NEGATIVE = -1.0e-30;
const double Storage::LARGE_POSITIVE =  1.0e-30;
const char* Storage::DEFAULT_HEADER_TOKEN = "endheader";
const char* Storage::DEFAULT_HEADER_SEPARATOR = " \t\r\n";
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
int numSimmReservedKeys=10;	// Keep this number in sync with above array size

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
	_storage(StateVector())
{
	// SET NULL STATES
	setNull();

	// CAPACITY
	_storage.ensureCapacity(aCapacity);
	_storage.setCapacityIncrement(-1);

	// SET THE STATES
	setName(aName);
}
//_____________________________________________________________________________
/**
 * Construct an Storage instance from file.
 * This constructor is far from bullet proof.
 *
 * @param aFileName Name of the file from which the Storage is to be
 * constructed.
 */
Storage::Storage(const string &aFileName) :
	_storage(StateVector())
{
	// SET NULL STATES
	setNull();

	// OPEN FILE
	ifstream *fp = IO::OpenInputFile(aFileName);
	if(fp==NULL) throw Exception("Storage: ERROR- failed to open file " + aFileName, __FILE__,__LINE__);

	int nr=0,nc=0,nd=0;
	if (!parseHeaders(*fp, nr, nc)) throw Exception("Storage: ERROR- failed to parse headers of file " + aFileName, __FILE__,__LINE__);
	cout << "Storage: file=" << aFileName << " (nr=" << nr << " nc=" << nc << ")" << endl;


	// IGNORE blank lines after header -- treat \r and \n as end of line chars
	while(fp->good()) {
		int c = fp->peek();
		if(c!='\n' && c!='\r') break;
		fp->get();
	}
	// Ayman: Support for oldStyleDescription dropped. 04/11/07

	// COLUMN LABELS
	string line;
	getline(*fp, line);
	parseColumnLabels(line.c_str());

	// CAPACITY
	_storage.ensureCapacity(nr);
	_storage.setCapacityIncrement(-1);

	// DATA
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

	// CLOSE FILE
	delete fp;
	// If what we read was really a sIMM motion file, adjust the data 
	// to account for different assumptions between SIMM.mot OpenSim.sto
	postProcessSIMMMotion();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 */
Storage::Storage(const Storage &aStorage,bool aCopyData) :
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
//_____________________________________________________________________________
/**
 * Copy this object.
 *
 * @return Pointer to a copy of this object.
 */
Object* Storage::
copy() const
{
	Storage *store = new Storage(*this);
	return(store);
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
}
//_____________________________________________________________________________
/**
 * Copy the data stored by another storage instance.
 *
 * Note that this method only copies the stored data.  It does not copy
 * other members of aStorage such as the name and the description.  To get
 * a complete copy, the copy constructor should be used.
 *
 * If this instance does not have enough capicity to hold the states
 * of the specified storage (aStorage), the capacity is increased.
 */
void Storage::
copyData(const Storage &aStorage)
{
	_units = aStorage._units;
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
/**
 * Get the column index corresponding to specified column name.
 *
 * @return State index of column or -1.  Note that the returned index is equivalent
 * to the state index.  For example, for the first column in a storage (usually
 * time) -1 would be returned.  For the second colunn in a storage (the first
 * state) 0 would be returned.
 * @todo Rename this method getStateIndex()
 * 
 * added a default Parameter for startIndex. -Ayman
 */
const int Storage::
getStateIndex(const std::string &aColumnName, int startIndex) const
{
	int size = _columnLabels.getSize();
	for(int i=startIndex;i<size;i++)
		if(_columnLabels[i]==aColumnName)
			return(i-1);

	return(-1);
}


//_____________________________________________________________________________
/**
 * Set a labels string for the columns in this Storage instance.
 * 
 * A character string is used to label the columns.  Each separate column
 * label is usually delimited by a tab ("\t"), but any delimeter may
 * be used.
 * 
 * The first column is almost always "Time."  The other columns
 * correspond to the separate elements of a state vector (StateVector).
 *
 * If the labels string is set to NULL, the following default labels
 * will be used when the Storage instance is saved to file:
 *
 * time	state_0	state_1	state_2	...
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
	int len = strlen(aLabels);
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
		vec = &_storage.getLast();
	} catch(Exception &) {
		//x.print(cout);
	}
	return(vec);
}
//_____________________________________________________________________________
/**
 * Get the StateVector at a spcified time index.
 *
 * @param aTimeIndex Time index at which to get the state vector:
 * 0 <= aTimeIndex < _storage.getSize().
 * @return Statevector. If no valid statevector exists at aTimeIndex, NULL
 * is returned.
 */
StateVector* Storage::
getStateVector(int aTimeIndex) const
{
	return(&_storage.get(aTimeIndex));
}

//-----------------------------------------------------------------------------
// TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the time of the first stored states.
 *
 * @return Time of the first stored states.  If there is no stored state,
 * the constant rdMath::NAN (not a number) is returned.
 */
double Storage::
getFirstTime() const
{
	if(_storage.getSize()<=0) {
		return(rdMath::NAN);
	}
	return(_storage[0].getTime());
}
//_____________________________________________________________________________
/**
 * Get the time of the last states.
 *
 * @return Time of the first stored states.  If there is no stored state,
 * the constant rdMath::NAN (not a number) is returned.
 */
double Storage::
getLastTime() const
{
	if(_storage.getSize()<=0) {
		return(rdMath::NAN);
	}
	return(_storage.getLast().getTime());
}
//_____________________________________________________________________________
/**
 * Get the smallest time step.
 *
 * @return Smallest time step.  If there are less than 2 state vectors, rdMath::PLUS_INFINITY is returned.
 */
double Storage::
getMinTimeStep() const
{
	double *time=NULL;
	int n = getTimeColumn(time);
	double dtmin = rdMath::PLUS_INFINITY;
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
 *	@param aTimeIndex Time index at which to get the states.
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
 *	@param aTimeIndex Time index at which to get the states.
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
 * Get the first aN states at a specified time.
 * The values of the states are determined by linear interpolation.
 *
 *	@param aT Time at which to get the states.
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
	if(den<rdMath::ZERO) {
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
 *	@param aT Time at which to get the states.
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
		cout<<"Storage.setDataColumn: ERR- sizes don't match." << endl;
		return;
	}

	// ASSIGNMENT
	for(int i=0;i<n;i++) {
		StateVector *vec = getStateVector(i);
		if(vec==NULL) continue;
		vec->setDataValue(aStateIndex,aData[i]);
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
	return getDataColumn(getStateIndex(aColumnName), rData);
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
		_storage.getLast() = aStateVector;
	else
		_storage.append(aStateVector);
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
	_storage.append(aStorage);
	return(_storage.getSize());
}
//_____________________________________________________________________________
/**
 * Append an array of data that occured at a specified time.
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
	if(aN<=0) return(_storage.getSize());

	// APPEND
	StateVector vec(aT,aN,aY);
	// TODO: use some tolerance when checking for duplicate time?
	if(aCheckForDuplicateTime && _storage.getSize() && _storage.getLast().getTime()==vec.getTime())
		_storage.getLast() = vec;
	else
		_storage.append(vec);

	return(_storage.getSize());
}
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
 * @param aN Length of aY
 * @param aY Array of values to add to the state vectors.
 * @see StateVector::add(int,double[])
 */
void Storage::
add(int aN,double aY[])
{
	for(int i=0;i<_storage.getSize();i++) {
		_storage[i].add(aN,aY);
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
		_storage[i].add(nN,Y);
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
 * @param aN Length of aY
 * @param aY Array of values to subtract from the state vectors.
 * @see StateVector::subtract(int,double[])
 */
void Storage::
subtract(int aN,double aY[])
{
	for(int i=0;i<_storage.getSize();i++) {
		_storage[i].subtract(aN,aY);
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
		_storage[i].subtract(nN,Y);
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
 * @param aValue Value by which to mutiply the state vectors.
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
 * @param aN Length of aY
 * @param aY Array of values the states are to be multiplied by.
 * @see StateVector::multiply(int,double[])
 */
void Storage::
multiply(int aN,double aY[])
{
	for(int i=0;i<_storage.getSize();i++) {
		_storage[i].multiply(aN,aY);
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
 * Multipy this storage instance by a storage instance.
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
		_storage[i].multiply(nN,Y);
	}

	// CLEANUP
	if(Y!=NULL) delete[] Y;
}
//_____________________________________________________________________________
/**
 * Multiply entries at column aIndex by a value.
 *
 * @param aIndex is the index of the column to multiply
 * @param aValue Value by which to mutiply the column.
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
 * @param aN Length of aY
 * @param aY Array of values the states are to be divided by.
 */
void Storage::
divide(int aN,double aY[])
{
	for(int i=0;i<_storage.getSize();i++) {
		_storage[i].divide(aN,aY);
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
		_storage[i].divide(nN,Y);
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
 * simply divides by the the time interval (tf-ti).
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
		cout << "Storage.computeAverage: ERROR- time interval invalid." << endl
			  << "\tfirstTime=" << ti << "  lastTime=" << tf << endl;
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
		cout << "Storage.integrate: ERROR- no stored states." << endl;
		return(0);
	}

	// CHECK INDICES
	if(aI1>=aI2) {
		cout << "Storage.integrate:  ERROR- aI1 >= aI2." << endl;
		return(0);
	}

	// GET THE SMALLEST NUMBER OF STATES
	int n = getSmallestNumberOfStates();
	if(n>aN) n = aN;
	if(n<=0) {
		cout << "Storage.computeArea: ERROR- no stored states" << endl;
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
		cout << "Storage.integrate: ERROR- no stored states." << endl;
		return(0);
	}

	// CHECK INITIAL AND FINAL TIMES
	if(aTI>=aTF) {
		cout << "Storage.integrate:  ERROR- bad time range." << endl
			  << "\tInitial time (" << aTI << ") is not smaller than final time (" << aTF << ")" << endl;
		return(0);
	}

	// CHECK TIME RANGE
	double fstT = getFirstTime();
	double lstT = getLastTime();
	if((aTI<fstT)||(aTI>lstT)||(aTF<fstT)||(aTF>lstT)) {
		cout << "Storage.integrate: ERROR- bad time range." << endl
			  << "\tThe specified range (" << aTI << " to " << aTF << ") is not covered by" << endl
			  << "\ttime range of the stored states (" << fstT << " to " << lstT << ")." << endl;
		return(0);
	}

	// CHECK FOR VALID OUTPUT ARRAYS
	if(aN<=0) return(0);

	// GET THE SMALLEST NUMBER OF STATES
	int n = getSmallestNumberOfStates();
	if(n>aN) n = aN;
	if(n<=0) {
		cout << "Storage.integrate: ERROR- no stored states" << endl;
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

	// GET RELAVENT STATE INDICES
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

//_____________________________________________________________________________
/**
 * Lowpass filter each of the columns in the storage.  Note that as a part
 * of this operation, the storage is resampled so that the statevectors are
 * at equal spacing.
 *
 * @param aOrder Order of the FIR filter.
 * @param aCutoffFrequency Cutoff frequency.
 */
void Storage::
lowpassFIR(int aOrder,double aCutoffFrequency)
{
	double dtmin = getMinTimeStep();
	if(dtmin<rdMath::ZERO) {
		cout<<"Storage.lowpassFIR: storage cannot be resampled."<<endl;
		return;
	}

	// RESAMPLE
	resample(dtmin,5);
	int size = getSize();
	if(size<(2*aOrder)) {
		cout<<"Storage.lowpassFIR: too few data points to filter."<<endl;
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
 * Find the index of the storage element that occured immediately before
 * or at time aT ( aT <= getTime(index) ).
 *
 * This method can be much more efficient than findIndex(aT) if a good guess
 * is made for aI.
 * If aI corresponds to a state which occured later than aT, an exhaustive
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
 * Find the index of the storage element that occured immediately before
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
	if(aStartTime > aEndTime) {
		double tmp = aStartTime;
		aStartTime = aEndTime;
		aEndTime = tmp;
	}

	oStartFrame = findIndex(0, aStartTime);
	oEndFrame = findIndex(getSize()-1, aEndTime);
}
//_____________________________________________________________________________
/**
 * Resample Storage columns to specified rate. This's done by fitting splines
 * to Storage columns and resampling
 *
 * @param aDT Time interval between adjacent statevectors.
 */
void Storage::
resample(const double aDT, const int aDegree)
{
	int numDataRows = _storage.getSize();

	if(numDataRows<=1) return;

	GCVSplineSet *splineSet = new GCVSplineSet(aDegree,this);

	Array<std::string> saveLabels = getColumnLabels();
	// Free up memory used by Storage
	_storage.setSize(0);
	// For every column, collect data and fit spline to originalTimes, dataColumn.
	Storage *newStorage = splineSet->constructStorage(0,aDT);

	copyData(*newStorage);

	setColumnLabels(saveLabels);

	delete newStorage;
	delete splineSet;
}


//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Print the contents of this storage instance to standard output.
 */
void Storage::
print() const
{
	cout<<"\nStorage.print:  "<<getName()<<"."<<endl;
	for(int i=0;i<_storage.getSize();i++) {
		getStateVector(i)->print();
	}
}
//_____________________________________________________________________________
/**
 * Print the contents of this storage instance to a file.
 *
 * The argument aMode specifies whether the file is openned for writting, "w",
 * or appending, "a".  If a bad value for aMode is sent in, the file is openned
 * for writing.
 *
 * The total number of characters written is returned.  If an error occured,
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
		cout << "Storage.print(const string&,const string&): failed to" << endl
			  << " write header to file " << aFileName << endl;
		return(false);
	}

	// WRITE SIMM HEADER
	if(_writeSIMMHeader) {
		n = writeSIMMHeader(fp, -1, aComment.c_str());
		if(n<0) {
			cout << "Storage.print(const string&,const string&): failed to" << endl
				  << " write SIMM header to file " << aFileName << endl;
			return(false);
		}
	}
	
	// WRITE THE DESCRIPTION
	n = writeDescription(fp);
	if(n<0) {
		cout << "Storage.print(const string&,const string&): failed to" << endl
			  << " write description to file " << aFileName << endl;
		return(false);
	}

	// WRITE THE COLUMN LABELS
	n = writeColumnLabels(fp);
	if(n<0) {
		cout << "Storage.print(const string&,const string&): failed to" << endl
			  << " write column labels to file " << aFileName << endl;
		return(false);
	}

	// VECTORS
	for(int i=0;i<_storage.getSize();i++) {
		n = getStateVector(i)->print(fp);
		if(n<0) {
			cout << "Storage.print(const string&,const string&): error printing to " << aFileName;
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
 * The argument aMode specifies whether the file is openned for writting, "w",
 * or appending, "a".  If a bad value for aMode is sent in, the file is openned
 * for writing.
 *
 * The argument aDT specifies the time spacing.
 *
 * The total number of characters written is returned.  If an error occured,
 * a negative number is returned.
 */
int Storage::
print(const string &aFileName,double aDT,const string &aMode) const
{
	// CHECK FOR VALID DT
	if(aDT<=0) return(0);

	// OPEN THE FILE
	FILE *fp = IO::OpenFile(aFileName,aMode);
	if(fp==NULL) return(-1);

	// HOW MANY TIME STEPS?
	double ti = getFirstTime();
	double tf = getLastTime();
	int nr = IO::ComputeNumberOfSteps(ti,tf,aDT);

	// WRITE THE HEADER
	int n,nTotal=0;
	n = writeHeader(fp,aDT);
	if(n<0) {
		cout << "Storage.print(const string&,const string&,double): failed to" << endl
			  << " write header of file " << aFileName << endl;
		return(n);
	}

	// WRITE SIMM HEADER
	if(_writeSIMMHeader) {
		n = writeSIMMHeader(fp,aDT);
		if(n<0) {
			cout << "Storage.print(const string&,const string&): failed to" << endl
				  << " write SIMM header to file " << aFileName << endl;
			return(n);
		}
	}

	// WRITE THE DESCRIPTION
	n = writeDescription(fp);
	if(n<0) {
		cout << "Storage.print(const string&,const string&): failed to" << endl
			  << " write description to file " << aFileName << endl;
		return(n);
	}

	// WRITE THE COLUMN LABELS
	n = writeColumnLabels(fp);
	if(n<0) {
		cout << "Storage.print(const string&,const string&): failed to" << endl
			  << " write column labels to file " << aFileName << endl;
		return(n);
	}

	// LOOP THROUGH THE DATA
	int i,ny=0;
	double t,*y=NULL;
	StateVector vec;
	for(t=ti,i=0;i<nr;i++,t=ti+aDT*(double)i) {

		// INTERPOLATE THE STATES
		ny = getDataAtTime(t,ny,&y);
		vec.setStates(t,ny,y);

		// PRINT
		n = vec.print(fp);
		if(n<0) {
			cout << "Storage.print(const string&,const string&): error printing to " << aFileName;
			return(n);
		}
		nTotal += n;		
	}

	// CLEANUP
	fclose(fp);
	if(y!=NULL) { delete[] y;  y=NULL; }

	return(nTotal);
}

void Storage::
printResult(const Storage *aStorage,const std::string &aName,
				const std::string &aDir,double aDT,const std::string &aExtension)
{
	if(!aStorage) return;
	std::string path = (aDir=="") ? "." : aDir;
	std::string name = path + "/" + aName + aExtension;
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
	fprintf(rFP,"nRows=%d\n",nr);
	fprintf(rFP,"nColumns=%d\n",nc);

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
	 * looking for a match (by time) in the rows of SimmMotionData.
	 * If you find a match, add the columns in the SimmMotionData
	 * to the end of the state vector in the Storage. If you
	 * don't find one, it's a fatal error so throw an exception.
	 * Don't add a column if its name is 'unassigned'.
	 */
	int i, j, startIndex, endIndex;
	rStorage.findFrameRange(aStartTime, aEndTime, startIndex, endIndex);
	int numColumns=getColumnLabels().getSize();
	for (i = startIndex; i <= endIndex; i++)
	{
		rStorage.getTime(i, stateTime);
		for (j = 0; j < getSize(); j++)
		{
			/* Assume that the first column is 'time'. */
			time = getStateVector(j)->getTime();
			if (EQUAL_WITHIN_TOLERANCE(time, stateTime, 0.0001))
			{
				Array<double>& states = rStorage.getStateVector(i)->getData();
				for (int k = 1; k < numColumns; k++)	// Start at 1 to avoid duplicate time column
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
		if (j == getSize())
		{
			stringstream errorMessage;
			errorMessage << "Error: no coordinate data found at time " << stateTime << " in " << getDocumentFileName();
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
 * Check that a Token belongs to a list of resere
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
	// Parse until the end of header
	while(!done){
		// NAME
		string line = IO::ReadLine(aStream);
		// Always Strip leading and trainling spaces and tabs
		IO::TrimLeadingWhitespace(line);
		IO::TrimTrailingWhitespace(line);
		if(line.empty() && !aStream.good()) {
			cout << "Storage: ERROR- no more lines in storage file." << endl;
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
		else if(key== DEFAULT_HEADER_TOKEN){				
			break;			
		}
		else if (firstLine){	// Storage file have their names without "name prefix on first line"
			setName(line);
		}
		firstLine=false;
	}
	if(rNumColumns==0 || rNumRows==0) {
		cout << "Storage: ERROR- failed to parse header of storage file." << endl;
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
 * If that was a SIMM motion file postprocess it to account for
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
			iter = _keyValueMap.find("range");	// Should we check "Range", "RANGE" too?
			if (iter !=_keyValueMap.end()){
				string rangeValue = iter->second;
				double start, end;
				sscanf(rangeValue.c_str(), "%lf %lf", &start, &end);
				if (_storage.getSize()<2){	// Something wrong throw exception unless start==end
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
				else {	// time  column from range, size
					double timeStep = (end - start)/(_storage.getSize()-1);
					_columnLabels.append("time");
					for(int i=0; i<_storage.getSize(); i++){
						Array<double>& data=_storage.get(i).getData();
						data.append(i*timeStep);
					}
					int timeColumnIndex=_columnLabels.findIndex("time");
					exchangeTimeColumnWith(timeColumnIndex-1);
				}
			}
			else {	// No time specified altogether
					throw (Exception("Storage::postProcessSIMMMotion Not implemented yet"));
			}
		}
	}
}
