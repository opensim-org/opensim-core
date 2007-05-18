#ifndef _Storage_h_
#define _Storage_h_
// Storage.h
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

#include "osimCommonDLL.h"
#include "Object.h"
#include "StateVector.h"
#include "Units.h"

const int Storage_DEFAULT_CAPACITY = 256;

template class OSIMCOMMON_API OpenSim::Array<OpenSim::StateVector>;

//=============================================================================
//=============================================================================
/**
 * A class for storing an array of statevectors.  A statevector is an
 * array of data that has an associated time stamp (see StateVector).
 * Generally, it is used to store the time histories of the states during
 * an integration, but may be used for a variety of applications.  Note that
 * it is assumed by several methods in this class that the time stamps of
 * stored statevectors are monotonically increasing.
 *
 * When stored as a file, the statevectors are stored in rows.  This first
 * value in a row is the time stamp at which the states occured.  The
 * rest of the elements in a row are the states.  Therefore, each column of
 * data in a file corresponds to a particular state.
 *
 * In an Storage object, statevectors (or rows) are indexed by the
 * TimeIndex, and a particular state (or column) is indexed by the
 * StateIndex.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
namespace OpenSim { 

typedef std::map<std::string, std::string, std::less<std::string> > MapKeysToValues;

//static std::string[] simmReservedKeys;
class OSIMCOMMON_API Storage : public Object
{

//=============================================================================
// DATA
//=============================================================================
public:
	/** Large negative number. */
	static const double LARGE_NEGATIVE;
	/** Large positive number. */
	static const double LARGE_POSITIVE;
	/** Default token used to mark the end of the storage description in
	a file. */
	static const char *DEFAULT_HEADER_TOKEN;
	static const char* DEFAULT_HEADER_SEPARATOR;
protected:
	static std::string simmReservedKeys[];

	/** Array of StateVectors. */
	Array<StateVector> _storage;
	/** Token used to mark the end of the description in a file. */
	std::string _headerToken;
	/** Column labels. */
	Array<std::string> _columnLabels;
	/** Step interval at which states in a simulation are stored. See
	store(). */
	int _stepInterval;
	/** Last index at which a search was started. */
	mutable int _lastI;
	/** Flag for whether or not to insert a SIMM style header. */
	bool _writeSIMMHeader;
	/** Units in which the data is represented. */
	Units _units;
	/** Map between keys in file header and values */
	MapKeysToValues	_keyValueMap;
//=============================================================================
// METHODS
//=============================================================================
public:
	// make this constructor explicit so you don't get implicit casting of int to Storage
	explicit Storage(int aCapacity=Storage_DEFAULT_CAPACITY,
		const std::string &aName="UNKNOWN");
	Storage(const std::string &aFileName);
	Storage(const Storage &aStorage,bool aCopyData=true);
	Storage(const Storage &aStorage,int aStateIndex,int aN,
		const char *aDelimiter="\t");
	virtual Object* copy() const;
	virtual ~Storage();

private:
	//--------------------------------------------------------------------------
	// CONSTRUCTION METHODS
	//--------------------------------------------------------------------------
	void allocateCapacity();
	void setNull();
	void copyData(const Storage &aStorage);
	void parseColumnLabels(const char *aLabels);
	bool parseHeaders(std::ifstream& aStream, int& rNumRows, int& rNumColumns);
	bool isSimmReservedToken(const std::string& aToken);
	void postProcessSIMMMotion();
	void exchangeTimeColumnWith(int aColumnIndex);
public:

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// SIZE
	int getSize() const { return(_storage.getSize()); }
	// STATEVECTOR
	int getSmallestNumberOfStates() const;
	StateVector* getStateVector(int aTimeIndex) const;
	StateVector* getLastStateVector() const;
	// TIME
	double getFirstTime() const;
	double getLastTime() const;
	int getTime(int aTimeIndex,double &rTime,int aStateIndex=-1) const;
	int getTimeColumn(double *&rTimes,int aStateIndex=-1);
	void getTimeColumn(Array<double>& times, const double& startTime=0.0);
	// HEADERS, Key-Value pairs
	void addKeyValuePair(const std::string& aKey, const std::string& aValue);
	void getValueForKey(const std::string& aKey, std::string& rValue) const;
	bool hasKey(const std::string& aKey) const;
	// DATA
	int getData(int aTimeIndex,int aStateIndex,double &rValue) const;
	int getData(int aTimeIndex,int aStateIndex,int aN,double **rData) const;
	int getData(int aTimeIndex,int aStateIndex,int aN,double *rData) const;
	int getData(int aTimeIndex,int aN,double **rData) const;
	int getData(int aTimeIndex,int aN,double *rData) const;
	int getDataAtTime(double aTime,int aN,double **rData) const;
	int getDataAtTime(double aTime,int aN,double *rData) const;
	int getDataColumn(int aStateIndex,double *&rData) const;
	void setDataColumn(int aStateIndex,const Array<double> &aData);
	int getDataColumn(const std::string& columnName,double *&rData) const;
	void getDataColumn(const std::string& columnName, Array<double>& data, const double& startTime=0.0);
	// STEP INTERVAL
	void setStepInterval(int aStepInterval);
	int getStepInterval() const;
	// CAPACITY INCREMENT
	void setCapacityIncrement(int aIncrement);
	int getCapacityIncrement() const;
	// IO
	void setWriteSIMMHeader(bool aTrueFalse);
	bool getWriteSIMMHeader() const;
	void setHeaderToken(const std::string &aToken);
	const std::string& getHeaderToken() const;
	// COLUMN LABELS
	const int getStateIndex(const std::string &aColumnName, int startIndex=0) const;
	void setColumnLabels(const Array<std::string> &aColumnLabels);
	const Array<std::string> &getColumnLabels() const;
	//--------------------------------------------------------------------------
	// RESET
	//--------------------------------------------------------------------------
	int reset(int aIndex=0);
	int reset(double aTime);

	//--------------------------------------------------------------------------
	// STORAGE
	//--------------------------------------------------------------------------
	virtual int append(const StateVector &aVec, bool aCheckForDuplicateTime=true);
	virtual int append(const Array<StateVector> &aArray);
	virtual int append(double aT,int aN,const double *aY, bool aCheckForDuplicateTime=true);
	virtual int store(int aStep,double aT,int aN,const double *aY);

	//--------------------------------------------------------------------------
	// OPERATIONS
	//--------------------------------------------------------------------------
	void shiftTime(double aValue);
	void scaleTime(double aValue);
	void add(double aValue);
	void add(int aN,double aY[]);
	void add(int aN,double aValue);
	void add(StateVector *aStateVector);
	void add(Storage *aStorage);
	void subtract(double aValue);
	void subtract(int aN,double aY[]);
	void subtract(StateVector *aStateVector);
	void subtract(Storage *aStorage);
	void multiply(double aValue);
	void multiplyColumn(int aIndex, double aValue);
	void multiply(int aN,double aY[]);
	void multiply(StateVector *aStateVector);
	void multiply(Storage *aStorage);
	void divide(double aValue);
	void divide(int aN,double aY[]);
	void divide(StateVector *aStateVector);
	void divide(Storage *aStorage);
	Storage* integrate(int aI1=-2,int aI2=-1) const;
	Storage* integrate(double aT1,double aT2) const;
	int computeArea(int aN,double *aArea) const;
	int computeArea(double aTI,double aTF,int aN,double *aArea) const;
	int computeAverage(int aN,double *aAve) const;
	int computeAverage(double aTI,double aTF,int aN,double *aAve) const;
	void pad(int aPadSize);
	void lowpassFIR(int aOrder,double aCutoffFequency);
	// Append rows of two storages at matched time
	void addToRdStorage(Storage& rStorage, double aStartTime, double aEndTime);
	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	int findIndex(double aT) const;
	int findIndex(int aI,double aT) const;
	void resample(const double aDT, const int aDegree);
	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
	void print() const;
	bool print(const std::string &aFileName,const std::string &aMode="w", const std::string& aComment="") const;
	int print(const std::string &aFileName,double aDT,const std::string &aMode="w") const;
	// convenience function for Analyses and DerivCallbacks
	static void printResult(const Storage *aStorage,const std::string &aName,
		const std::string &aDir,double aDT,const std::string &aExtension);
private:
	int writeHeader(FILE *rFP,double aDT=-1) const;
	int writeSIMMHeader(FILE *rFP,double aDT=-1, const char*aComment=0) const;
	int writeDescription(FILE *rFP) const;
	int writeColumnLabels(FILE *rFP) const;
	int integrate(double aTI,double aTF,int aN,double *rArea,Storage *rStorage) const;
	int integrate(int aI1,int aI2,int aN,double *rArea,Storage *rStorage) const;

//=============================================================================
};	// END of class Storage

}; //namespace
//=============================================================================
//=============================================================================

#endif //__Storage_h__
