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


#include "rdTools.h"
#include "Object.h"
#include "StateVector.h"


const int Storage_DEFAULT_CAPACITY = 256;

template class RDTOOLS_API OpenSim::Array<OpenSim::StateVector>;

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

class RDTOOLS_API Storage : public Object
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
	/** Array of StateVectors. */
	Array<StateVector> _storage;
	/** Token used to mark the end of the description in a file. */
	char _headerToken[Object::NAME_LENGTH];
	/** Column labels for the states, usually tab delimited. */
	char *_columnLabels;
	/** Parsed version of _columnLabels to be used to query data by column */
	Array<std::string> _columnLabelsArray;
	/** Step interval at which states in a simulation are stored. See
	store(). */
	int _stepInterval;
	/** Last index at which a search was started. */
	int _lastI;
	/** Flag for whether or not to insert a SIMM style header. */
	bool _writeSIMMHeader;

//=============================================================================
// METHODS
//=============================================================================
public:
	Storage(int aCapacity=Storage_DEFAULT_CAPACITY,
		const char *aName="UNKNOWN");
	Storage(const char* aFileName);
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
public:

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// SIZE
	int getSize() const { return(_storage.getSize()); }
	// STATEVECTOR
	int getSmallestNumberOfStates();
	StateVector* getStateVector(int aTimeIndex) const;
	StateVector* getLastStateVector() const;
	// TIME
	double getFirstTime();
	double getLastTime();
	int getTime(int aTimeIndex,double &rTime,int aStateIndex=-1) const;
	int getTimeColumn(double *&rTimes,int aStateIndex=-1);
	// DATA
	int getData(int aTimeIndex,int aStateIndex,double &rValue) const;
	int getData(int aTimeIndex,int aStateIndex,int aN,double *rData) const;
	int getData(int aTimeIndex,int aN,double **rData) const;
	int getData(int aTimeIndex,int aN,double *rData) const;
	int getDataAtTime(double aTime,int aN,double **rData);
	int getDataAtTime(double aTime,int aN,double *rData);
	int getDataColumn(int aStateIndex,double *&rData) const;
	void setDataColumn(int aStateIndex,const Array<double> &aData);
	int getDataColumn(std::string& columnName,double *&rData) const;
	// STEP INTERVAL
	void setStepInterval(int aStepInterval);
	int getStepInterval() const;
	// CAPACITY INCREMENT
	void setCapacityIncrement(int aIncrement);
	int getCapacityIncrement() const;
	// IO
	void setWriteSIMMHeader(bool aTrueFalse);
	bool getWriteSIMMHeader() const;
	void setHeaderToken(const char *aToken);
	const char* getHeaderToken() const;
	// COLUMN LABELS
	const int getColumnIndex(const std::string &aColumnName) const;
	void setColumnLabels(const char *aLabels=NULL);
	const char* getColumnLabels() const;
	const Array<std::string> &getColumnLabelsArray() const;
	//--------------------------------------------------------------------------
	// RESET
	//--------------------------------------------------------------------------
	int reset(int aIndex=0);
	int reset(double aTime);

	//--------------------------------------------------------------------------
	// STORAGE
	//--------------------------------------------------------------------------
	virtual int append(const StateVector &aVec);
	virtual int append(const Array<StateVector> &aArray);
	virtual int append(double aT,int aN,const double *aY);
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
	Storage* integrate(int aI1=-2,int aI2=-1);
	Storage* integrate(double aT1,double aT2);
	int computeArea(int aN,double *aArea);
	int computeArea(double aTI,double aTF,int aN,double *aArea);
	int computeAverage(int aN,double *aAve);
	int computeAverage(double aTI,double aTF,int aN,double *aAve);
	void pad(int aPadSize);
	void lowpassFIR(int aOrder,double aCutoffFequency);

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	int findIndex(double aT);
	int findIndex(int aI,double aT);
	void resample(const double aDT, const int aDegree);
	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
	void print();
	bool print(const char *aFileName,const char *aMode="w");
	int print(const char *aFileName,double aDT,const char *aMode="w");
private:
	int writeHeader(FILE *rFP,double aDT=-1);
	int writeSIMMHeader(FILE *rFP,double aDT=-1);
	int writeDescription(FILE *rFP);
	int writeColumnLabels(FILE *rFP);
	int writeDefaultColumnLabels(FILE *rFP);

//=============================================================================
};	// END of class Storage

}; //namespace
//=============================================================================
//=============================================================================

#endif //__Storage_h__
