#ifndef _Storage_h_
#define _Storage_h_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Storage.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

#include "osimCommonDLL.h"
#include "Object.h"
#include "StateVector.h"
#include "Units.h"
#include "SimTKcommon.h"
#include "StorageInterface.h"

const int Storage_DEFAULT_CAPACITY = 256;
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
class OSIMCOMMON_API Storage : public StorageInterface {
OpenSim_DECLARE_CONCRETE_OBJECT(Storage, StorageInterface);

//=============================================================================
// DATA
//=============================================================================
public:
	/** Default token used to mark the end of the storage description in
	a file. */
	static const char *DEFAULT_HEADER_TOKEN;
	static const char* DEFAULT_HEADER_SEPARATOR;
	static const int MAX_RESAMPLE_SIZE;
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
	/** Are angles, if any, specified in radians or degrees? */
	bool _inDegrees;
	/** Map between keys in file header and values */
	MapKeysToValues	_keyValueMap;
	/** Cache for fileName and file pointer when the file is opened so we can flush and write intermediate files if needed */
	std::string _fileName;
	FILE *_fp;
	/** Name and Description */
	std::string _name;
	std::string _description;

	/** Storage file version as written to the file */
	int _fileVersion;
	static const int LatestVersion;
//=============================================================================
// METHODS
//=============================================================================
public:
	// make this constructor explicit so you don't get implicit casting of int to Storage
	explicit Storage(int aCapacity=Storage_DEFAULT_CAPACITY,
		const std::string &aName="UNKNOWN");
	Storage(const std::string &aFileName, bool readHeadersOnly=false) SWIG_DECLARE_EXCEPTION;
	Storage(const Storage &aStorage,bool aCopyData=true);
	Storage(const Storage &aStorage,int aStateIndex,int aN,
		const char *aDelimiter="\t");
	virtual ~Storage();

#ifndef SWIG
	/** Assignment operator to copy contents of an existing storage */
	Storage& operator=(const Storage &aStorage);
#endif

	const std::string& getName() const { return _name; };
	const std::string& getDescription() const { return _description; };
	void setName(const std::string& aName) { _name = aName; };
	void setDescription(const std::string& aDescription) { _description = aDescription; };
	//--------------------------------------------------------------------------
	// VERSIONING /BACKWARD COMPATIBILITY SUPPORT
	//--------------------------------------------------------------------------	
	static const int& getLatestVersion() { return LatestVersion; };
	const int& getFileVersion() const { return _fileVersion; };
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
	virtual int getSize() const { return(_storage.getSize()); }
	// STATEVECTOR
	int getSmallestNumberOfStates() const;
	virtual StateVector* getStateVector(int aTimeIndex) const;
	virtual StateVector* getLastStateVector() const;
	// TIME
	virtual double getFirstTime() const;
	virtual double getLastTime() const;
	double getMinTimeStep() const;
	bool getTime(int aTimeIndex,double &rTime,int aStateIndex=-1) const;
	virtual int getTimeColumn(double *&rTimes,int aStateIndex=-1) const;
	virtual int getTimeColumn(Array<double>& rTimes,int aStateIndex=-1) const;
	virtual void getTimeColumnWithStartTime(Array<double>& rTimes,double startTime=0.0) const;
	// HEADERS, Key-Value pairs
	void addKeyValuePair(const std::string& aKey, const std::string& aValue);
	void getValueForKey(const std::string& aKey, std::string& rValue) const;
	bool hasKey(const std::string& aKey) const;
	const bool isInDegrees() const { return _inDegrees; };
	void setInDegrees(const bool isInDegrees) { _inDegrees = isInDegrees; };
	// DATA
	int getData(int aTimeIndex,int aStateIndex,double &rValue) const;
	int getData(int aTimeIndex,int aStateIndex,int aN,double **rData) const;
#ifndef SWIG
	int getData(int aTimeIndex,int aStateIndex,int aN,double *rData) const;
	int getData(int aTimeIndex,int aN,double **rData) const;
	int getData(int aTimeIndex,int aN,double *rData) const;
    int getData(int aTimeIndex,int aN,Array<double> &rData) const;
    int getData(int aTimeIndex,int aN,SimTK::Vector &v) const;
#endif
	int getDataAtTime(double aTime,int aN,double **rData) const;
	int getDataAtTime(double aTime,int aN,double *rData) const;
	int getDataAtTime(double aTime,int aN,Array<double> &rData) const;
    int getDataAtTime(double aTime,int aN,SimTK::Vector& v) const;
	int getDataColumn(int aStateIndex,double *&rData) const;
	int getDataColumn(int aStateIndex,Array<double> &rData) const;
    // Set entries in a column of the storage to a fixed value, 
    void setDataColumnToFixedValue(const std::string& columnName, double newValue);
	void setDataColumn(int aStateIndex,const Array<double> &aData);
	int getDataColumn(const std::string& columnName,double *&rData) const;
	void getDataColumn(const std::string& columnName, Array<double>& data, double startTime=0.0);

	/** A data block, like a vector for a force, point, etc... will span multiple "columns"
	    It is desirable to access the block as a single entity provided an identifier that is common 
	   to all components (such as prefix in the column label).
	 @param identifier	string identifying a single block of data 
	 @param rData		Array<Array<double>> of data belonging to the identifier 
     @param startTime   at what time to begin (if not 0) */
	void getDataForIdentifier(const std::string& identifier, Array< Array<double> >& rData, double startTime=0.0) const;

	/**
	 * Get indices of columns corresponding to identifier, empty array if identifier is not found in labels
	 */
	OpenSim::Array<int>  getColumnIndicesForIdentifier(const std::string& identifier) const;

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
	void purge() { _storage.setSize(0); };	// Similar to reset but doesn't try to keep history
	void crop(const double newStartTime, const double newFinalTime);
	//--------------------------------------------------------------------------
	// STORAGE
	//--------------------------------------------------------------------------
	virtual int append(const StateVector &aVec, bool aCheckForDuplicateTime=true);
	virtual int append(const Array<StateVector> &aArray);
	virtual int append(double aT,int aN,const double *aY, bool aCheckForDuplicateTime=true);
	virtual int append(double aT,const SimTK::Vector& aY, bool aCheckForDuplicateTime=true);
    virtual int append(double aT,const Array<double>& aY, bool aCheckForDuplicateTime=true);
	virtual int append(double aT, const SimTK::Vec3& aY,bool aCheckForDuplicateTime=true){
		return append(aT, 3, &aY[0], aCheckForDuplicateTime);
	}
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
	void smoothSpline(int aOrder,double aCutoffFrequency);
	void lowpassIIR(double aCutoffFequency);
	void lowpassFIR(int aOrder,double aCutoffFequency);
	// Append rows of two storages at matched time
	void addToRdStorage(Storage& rStorage, double aStartTime, double aEndTime);
	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	virtual int findIndex(double aT) const;
	virtual int findIndex(int aI,double aT) const;
	void findFrameRange(double aStartTime, double aEndTime, int& oStartFrame, int& oEndFrame) const;
	double resample(double aDT, int aDegree);
	double resampleLinear(double aDT);
	double compareColumn(Storage& aOtherStorage, 
						 const std::string& aColumnName,
						 double startTime, double endTime=-1.0);
	double compareColumnRMS(Storage& aOtherStorage, 
						 const std::string& aColumnName,
						 double startTime=SimTK::NaN, double endTime=SimTK::NaN);
	//void checkAgainstStandard(Storage standard, Array<double> &tolerances, std::string testFile = "", int testFileLine = -1, std::string errorMessage = "Exception");
	void compareWithStandard(Storage& standard, Array<std::string> &columnsUsed, Array<double> &comparisons);
	bool makeStorageLabelsUnique();
	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
	bool print(const std::string &aFileName,const std::string &aMode="w", const std::string& aComment="") const;
	int print(const std::string &aFileName,double aDT,const std::string &aMode="w") const;
	void setOutputFileName(const std::string& aFileName) ;
	// convenience function for Analyses and DerivCallbacks
	static void printResult(const Storage *aStorage,const std::string &aName,
		const std::string &aDir,double aDT,const std::string &aExtension);
    void interpolateAt(const Array<double> &targetTimes);
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
