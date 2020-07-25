#ifndef OPENSIM_STORAGE_H_
#define OPENSIM_STORAGE_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Storage.h                             *
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

#include "StateVector.h"
#include "Units.h"
#include "StorageInterface.h"
#include "TimeSeriesTable.h"

const int Storage_DEFAULT_CAPACITY = 256;
//=============================================================================
//=============================================================================
namespace OpenSim { 

typedef std::map<std::string, std::string, std::less<std::string> > MapKeysToValues;

//static std::string[] simmReservedKeys;
 
/**
 * A class for storing an array of statevectors.  A statevector is an
 * array of data that has an associated time stamp (see StateVector).
 * Generally, it is used to store the time histories of the states during
 * an integration, but may be used for a variety of applications.  Note that
 * it is assumed by several methods in this class that the time stamps of
 * stored statevectors are monotonically increasing.
 *
 * When stored as a file, the statevectors are stored in rows.  This first
 * value in a row is the time stamp at which the states occurred.  The
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
    MapKeysToValues _keyValueMap;
    /** Cache for fileName and file pointer when the file is opened so we can flush and write intermediate files if needed */
    std::string _fileName;
    FILE *_fp;
    /** Name and Description */
    std::string _name;
    std::string _description;

    /** Storage file version as written to the file */
    int _fileVersion = -1;
    static const int LatestVersion;
//=============================================================================
// METHODS
//=============================================================================
public:
    // make this constructor explicit so you don't get implicit casting of int to Storage
    explicit Storage(int aCapacity=Storage_DEFAULT_CAPACITY,
        const std::string &aName="UNKNOWN");
    /** Load a data file into a Storage. 
    <b>Version 2 STO files</b>: This constructor can read MOT, unversioned, 
    version 1 and 2 STO files, and any data file format supported by FileAdapter,
    which includes C3D and TRC files. Several of these formats support tables of
    different element types (e.g. Vec3, Quaternion, SpatialVec), in which case these
    elements are flattened to scalar values with more columns.
    @see DataTable_::faltten()
    Note, this capability was introduced to support plotting OutputReporter results
    in the GUI and is not recommended for API users. In addition to only supporting
    scalar elements, Storage does not preserve the metadata that can be contained by
    version 2 STO files and read in from C3D and TRC files via their respective
    FileAdapters. In the case of data files with multiple tables (like C3D files)
    only the first table read is converted into a Storage.
    Please use FileAdapter (STOFileAdpater, C3DFileAdapter, ...) and TimeSeriesTable
    instead, whenever possible. */
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
    int getSize() const override { return(_storage.getSize()); }
    // STATEVECTOR
    int getSmallestNumberOfStates() const;
    StateVector* getStateVector(int aTimeIndex) const override;
    StateVector* getLastStateVector() const override;
    // TIME
    double getFirstTime() const override;
    double getLastTime() const override;
    double getMinTimeStep() const;
    bool getTime(int aTimeIndex,double &rTime,int aStateIndex=-1) const;
    virtual int getTimeColumn(double *&rTimes,int aStateIndex=-1) const;
    int getTimeColumn(Array<double>& rTimes,int aStateIndex=-1) const override;
    void getTimeColumnWithStartTime(Array<double>& rTimes,double startTime=0.0) const override;
    // HEADERS, Key-Value pairs
    void addKeyValuePair(const std::string& aKey, const std::string& aValue);
    void getValueForKey(const std::string& aKey, std::string& rValue) const;
    bool hasKey(const std::string& aKey) const;
    bool isInDegrees() const { return _inDegrees; };
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
    int getDataAtTime(double aTime,int aN,Array<double> &rData) const override;
    int getDataAtTime(double aTime,int aN,SimTK::Vector& v) const;
    int getDataColumn(int aStateIndex,double *&rData) const;
    int getDataColumn(int aStateIndex,Array<double> &rData) const;
    // Set entries in a column of the storage to a fixed value, 
    void setDataColumnToFixedValue(const std::string& columnName, double newValue);
    void setDataColumn(int aStateIndex,const Array<double> &aData);
    int getDataColumn(const std::string& columnName,double *&rData) const;
    void getDataColumn(const std::string& columnName, Array<double>& data, double startTime=0.0) override;

    /** Convert to a TimeSeriesTable. This may be useful if you need to use
    parts of the API that require a TimeSeriesTable instead of a Storage. */
    TimeSeriesTable exportToTable() const;

#ifndef SWIG
    /** A data block, like a vector for a force, point, etc... will span multiple "columns"
        It is desirable to access the block as a single entity provided an identifier that is common 
       to all components (such as prefix in the column label).
     @param identifier  string identifying a single block of data 
     @param rData       Array<Array<double>> of data belonging to the identifier 
     @param startTime   at what time to begin (if not 0) */
    void getDataForIdentifier(const std::string& identifier, Array< Array<double> >& rData, double startTime=0.0) const;
#endif
    /**
     * Get indices of columns whose labels begin with the specified "identifier"
     * (prefix). Returns an empty Array if none of the column labels begin with
     * the identifier.
     */
    OpenSim::Array<int>
        getColumnIndicesForIdentifier(const std::string& identifier) const;

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
    /** Get the column index corresponding to specified column name. This
     * function attempts to handle the change in state variable names that
     * occurred in OpenSim version 4.0; for example, if you search for
     * `<coord-name>/speed` and it is not found, then this function looks for
     * `<coord-name>_u`.
     *
     * @return State index of column or -1.  Note that the returned index is
     * equivalent to the state index.  For example, for the first column in a
     * storage (usually time) -1 would be returned.  For the second column in a
     * storage (the first state) 0 would be returned. */
    int getStateIndex(const std::string &aColumnName, int startIndex=0) const;
    void setColumnLabels(const Array<std::string>& aColumnLabels);
    const Array<std::string>& getColumnLabels() const;
    //--------------------------------------------------------------------------
    // RESET
    //--------------------------------------------------------------------------
    int reset(int aIndex=0);
    int reset(double aTime);
    void purge() { _storage.setSize(0); };  // Similar to reset but doesn't try to keep history
    void crop(const double newStartTime, const double newFinalTime);
    //--------------------------------------------------------------------------
    // STORAGE
    //--------------------------------------------------------------------------
    int append(const StateVector &aVec, bool aCheckForDuplicateTime=true) override;
    int append(const Array<StateVector> &aArray) override;
    int append(double aT,int aN,const double *aY, bool aCheckForDuplicateTime=true) override;
    int append(double aT,const SimTK::Vector& aY, bool aCheckForDuplicateTime=true) override;
    virtual int append(double aT,const Array<double>& aY, bool aCheckForDuplicateTime=true);
    int append(double aT, const SimTK::Vec3& aY,bool aCheckForDuplicateTime=true) override {
        return append(aT, 3, &aY[0], aCheckForDuplicateTime);
    }
    int store(int aStep,double aT,int aN,const double *aY) override;

    //--------------------------------------------------------------------------
    // OPERATIONS
    //--------------------------------------------------------------------------
    void shiftTime(double aValue);
    void scaleTime(double aValue);
    void add(double aValue);
    void add(const SimTK::Vector_<double>& values);
    void add(int aN,double aValue);
    void add(StateVector *aStateVector);
    void add(Storage *aStorage);
    void subtract(double aValue);
    void subtract(const SimTK::Vector_<double>& values);
    void subtract(StateVector *aStateVector);
    void subtract(Storage *aStorage);
    void multiply(double aValue);
    void multiplyColumn(int aIndex, double aValue);
    void multiply(const SimTK::Vector_<double>& values);
    void multiply(StateVector *aStateVector);
    void multiply(Storage *aStorage);
    void divide(double aValue);
    void divide(const SimTK::Vector_<double>& values);
    void divide(StateVector *aStateVector);
    void divide(Storage *aStorage);
    Storage* integrate(int aI1=-2,int aI2=-1) const;
    Storage* integrate(double aT1,double aT2) const;
    int computeArea(int aN,double *aArea) const;
    int computeArea(double aTI,double aTF,int aN,double *aArea) const;
    int computeAverage(int aN,double *aAve) const;
    int computeAverage(double aTI,double aTF,int aN,double *aAve) const;
    void pad(int aPadSize);
    /**
    * Smooth spline each of the columns in the storage.  Note that as a part
    * of this operation, the storage is re-sampled to obtain uniform samples
    * unless its time steps are already uniform.
    *
    * @param order Order of the spline.
    * @param cutoffFrequency Cutoff frequency of the smoothing filter.
    */
    void smoothSpline(int order,double cutoffFrequency);
    /**
    * Low-pass filter each of the columns in the storage using a 3rd order
    * lowpass IIR Butterworth digital filter. Note that as a part of this
    * operation, the storage is re-sampled to obtain uniform samples unless
    * its time steps are already uniform.
    *
    * @param cutoffFrequency Cutoff frequency of the lowpass filter.
    */
    void lowpassIIR(double cutoffFrequency);
    /**
    * Lowpass filter each of the columns in the storage using an FIR non-
    * recursive digital filter. Note that as a part of this operation, the
    * storage is re-sampled to obtain uniform samples unless its time steps
    * are already uniform.
    *
    * @param order Order of the FIR filter.
    * @param cutoffFrequency Cutoff frequency.
    */
    void lowpassFIR(int order, double cutoffFrequency);
    // Append rows of two storages at matched time
    void addToRdStorage(Storage& rStorage, double aStartTime, double aEndTime);
    //--------------------------------------------------------------------------
    // UTILITY
    //--------------------------------------------------------------------------
    int findIndex(double aT) const override;
    int findIndex(int aI,double aT) const override;
    void findFrameRange(double aStartTime, double aEndTime, int& oStartFrame, int& oEndFrame) const;
    double resample(double aDT, int aDegree);
    double resampleLinear(double aDT);
    double compareColumn(Storage& aOtherStorage, 
                         const std::string& aColumnName,
                         double startTime, double endTime=-1.0);
    double compareColumnRMS(const Storage& aOtherStorage, 
                            const std::string& aColumnName,
                            double startTime=SimTK::NaN, double endTime=SimTK::NaN) const;
    //void checkAgainstStandard(Storage standard, Array<double> &tolerances, std::string testFile = "", int testFileLine = -1, std::string errorMessage = "Exception");
    void compareWithStandard(const Storage& standard, 
                             std::vector<std::string>& columnsUsed, 
                             std::vector<double>& comparisons) const;
    /** Force column labels for a Storage object to become unique. This is done
     * by prepending the string (n_) as needed where n=1, 2, ...
     *
     * @returns true if labels were already unique.
     **/
    bool makeStorageLabelsUnique();
    bool storageLabelsAreUnique() const;
    //--------------------------------------------------------------------------
    // IO
    //--------------------------------------------------------------------------
    bool print(const std::string &aFileName,const std::string &aMode="w", const std::string& aComment="") const;
    int print(const std::string &aFileName,double aDT,const std::string &aMode="w") const;
    void setOutputFileName(const std::string& aFileName) override ;
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
};  // END of class Storage

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_STORAGE_H_
