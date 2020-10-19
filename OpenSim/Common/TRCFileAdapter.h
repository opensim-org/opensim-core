/* -------------------------------------------------------------------------- *
 *                          OpenSim:  TRCFileAdapter.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#ifndef OPENSIM_TRC_FILE_ADAPTER_H_
#define OPENSIM_TRC_FILE_ADAPTER_H_

/** @file
* TRCFileAdapter is a concrete FileAdpater for reading and writing TRC format
files. A TRC file is a Motion Analysis Trace file that contains a time trace
or trajectory of 3D markers location with respect to a lab or Ground reference
frame. A TRC file contains a structured header with information for the file
type, filename, number of markers, sampling rate, units and several others.
Data are tab-delimited rows with the same number of elements. Missing values
can be blank and/or NaN. Blank values are interpreted as NaN. Here is an
example of a valid TRC file:

   \code
PathFileType<tab>4<tab>(X/Y/Z)<tab>example.trc
DataRate<tab>CameraRate<tab>NumFrames<tab>NumMarkers<tab>Units<tab>OrigDataRate<tab>OrigDataStartFrame<tab>OrigNumFrames
100<tab>100<tab>5<tab>3<tab>m<tab>100<tab>1<tab>5
Frame#<tab>Time<tab>marker1<tab><tab><tab>marker2<tab><tab><tab>marker3<tab><tab>
<tab><tab>X1<tab>Y1<tab>Z1<tab>X2<tab>Y2<tab>Z2<tab>X3<tab>Y3<tab>Z3
1<tab>0.01<tab><tab><tab><tab>-0.152<tab>0.245<tab>-1.71<tab>-0.0517<tab>0.305<tab>-1.7
2<tab>0.02<tab>-0.273<tab>0.0745<tab>-1.57<tab>-0.152<tab>0.245<tab>-1.71<tab>-0.0517<tab>0.305<tab>-1.7
3<tab>0.03<tab>-0.273<tab>0.0745<tab>-1.57<tab>Nan<tab>nan<tab>NAN<tab>-0.0517<tab>0.305<tab>-1.7
4<tab>0.04<tab>-0.273<tab>0.0745<tab>-1.57<tab>-0.152<tab>0.245<tab>-1.71<tab>-0.0517<tab>0.305<tab>-1.7
5<tab>0.05<tab>-0.273<tab>0.0745<tab>-1.57<tab>-0.152<tab>0.245<tab>-1.71<tab>-0.0517<tab>0.305<tab>-1.7
\endcode

*/

#include "FileAdapter.h"
#include "TimeSeriesTable.h"

namespace OpenSim {

class MissingHeader : public IOError {
public:
#ifndef SWIG
    using IOError::IOError;
#endif
};

class IncorrectNumMetaDataKeys : public IOError {
public: 
    IncorrectNumMetaDataKeys(const std::string& file,
                             size_t line,
                             const std::string& func,
                             const std::string& filename,
                             size_t expected,
                             size_t received) :
        IOError(file, line, func) {
        std::string msg = "Error reading MetaData in file '" + filename + "'. ";
        msg += "Incorrect number of keys. ";
        msg += "Expected = " + std::to_string(expected) + ". ";
        msg += "Received = " + std::to_string(received) + ". ";

        addMessage(msg);
    }
};

class UnexpectedMetaDataKey : public IOError {
public:
    UnexpectedMetaDataKey(const std::string& file,
                          size_t line,
                          const std::string& func,
                          const std::string& filename,
                          const std::string& expected,
                          const std::string& received) :
        IOError(file, line, func) {
        std::string msg = "Error reading MetaData in file '" + filename + "'. ";
        msg += "Unexpected key. ";
        msg += "Expected = " + expected + ". ";
        msg += "Received = " + received + ".";

        addMessage(msg);
    }
};

class MetaDataLengthMismatch : public IOError {
public:
    MetaDataLengthMismatch(const std::string& file,
                           size_t line,
                           const std::string& func,
                           const std::string& filename,
                           size_t keys_len,
                           size_t values_len) :
        IOError(file, line, func) {
        std::string msg = "Error reading Metadata in file '" + filename + "'. ";
        msg += "Number of keys and values do not match. ";
        msg += "Keys = " + std::to_string(keys_len) + ". ";
        msg += "Values = " + std::to_string(values_len) + ". ";

        addMessage(msg);
    }
};

class IncorrectNumColumnLabels : public IOError {
public:
    IncorrectNumColumnLabels(const std::string& file,
                             size_t line,
                             const std::string& func,
                             const std::string& filename,
                             size_t expected,
                             size_t received) :
        IOError(file, line, func) {
        std::string msg = "Error reading column labels in file '" + filename;
        msg += "'. Unexpected number of column labels. ";
        msg += "Expected = " + std::to_string(expected) + ". ";
        msg += "Received = " + std::to_string(received) + ".";

        addMessage(msg);
    }
};

/** TRCFileAdapter is a FileAdapter that reads and writes TRC files. It accepts
(when writing) and returns (when reading) a specific type of DataTable referred 
to as Table in this class. Be sure to expect/provide that table when working
with this adapter.                                                            */
class OSIMCOMMON_API TRCFileAdapter : public FileAdapter {
public:
    TRCFileAdapter()                                 = default;
    TRCFileAdapter(const TRCFileAdapter&)            = default;
    TRCFileAdapter(TRCFileAdapter&&)                 = default;
    TRCFileAdapter& operator=(const TRCFileAdapter&) = default;
    TRCFileAdapter& operator=(TRCFileAdapter&&)      = default;
    ~TRCFileAdapter()                                = default;
    
    TRCFileAdapter* clone() const override;

    /** Write a table to a TRC file. The filename provided need not contain 
    ".trc".                                                                   */
    static
    void write(const TimeSeriesTableVec3& table, const std::string& filename);

    /** Key used for table associative array returned/accepted by write/read. */
    static const std::string              _markers;

protected:
    /** Implementation of the read functionality.                             */
    OutputTables extendRead(const std::string& filename) const override;

    /** Implementation of the write functionality.                            */
    void extendWrite(const InputTables& tables, 
                     const std::string& filename) const override;
    
private:
    /** Delimiter used for parsing the header of TRC file.                    */
    static const std::string              _headerDelimiters;
    /** Delimiter used for writing.                                           */
    static const std::string              _delimiterWrite;
    /** Delimiters used for reading.                                          */
    static const std::string              _delimitersRead;
    /** Column label of the column representing frame number.                 */
    static const std::string              _frameNumColumnLabel;
    /** Column label of the column representing time.                         */
    static const std::string              _timeColumnLabel;
    /** Letter used to represent x-component.                                 */
    static const std::string              _xLabel;
    /** Letter used to represent y-component.                                 */
    static const std::string              _yLabel;
    /** Letter used to represent z-component.                                 */
    static const std::string              _zLabel;
    /** Metadata key representing number of markers.                          */
    static const std::string              _numMarkersLabel;
    /** Metadata key representing number of frames.                           */
    static const std::string              _numFramesLabel;
    /** Line number at which the data rows start.                             */
    static const unsigned                 _dataStartsAtLine;
    /** Ordered collection of metadata keys.                                  */
    static const std::vector<std::string> _metadataKeys;
};

} // namespace OpenSim

#endif // OPENSIM_TRC_FILE_ADAPTER_H_
