/* -------------------------------------------------------------------------- *
 *                          OpenSim:  STOFileAdapter.h                        *
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

#ifndef OPENSIM_STO_FILE_ADAPTER_H_
#define OPENSIM_STO_FILE_ADAPTER_H_

#include "DelimFileAdapter.h"


namespace OpenSim {

class STODataTypeNotSupported : public Exception {
public:
    STODataTypeNotSupported(const std::string& file,
                            size_t line,
                            const std::string& func,
                            const std::string& datatype) :
        Exception(file, line, func) {
        std::string msg = "Datatype '" + datatype + "' is not supported.";

        addMessage(msg);
    }
};

/** STOFileAdapter is a DelimFileAdapter that presets the delimiters 
appropriately for STO files. The format of the file is as follows:
\code
.................................
.................................
........ <metadata> .............
.................................
.................................
endheader
time colname1 colname2 ..........
.................................
.................................
............ <data-rows> ........
.................................
.................................
\endcode
For example a TimeSeriesTable_<SimTK::Vec3> could look like:
\code
units=m
DataType=Vec3
endheader
time  r_shoulder      l_shoulder      r_leg           l_leg
0.1   0.11,0.22,0.33  0.44,0.55,0.66  0.77,0.88,0.99  0.10,0.11,0.12
0.2   0.22,0.44,0.66  0.88,0.99,0.35  0.75,0.65,0.43  0.47,0.57,0.67
\endcode
Columns are delimited by tab character. Elements within a column are delimited
by comma character. STOFileAdapter allows reading/writing following
tables:
<table>
<tr><td>TimeSeriesTable_<double></td>
<td>DataType=double</td></tr>
<tr><td>TimeSeriesTable_<SimTK::Vec2></td>
<td>DataType=Vec2</td></tr>
<tr><td>TimeSeriesTable_<SimTK::Vec3></td>
<td>DataType=Vec3</td></tr>
<tr><td>TimeSeriesTable_<SimTK::Vec4></td>
<td>DataType=Vec4</td></tr>
<tr><td>TimeSeriesTable_<SimTK::Vec5></td>
<td>DataType=Vec5</td></tr>
<tr><td>TimeSeriesTable_<SimTK::Vec6></td>
<td>DataType=Vec6</td></tr>
<tr><td>TimeSeriesTable_<SimTK::Vec7></td>
<td>DataType=Vec7</td></tr>
<tr><td>TimeSeriesTable_<SimTK::Vec8></td>
<td>DataType=Vec8</td></tr>
<tr><td>TimeSeriesTable_<SimTK::Vec9></td>
<td>DataType=Vec9</td></tr>
<tr><td>TimeSeriesTable_<SimTK::Vec<10>></td>
<td>DataType=Vec10</td></tr>
<tr><td>TimeSeriesTable_<SimTK::Vec<11>></td>
<td>DataType=Vec11</td></tr>
<tr><td>TimeSeriesTable_<SimTK::Vec<12>></td>
<td>DataType=Vec12</td></tr>
<tr><td>TimeSeriesTable_<SimTK::UnitVec3></td>
<td>DataType=UnitVec3</td></tr>
<tr><td>TimeSeriesTable_<SimTK::Quaternion></td>
<td>DataType=Quaternion</td></tr>
<tr><td>TimeSeriesTable_<SimTK::SpatialVec></td>
<td>DataType=SpatialVec</td></tr>
</table>
Files contain metadata (in form of "DataType=<something>" as shown above) 
indicating the type of the table they contain.
*/
template<typename T>
class STOFileAdapter_ : public DelimFileAdapter<T> {
public:
    STOFileAdapter_();
    STOFileAdapter_(const STOFileAdapter_&)            = default;
    STOFileAdapter_(STOFileAdapter_&&)                 = default;
    STOFileAdapter_& operator=(const STOFileAdapter_&) = default;
    STOFileAdapter_& operator=(STOFileAdapter_&&)      = default;
    ~STOFileAdapter_()                                 = default;

    STOFileAdapter_* clone() const override;

    /** Write a STO file.                                                     */
    static
    void write(const TimeSeriesTable_<T>& table, const std::string& fileName);
};

template<typename T>
STOFileAdapter_<T>::STOFileAdapter_() :
    DelimFileAdapter<T>("\t", // delimiter for read between elements
                        "\t",  // delimiter for write between elements
                        ",",   // delim for reading components(within element)
                        ","    // delim for writing components(within element)
                        ) {}

template<typename T>
STOFileAdapter_<T>*
STOFileAdapter_<T>::clone() const {
    return new STOFileAdapter_{*this};
}

template<typename T>
void 
STOFileAdapter_<T>::write(const TimeSeriesTable_<T>& table, 
                         const std::string& fileName) {
    DataAdapter::InputTables tables{};
    tables.emplace(DelimFileAdapter<T>::tableString(), &table);
    STOFileAdapter_{}.extendWrite(tables, fileName);
}

std::shared_ptr<DataAdapter> 
createSTOFileAdapterForReading(const std::string& fileName);

std::shared_ptr<DataAdapter>
createSTOFileAdapterForWriting(const AbstractDataTable& table);

typedef STOFileAdapter_<double> STOFileAdapter;
typedef STOFileAdapter_<SimTK::Vec3> STOFileAdapterVec3;
typedef STOFileAdapter_<SimTK::Quaternion> STOFileAdapterQuaternion;
}

#endif // OPENSIM_STO_FILE_ADAPTER_H_
