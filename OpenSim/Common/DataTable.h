/* -------------------------------------------------------------------------- *
 *                            OpenSim:  DataTable.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Authors:                                                                   *
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

#ifndef OPENSIM_DATA_TABLE_H_
#define OPENSIM_DATA_TABLE_H_

/** \file
This file defines the  DataTable_ class, which is used by OpenSim to provide an 
in-memory container for data access and manipulation.                         */

#include "AbstractDataTable.h"
#include "FileAdapter.h"
#include "SimTKcommon/internal/BigMatrix.h"

#include <iomanip>
#include <numeric>

namespace OpenSim {

/** DataTable_ is an in-memory storage container for data with support for 
holding metadata (using the base class AbstractDataTable). Data contains an 
independent column and a set of dependent columns. The type of the independent 
column can be configured using ETX (template param). The type of the dependent 
columns, which together form a matrix, can be configured using ETY (template 
param). Independent and dependent columns can contain metadata. DataTable_ as a 
whole can contain metadata.

\tparam ETX Type of each element of the column holding independent data.
\tparam ETY Type of each element of the underlying matrix holding dependent 
            data.                                                             */
template<typename ETX = double, typename ETY = SimTK::Real>
class DataTable_ : public AbstractDataTable {
    static_assert(!std::is_reference<ETY>::value,
                  "Template argument ETY cannot be a 'reference'.");
    static_assert(!std::is_pointer<ETY>::value,
                  "Template argument ETY cannot be a 'pointer'.");
    static_assert(!std::is_const<ETY>::value && !std::is_volatile<ETY>::value,
                  "Template argument ETY cannot be 'const' or 'volatile'.");

public:
    /** Type of each row of matrix holding dependent data.                    */
    typedef SimTK::RowVector_<ETY>     RowVector;
    /** (Read only view) Type of each row of matrix.                          */
    typedef SimTK::RowVectorView_<ETY> RowVectorView;
    /** Type of each column of matrix holding dependent data.                 */
    typedef SimTK::Vector_<ETY>        Vector;
    /** Type of each column of matrix holding dependent data.                 */
    typedef SimTK::VectorView_<ETY>    VectorView;
    /** Type of the matrix holding the dependent data.                        */
    typedef SimTK::Matrix_<ETY>        Matrix;
    /** (Read only view) Type of the matrix  holding the dependent data.      */
    typedef SimTK::MatrixView_<ETY>    MatrixView;

    DataTable_()                             = default;
    DataTable_(const DataTable_&)            = default;
    DataTable_(DataTable_&&)                 = default;
    DataTable_& operator=(const DataTable_&) = default;
    DataTable_& operator=(DataTable_&&)      = default;
    ~DataTable_()                            = default;

    std::shared_ptr<AbstractDataTable> clone() const override {
        return std::shared_ptr<AbstractDataTable>{new DataTable_{*this}};
    }

    /** Construct DataTable_ from a file.                                     

    \param filename Name of the file. File should contain only one table. For
                    example, trc, csv & sto files contain one table whereas a 
                    c3d file can contain more than.
    \param tablename Name of the table in file to construct this DataTable_
                     from. For example, a c3d file contains tables named
                     'markers' and 'forces'.

    \throws InvalidArgument If the input file contains more than one table and
                            tablename was not specified.   
    \throws InvalidArgument If the input file contains a table that is not of
                            this DataTable_ type.                             */
    DataTable_(const std::string& filename,
               const std::string& tablename) {
        auto absTables = FileAdapter::readFile(filename);

        OPENSIM_THROW_IF(absTables.size() > 1 && tablename.empty(),
                         InvalidArgument,
                         "File '" + filename + 
                         "' contains more than one table and tablename not"
                         " specified.");

        AbstractDataTable* absTable{};
        if(tablename.empty()) {
            absTable = (absTables.cbegin()->second).get();
        } else {
            try {
                absTable = absTables.at(tablename).get();
            } catch (const std::out_of_range&) {
                OPENSIM_THROW(InvalidArgument,
                              "File '" + filename + "' contains no table named "
                              "'" + tablename + "'.");
            }
        }
        auto table = dynamic_cast<DataTable_*>(absTable);
        OPENSIM_THROW_IF(table == nullptr,
                         IncorrectTableType,
                         "DataTable cannot be created from file '" + filename +
                         "'. Type mismatch.");

        *this = std::move(*table);
    }

    /** Construct DataTable_<double, double> from 
    DataTable_<double, ThatETY> where ThatETY can be SimTK::Vec<X>. Each column
    of the other table is split into multiple columns of this table. For example
    , DataTable_<double, Vec3> with 3 columns and 4 rows will construct
    DataTable<double, double> of 9 columns and 4 rows where each component of
    SimTK::Vec3 ends up in one column. Column labels of the resulting DataTable
    will use column labels of source table appended with suffixes provided.
    This constructor only makes sense for DataTable_<double, double>.

    \tparam ThatETY Datatype of the matrix underlying the given DataTable.

    \param that DataTable to copy-construct this table from. This table can be
                of different SimTK::Vec<X> types, for example DataTable_<double,
                Vec<3>>, DataTable_<double, Quaternion>, DataTable_<double, 
                Vec6> etc.
    \param suffixes Suffixes to be used for column-labels of individual 
                    components/columns in this table when splitting columns of 
                    'that' table. For example a column labeled 'marker' from 
                    DataTable_<double, Vec3> will be split into 3 columns named
                    \code
                    std::string{'marker' + suffixes[0]},
                    std::string{'marker' + suffixes[1]},
                    std::string{'marker' + suffixes[2]}
                    \endcode

    \throws InvalidArgument If 'that' DataTable has no column-labels.
    \throws InvalidArgument If 'that' DataTable has zero number of rows/columns.
    \throws InvalidArgument If 'suffixes' does not contain same number of
                            elements as that.numComponentsPerElement().       */
    template<typename ThatETY>
    DataTable_(const DataTable_<double, ThatETY>& that,
               const std::vector<std::string>& suffixes) :
    AbstractDataTable{that} {
        static_assert(std::is_same<ETY, double>::value,
                      "This constructor can only be used to construct "
                      "DataTable_<double, double>.");
        static_assert(!std::is_same<ThatETY, double>::value,
                      "This constructor cannot be used to construct from "
                      "DataTable_<double, double>. Use the copy constructor "
                      "instead.");

        std::vector<std::string> thatLabels{};
        OPENSIM_THROW_IF(!that.hasColumnLabels(),
                         InvalidArgument,
                         "DataTable 'that' has no column labels.");
        OPENSIM_THROW_IF(that.getNumRows() == 0 || that.getNumColumns() == 0,
                         InvalidArgument,
                         "DataTable 'that' has zero rows/columns.");
        OPENSIM_THROW_IF(!suffixes.empty() &&
                         suffixes.size() != that.numComponentsPerElement(),
                         InvalidArgument,
                         "'suffixes' must contain same number of elements as "
                         "number of components per element of DataTable 'that'."
                         "See documentation for numComponentsPerElement().");

        // If the dependents metadata is of std::string type,
        // replicate it to match the new number of columns. If not of
        // std::string type, drop the metadata because type information is
        // required to interpret them.
        // Column-labels will be handled separately as they need suffixing.
        for(const auto& key : _dependentsMetaData.getKeys()) {
            if(key == "labels")
                continue;

            auto absValueArray = &_dependentsMetaData.updValueArrayForKey(key);
            ValueArray<std::string>* valueArray{};
            try {
                valueArray =
                    dynamic_cast<ValueArray<std::string>*>(absValueArray);
            } catch (const std::bad_cast&) {
                _dependentsMetaData.removeValueArrayForKey(key);
                continue;
            }
            auto& values = valueArray->upd();
            std::vector<SimTK::Value<std::string>> newValues{};
            for(const auto& value : values)
                for(auto i = 0u; i < that.numComponentsPerElement(); ++i)
                    newValues.push_back(value);
            values = std::move(newValues);
        }

        std::vector<std::string> thisLabels{};
        thisLabels.reserve(that.getNumColumns() *
                           that.numComponentsPerElement());
        for(const auto& label : that.getColumnLabels()) {
            if(suffixes.empty()) {
                for(unsigned i = 1; i <= that.numComponentsPerElement(); ++i)
                    thisLabels.push_back(label + "_" + std::to_string(i));
            } else {
                for(const auto& suffix : suffixes)
                    thisLabels.push_back(label + suffix);
            }
        }
        // This calls validateDependentsMetadata, so no need for explicit call.
        setColumnLabels(thisLabels);

        for(unsigned r = 0; r < that.getNumRows(); ++r) {
            const auto& thatInd = that.getIndependentColumn().at(r);
            const auto& thatRow = that.getRowAtIndex(r);
            std::vector<ETY> thisRow{};
            for(unsigned c = 0; c < that.getNumColumns(); ++c)
                splitElementAndPushBack(thisRow, thatRow[c]);
            appendRow(thatInd, thisRow);
        }
    }

    /** Construct this DataTable from a DataTable_<double, double>. This is the
    opposite operation of flatten(). Multiple consecutive columns of the given
    DataTable will be 'packed' together to form columns of this DataTable. For
    example, if this DataTable is of type DataTable_<double, Vec3>, then every
    3 consecutive columns of the given DataTable will form one column of this
    DataTable. The column labels of this table will be formed by stripping out
    the suffixes from the column labels of the given DataTable. For the same 
    example above, if columns labels of the given DataTable are -- 
    "col0.x", "col0.y", "col0.x", "col1.x", "col1.y", "col1.x" -- the column 
    labels of this DataTable will be -- "col0", "col1" -- where suffixes are
    stripped out. This constructor will try to guess the suffixes used. If 
    unable to do so, it will throw an exception. Suffixes used can also be 
    specified as arguments. 
    This constructor only makes sense for DataTable_<double, SimTKType> where
    SimTKType is not 'double'. SimTKType can be for example, SimTK::Vec3, 
    SimTK::Vec6, SimTK::Quaternion, SimTK::SpatialVec etc.

    \param that DataTable to copy-construct this DataTable from.
    \param suffixes Suffixes used in the input DataTable to distinguish 
                    individual components. For example, if column labels are -- 
                    "force.x", "force.y", "force.z" -- suffixes will be -- ".x",
                    ".y", ".z".

    \throws InvalidArgument If 'that' DataTable has no column-labels.
    \throws InvalidArgument If 'that' DataTable has no rows/columns.
    \throws InvalidArgument If 'suffixes' does not contain same number of 
                            elements as this->numComponentsPerElement().
    \throws InvalidArgument If number of columns in 'that' DataTable is not a
                            multiple of this->numComponentsPerElement().
    \throws InvalidArgument If suffixes cannot be extracted from column-labels 
                            of 'that' DataTable.                              */
    explicit DataTable_(const DataTable_<double, double>& that,
                        const std::vector<std::string>& suffixes) :
        AbstractDataTable{that} {
        static_assert(!std::is_same<ETY, double>::value,
                      "This constructor cannot be used to construct "
                      "DataTable_<double, double>. Maybe use the copy "
                      "constructor instead.");

        OPENSIM_THROW_IF(!that.hasColumnLabels(),
                         InvalidArgument,
                         "DataTable 'that' has no column labels.");
        OPENSIM_THROW_IF(that.getNumRows() == 0 || that.getNumColumns() == 0,
                         InvalidArgument,
                         "DataTable 'that' has zero rows/columns.");
        OPENSIM_THROW_IF(!suffixes.empty() &&
                         suffixes.size() != numComponentsPerElement(),
                         InvalidArgument,
                         "'suffixes' must contain same number of elements as "
                         "number of components per element of 'this' DataTable."
                         " See documentation for numComponentsPerElement().");
        OPENSIM_THROW_IF(std::lldiv(that.getNumColumns(),
                                    numComponentsPerElement()).rem != 0,
                         InvalidArgument,
                         "Input DataTable must contain " +
                         std::to_string(numComponentsPerElement()) + "x "
                         "number of columns.");

        const auto& thatLabels = that.getColumnLabels();
        for(unsigned i = 0; i < thatLabels.size(); ++i)
            OPENSIM_THROW_IF(thatLabels[i].length() < 2,
                             InvalidArgument,
                             "Column label at index " + std::to_string(i) +
                             " is too short to have a suffix.");

        // Guess suffixes from columns labels of that table.
        std::vector<std::string> suffs{suffixes};
        if(suffs.empty()) {
            for(unsigned i = 0; i < numComponentsPerElement(); ++i) {
                std::string suff{thatLabels[i][thatLabels[i].size() - 1]};
                char nonSuffChar{thatLabels[i][thatLabels[i].size() - 2]};
                bool foundNonSuffChar{false};
                while(!foundNonSuffChar) {
                    for(unsigned c = i;
                        c < thatLabels.size();
                        c += numComponentsPerElement()) {
                        try {
                            if(thatLabels[c].at(thatLabels[c].size() - 1 -
                                                suff.length()) != nonSuffChar) {
                                foundNonSuffChar = true;
                                break;
                            }
                        } catch(const std::out_of_range&) {
                            OPENSIM_THROW(InvalidArgument,
                                          "Cannot guess the suffix from column"
                                          " label at index " +
                                          std::to_string(c));
                        }
                    }
                    if(!foundNonSuffChar) {
                        suff.insert(suff.begin(), nonSuffChar);
                        try {
                            nonSuffChar =
                                thatLabels[i].at(thatLabels[i].size() - 1 -
                                                 suff.length());
                        } catch(const std::out_of_range&) {
                            OPENSIM_THROW(InvalidArgument,
                                          "Cannot guess the suffix from column"
                                          " label at index " +
                                          std::to_string(i));
                        }
                    }
                }
                suffs.push_back(suff);
            }
        }

        // Form column labels for this table from that table.
        std::vector<std::string> thisLabels{};
        thisLabels.reserve(that.getNumColumns() / numComponentsPerElement());
        for(unsigned c = 0; c < thatLabels.size(); ) {
            std::string thisLabel{};
            for(unsigned i = 0; i < numComponentsPerElement(); ++i, ++c) {
                const auto& thatLabel = thatLabels[c];
                OPENSIM_THROW_IF(thatLabel.compare(thatLabel.length() -
                                                   suffs[i].length(),
                                                   suffs[i].length(),
                                                   suffs[i]) != 0,
                                 InvalidArgument,
                                 "Suffix not found in column label '" +
                                 thatLabel + "'. Expected suffix '" +
                                 suffs[i] + "'.");

                if(i == 0) {
                    thisLabel = thatLabel.substr(0,
                                                 thatLabel.length() -
                                                 suffs[i].length());
                    thisLabels.push_back(thisLabel);
                } else {
                    OPENSIM_THROW_IF(thisLabel !=
                                     thatLabel.substr(0,
                                                      thatLabel.length() -
                                                      suffs[i].length()),
                                     InvalidArgument,
                                     "Unexpected column-label '" + thatLabel +
                                     "'. Expected: '" + thisLabel + suffs[i] +
                                     "'.");
                }
            }
        }
        setColumnLabels(thisLabels);

        // Form rows for this table from that table.
        for(unsigned r = 0; r < that.getNumRows(); ++r) {
            const auto& thatInd = that.getIndependentColumn().at(r);
            auto thatRow = that.getRowAtIndex(r).getAsRowVector();
            std::vector<ETY> thisRow{};
            for(unsigned c = 0;
                c < that.getNumColumns();
                c += numComponentsPerElement()) {
                thisRow.push_back(makeElement(thatRow.begin() + c,
                                              thatRow.end()));
            }
            appendRow(thatInd, thisRow);
        }
    }

    /** Construct DataTable_<double, double> from 
    DataTable_<double, ThatETY> where ThatETY can be SimTK::Vec<X>. Each column
    of the other table is split into multiple columns of this table. For example
    , DataTable_<double, Vec3> with 3 columns and 4 rows will construct
    DataTable<double, double> of 9 columns and 4 rows where each component of
    SimTK::Vec3 ends up in one column. Column labels of the resulting DataTable
    will use column labels of source table appended with suffixes of form "_1",
    "_2", "_3" and so on.

    \tparam ThatETY Datatype of the matrix underlying the given DataTable.

    \param that DataTable to copy-construct this table from. This table can be
                of for example DataTable_<double, Quaternion>, 
                DataTable_<double, Vec6>.

    \throws InvalidArgument If 'that' DataTable has no column-labels.
    \throws InvalidArgument If 'that' DataTable has zero number of rows/columns.
    \throws InvalidArgument If 'suffixes' does not contain same number of
                            elements as that.numComponentsPerElement().       */
    template<typename ThatETY>
    explicit DataTable_(const DataTable_<double, ThatETY>& that) :
    DataTable_(that, std::vector<std::string>{}) {
        // No operation.
    }

    /** Copy assign a DataTable_<double, double> from 
    DataTable_<double, ThatETY> where ThatETY can be SimTK::Vec<X>. Each column
    of the other table is split into multiple columns of this table. For example
    , DataTable_<double, Vec3> with 3 columns and 4 rows will construct
    DataTable<double, double> of 9 columns and 4 rows where each component of
    SimTK::Vec3 ends up in one column. Column labels of the resulting DataTable
    will use column labels of source table appended with suffixes of form "_1",
    "_2", "_3" and so on.

    \tparam ThatETY Datatype of the matrix underlying the given DataTable.

    \param that DataTable to copy assign from. This table can be
                of for example DataTable_<double, Quaternion>, 
                DataTable_<double, Vec6>.

    \throws InvalidArgument If 'that' DataTable has no column-labels.
    \throws InvalidArgument If 'that' DataTable has zero number of rows/columns.
    \throws InvalidArgument If 'suffixes' does not contain same number of
                            elements as that.numComponentsPerElement().       */
    template<typename ThatETY>
    DataTable_& operator=(const DataTable_<double, ThatETY>& that) {
        return operator=(DataTable_{that});
    }

    /** Flatten the columns of this table to create a 
    DataTable_<double, double>. Each column will be split into its 
    constituent components. For example, each column of a 
    DataTable_<double, Vec3> will be split into 3 columns. The column-labels of
    the resulting columns will be suffixed "_1", "_2", "_3" and so on. See
    documentation for constructor DataTable_::DataTable_().                   */
    DataTable_<double, double> flatten() const {
        return DataTable_<double, double>{*this};
    }

    /** Flatten the columns of this table to create a 
    DataTable_<double, double>. Each column will be split into its 
    constituent components. For example, each column of a 
    DataTable_<double, Vec3> will be split into 3 columns. The column-labels of
    the resulting columns will be appended with 'suffixes' provided. See
    documentation for constructor DataTable_::DataTable_().                   */
    DataTable_<double, double>
    flatten(const std::vector<std::string>& suffixes) const {
        return DataTable_<double, double>{*this, suffixes};
    }

    /** Pack the columns of this table to create a DataTable_<double, ThatETY>,
    where 'ThatETY' is the template parameter which can be SimTK::Vec3, 
    SimTK::UnitVec3, SimTK::Quaternion, SimTK::SpatialVec and so on. Multiple
    consecutive columns of this table will be packed into one column of the 
    resulting table. For example while creating a DataTable_<double, Quaternion>
    , every group of 4 consecutive columns of this table will form one column 
    of the resulting table. The column-labels of the resulting table will be
    formed by stripping the suffixes in the column-labels of this table.
    This function will attempt to guess the suffixes of column-labels. See
    documentation for constructor DataTable_::DataTable_().                   */
    template<typename ThatETY>
    DataTable_<double, ThatETY> pack() const {
        return DataTable_<double, ThatETY>{*this};
    }

    /** Pack the columns of this table to create a DataTable_<double, ThatETY>,
    where 'ThatETY' is the template parameter which can be SimTK::Vec3, 
    SimTK::UnitVec3, SimTK::Quaternion, SimTK::SpatialVec and so on. Multiple
    consecutive columns of this table will be packed into one column of the 
    resulting table. For example while creating a DataTable_<double, Quaternion>
    , every group of 4 consecutive columns of this table will form one column 
    of the resulting table. The column-labels of the resulting table will be
    formed by stripping the suffixes in the column-labels of this table. See
    documentation for constructor DataTable_::DataTable_().                   */
    template<typename ThatETY>
    DataTable_<double, ThatETY>
    pack(const std::vector<std::string>& suffixes) const {
        return DataTable_<double, ThatETY>{*this, suffixes};
    }

    /** Retrieve the number of components each element (of type ETY) of the 
    table is made of. Some examples:

    Table Type                    | Element Type | Num of Components
    ------------------------------|--------------|------------------
    DataTable<double, double>     | double       | 1
    DataTable<double, Vec3>       | Vec3         | 3
    DataTable<double, Quaternion> | Quaternion   | 4                          */
    unsigned numComponentsPerElement() const override {
        return numComponentsPerElement_impl(ETY{});
    }

    /// @name Row accessors/mutators.
    /// Following get/upd functions operate on matrix and not the independent
    /// column.
    /// The function appendRow() is pretty flexible and it is possible to 
    /// append a row with any sequence of elements. Following are some examples:
    /// \code
    /// // Easiest way to append a row is to provide the list of elements 
    /// // directly to appendRow.
    /// // For a table with elements of type double, this could look like below.
    /// table.appendRow(0.1, // Independent column.
    ///                 {0.3, 0.4, 0.5, 0.6}); // 4 elements of type double.
    /// // For a table with elements of type SimTK::Vec3, this could like below.
    /// table.appendRow(0.1, // Independent column.
    ///                 {{0.31, 0.32, 0.33},
    ///                  {0.41, 0.42, 0.43},
    ///                  {0.51, 0.52, 0.53},
    ///                  {0.61, 0.62, 0.63}}); // 4 elements of SimTK::Vec3.
    /// \endcode
    /// \code
    /// // It is possible to append a sequence container like std::vector or 
    /// // std::list by providing it directly to appendRow.
    /// // For a table with elements of type double, this could look like below.
    /// std::vector<double> row{0.3, 0.4, 0.5, 0.6};
    /// table.appendRow(0.1, row);
    /// // For a table with elements of type SimTK::Vec3, this could look like
    /// // below.
    /// std::vector<SimTK::Vec3> row{{0.31, 0.32, 0.33},
    ///                              {0.41, 0.42, 0.43},
    ///                              {0.51, 0.52, 0.53},   // 4 elements of
    ///                              {0.61, 0.62, 0.63}}); //  SimTK::Vec3.
    /// table.appendRow(0.1, row);
    /// \endcode
    /// \code
    /// // A SimTK::RowVector can be provided to appendRow as well.
    /// // For a table with elements of type double, this could look like below.
    /// SimTK::RowVector row{0.3, 0.4, 0.5, 0.6};
    /// table.appendRow(0.1, row);
    /// // For a table with elements of type SimTK::Vec3, this could look like
    /// // below.
    /// SimTK::RowVector_<SimTK::Vec3> row{{0.31, 0.32, 0.33},
    ///                                    {0.41, 0.42, 0.43},
    ///                                    {0.51, 0.52, 0.53},  // 4 elements of
    ///                                    {0.61, 0.62, 0.63}}); // SimTK::Vec3.
    /// table.appendRow(0.1, row);
    /// \endcode
    /// \code
    /// // It is possible to be use a pair of iterators to append a row as well.
    /// // This could arise in situations where you might want to append a row
    /// // using a subset of elements in a sequence.
    /// // For a table with elements of type double, this could look like below.
    /// std::vector<double> row{0.3, 0.4, 0.5, 0.6, 0.7, 0.8};
    /// table.appendRow(0.1, // Independent column.
    ///                 row.begin() + 1, // Start from second element (0.4).
    ///                 row.end() - 1);  // End at last but one (0.7).
    /// // For a table with elements of type SimTK::Vec3, this could look like
    /// // below.
    /// std::vector<SimTK::Vec3> row{{0.31, 0.32, 0.33},
    ///                              {0.41, 0.42, 0.43},
    ///                              {0.51, 0.52, 0.53},   
    ///                              {0.61, 0.62, 0.63},
    ///                              {0.71, 0.72, 0.73},   // 6 elements of
    ///                              {0.81, 0.82, 0.83}}); //  SimTK::Vec3.
    /// table.appendRow(0.1, // Independent column.
    ///                 row.begin() + 1, // Start from second element.
    ///                 row.end() - 1); // End at last but one.
    /// \endcode
    /// @{

    /** Append row to the DataTable_.

    \param indRow Entry for the independent column corresponding to the row to
                  be appended.
    \param container Sequence container holding the elements of the row to be
                     appended.

    \throws IncorrectNumColumns If the row added is invalid. Validity of the 
    row added is decided by the derived class.                                */
    template<typename Container>
    void appendRow(const ETX& indRow, const Container& container) {
        using Value = decltype(*(container.begin()));
        using RmrefValue = typename std::remove_reference<Value>::type;
        using RmcvRmrefValue = typename std::remove_cv<RmrefValue>::type;
        static_assert(std::is_same<ETY, RmcvRmrefValue>::value,
                      "The 'container' specified does not provide an iterator "
                      "which when dereferenced provides elements that "
                      "are of same type as elements of this table.");

        appendRow(indRow, container.begin(), container.end());
    }

    /** Append row to the DataTable_.

    \param indRow Entry for the independent column corresponding to the row to
                  be appended.
    \param container std::initializer_list containing elements of the row to be
                     appended.

    \throws IncorrectNumColumns If the row added is invalid. Validity of the 
    row added is decided by the derived class.                                */
    void appendRow(const ETX& indRow, 
                   const std::initializer_list<ETY>& container) {
        appendRow(indRow, container.begin(), container.end());
    }

    /** Append row to the DataTable_.

    \param indRow Entry for the independent column corresponding to the row to
                  be appended.
    \param begin Iterator representing the beginning of the row to be appended.
    \param end Iterator representing one past the end of the row to be appended.

    \throws IncorrectNumColumns If the row added is invalid. Validity of the 
    row added is decided by the derived class.                                */
    template<typename RowIter>
    void appendRow(const ETX& indRow, RowIter begin, RowIter end) {
        using Value = decltype(*begin);
        using RmrefValue = typename std::remove_reference<Value>::type;
        using RmcvRmrefValue = typename std::remove_cv<RmrefValue>::type;
        static_assert(std::is_same<ETY, RmcvRmrefValue>::value,
                      "The iterator 'begin' provided does not provide elements"
                      " that are of same type as elements of this table.");

        RowVector row{static_cast<int>(std::distance(begin, end))};
        int ind{0};
        for(auto it = begin; it != end; ++it)
            row[ind++] = *it;

        appendRow(indRow, row);
    }

    /** Append row to the DataTable_.                                         

    \throws IncorrectNumColumns If the row added is invalid. Validity of the 
    row added is decided by the derived class.                                */
    void appendRow(const ETX& indRow, const RowVector& depRow) {
        appendRow(indRow, depRow.getAsRowVectorView());
    }

    /** Append row to the DataTable_.                                         

    \throws IncorrectNumColumns If the row added is invalid. Validity of the 
    row added is decided by the derived class.                                */
    void appendRow(const ETX& indRow, const RowVectorView& depRow) {
        validateRow(_indData.size(), indRow, depRow);

        _indData.push_back(indRow);

        if(_depData.nrow() == 0 || _depData.ncol() == 0) {
            try {
                auto& labels = 
                    _dependentsMetaData.getValueArrayForKey("labels");
                OPENSIM_THROW_IF(static_cast<unsigned>(depRow.ncol()) != 
                                 labels.size(),
                                 IncorrectNumColumns, 
                                 labels.size(), 
                                 static_cast<size_t>(depRow.ncol()));
            } catch(KeyNotFound&) {
                // No "labels". So no operation.
            }
            _depData.resize(1, depRow.size());
        } else
            _depData.resizeKeep(_depData.nrow() + 1, _depData.ncol());
            
        _depData.updRow(_depData.nrow() - 1) = depRow;
    }

    /** Get row at index.                                                     

    \throws RowIndexOutOfRange If index is out of range.                      */
    const RowVectorView getRowAtIndex(size_t index) const {
        OPENSIM_THROW_IF(isRowIndexOutOfRange(index),
                         RowIndexOutOfRange, 
                         index, 0, static_cast<unsigned>(_indData.size() - 1));

        return _depData.row(static_cast<int>(index));
    }

    /** Get row corresponding to the given entry in the independent column. This
    function searches the independent column for exact equality, which may not
    be appropriate if `ETX` is of type `double`. See 
    TimeSeriesTable_::getNearestRow().

    \throws KeyNotFound If the independent column has no entry with given
                        value.                                                */
    const RowVectorView getRow(const ETX& ind) const {
        auto iter = std::find(_indData.cbegin(), _indData.cend(), ind);

        OPENSIM_THROW_IF(iter == _indData.cend(),
                         KeyNotFound, std::to_string(ind));

        return _depData.row((int)std::distance(_indData.cbegin(), iter));
    }

    /** Update row at index.                                                  

    \throws RowIndexOutOfRange If the index is out of range.                  */
    RowVectorView updRowAtIndex(size_t index) {
        OPENSIM_THROW_IF(isRowIndexOutOfRange(index),
                         RowIndexOutOfRange, 
                         index, 0, static_cast<unsigned>(_indData.size() - 1));

        return _depData.updRow((int)index);
    }

    /** Update row corresponding to the given entry in the independent column.
    This function searches the independent column for exact equality, which may 
    not be appropriate if `ETX` is of type `double`. See 
    TimeSeriesTable_::updNearestRow().

    \throws KeyNotFound If the independent column has no entry with given
                        value.                                                */
    RowVectorView updRow(const ETX& ind) {
        auto iter = std::find(_indData.cbegin(), _indData.cend(), ind);

        OPENSIM_THROW_IF(iter == _indData.cend(),
                         KeyNotFound, std::to_string(ind));

        return _depData.updRow((int)std::distance(_indData.cbegin(), iter));
    }

    /** Set row at index. Equivalent to
    ```
    updRowAtIndex(index) = depRow;
    ```

    \throws RowIndexOutOfRange If the index is out of range.                  */
    void setRowAtIndex(size_t index, const RowVectorView& depRow) {
        updRowAtIndex(index) = depRow;
    }

    /** Set row at index. Equivalent to
    ```
    updRowAtIndex(index) = depRow;
    ```

    \throws RowIndexOutOfRange If the index is out of range.                  */
    void setRowAtIndex(size_t index, const RowVector& depRow) {
        updRowAtIndex(index) = depRow;
    }

    /** Set row corresponding to the given entry in the independent column.
    This function searches the independent column for exact equality, which may 
    not be appropriate if `ETX` is of type `double`. See 
    TimeSeriesTable_::updNearestRow().
    Equivalent to
    ```
    updRow(ind) = depRow;
    ```

    \throws KeyNotFound If the independent column has no entry with given
                        value.                                                */
    void setRow(const ETX& ind, const RowVectorView& depRow) {
        updRow(ind) = depRow;
    }

    /** Set row corresponding to the given entry in the independent column.
    This function searches the independent column for exact equality, which may 
    not be appropriate if `ETX` is of type `double`. See 
    TimeSeriesTable_::updNearestRow().
    Equivalent to
    ```
    updRow(ind) = depRow;
    ```

    \throws KeyNotFound If the independent column has no entry with given
                        value.                                                */
    void setRow(const ETX& ind, const RowVector& depRow) {
        updRow(ind) = depRow;
    }

    /** Remove row at index.

    \throws RowIndexOutOfRange If the index is out of range.                  */
    void removeRowAtIndex(size_t index) {
        OPENSIM_THROW_IF(isRowIndexOutOfRange(index),
                         RowIndexOutOfRange, 
                         index, 0, static_cast<unsigned>(_indData.size() - 1));

        if(index < getNumRows() - 1)
            for(size_t r = index; r < getNumRows() - 1; ++r)
                _depData.updRow((int)index) = _depData.row((int)(index + 1));
        
        _depData.resizeKeep(_depData.nrow() - 1, _depData.ncol());
        _indData.erase(_indData.begin() + index);
    }

    /** Remove row corresponding to the given entry in the independent column.

    \throws KeyNotFound If the independent column has no entry with the given
                        value.                                                */
    void removeRow(const ETX& ind) {
        auto iter = std::find(_indData.cbegin(), _indData.cend(), ind);

        OPENSIM_THROW_IF(iter == _indData.cend(),
                         KeyNotFound, std::to_string(ind));

        return removeRowAtIndex((int)std::distance(_indData.cbegin(), iter));
    }

    /// @} End of Row accessors/mutators.

    /// @name Dependent and Independent column accessors/mutators.
    /// @{

    /** Get independent column.                                               */
    const std::vector<ETX>& getIndependentColumn() const {
        return _indData;
    }

    /** Append column to the DataTable_ using a sequence container.
    \code
    std::vector<double> col{1, 2, 3, 4};
    table.appendColumn("new-column", col);
    \endcode

    \param columnLabel Label of the column to be added. Must not be same as the
                       label of an existing column.
    \param container Sequence container holding the elements of the column to be
                     appended.
    \throws InvalidCall If DataTable_ contains no rows at the time of this call.
    \throws InvalidArgument If columnLabel specified already exists in the
                            DataTable_.
    \throws InvalidColumn If the input column contains incorrect number of 
                          rows.                                               */
    template<typename Container>
    void appendColumn(const std::string& columnLabel,
                      const Container& container) {
        using Value = decltype(*(container.begin()));
        using RmrefValue = typename std::remove_reference<Value>::type;
        using RmcvRmrefValue = typename std::remove_cv<RmrefValue>::type;
        static_assert(std::is_same<ETY, RmcvRmrefValue>::value,
                      "The 'container' specified does not provide an iterator "
                      "which when dereferenced provides elements that "
                      "are of same type as elements of this table.");

        appendColumn(columnLabel, container.begin(), container.end());
    }

    /** Append column to the DataTable_ using an initializer list.
    \code
    table.appendColumn("new-column", {1, 2, 3, 4});
    \endcode

    \param columnLabel Label of the column to be added. Must not be same as the
                       label of an existing column.
    \param container Sequence container holding the elements of the column to be
                     appended.
    \throws InvalidCall If DataTable_ contains no rows at the time of this call.
    \throws InvalidArgument If columnLabel specified already exists in the
                            DataTable_.
    \throws InvalidColumn If the input column contains incorrect number of 
                          rows.                                               */
    void appendColumn(const std::string& columnLabel,
                      const std::initializer_list<ETY>& container) {
        appendColumn(columnLabel, container.begin(), container.end());
    }

    /** Append column to the DataTable_ using an iterator pair.
    \code
    std::vector<double> col{};
    // ......
    // Fill up 'col'.
    // ......
    table.append("new-column", col.begin(), col.end());
    \endcode

    \param columnLabel Label of the column to be added. Must not be same as the
                       label of an existing column.
    \param begin Iterator referring to the beginning of the range.
    \param end Iterator referring to the end of the range.

    \throws InvalidCall If DataTable_ contains no rows at the time of this call.
    \throws InvalidArgument If columnLabel specified already exists in the
                            DataTable_.
    \throws InvalidColumn If the input column contains incorrect number of 
                          rows.                                               */
    template<typename ColIter>
    void appendColumn(const std::string& columnLabel,
                      ColIter begin, ColIter end) {
        using Value = decltype(*begin);
        using RmrefValue = typename std::remove_reference<Value>::type;
        using RmcvRmrefValue = typename std::remove_cv<RmrefValue>::type;
        static_assert(std::is_same<ETY, RmcvRmrefValue>::value,
                      "The iterator 'begin' does not provide elements that are "
                      "of same type as elements of this table.");

        Vector col{static_cast<int>(std::distance(begin, end))};
        int ind{0};
        for(auto it = begin; it != end; ++it)
            col[ind++] = *it;

        appendColumn(columnLabel, col);
    }

    /** Append column to the DataTable_ using a SimTK::Vector.

    \param columnLabel Label of the column to be added. Must not be same as the
                       label of an existing column.
    \param depCol Column vector to be appended to the table.

    \throws InvalidCall If DataTable_ contains no rows at the time of this call.
    \throws InvalidArgument If columnLabel specified already exists in the
                            DataTable_.
    \throws InvalidColumn If the input column contains incorrect number of 
                          rows.                                               */
    void appendColumn(const std::string& columnLabel,
                      const Vector& depCol) {
        appendColumn(columnLabel, depCol.getAsVectorView());
    }

    /** Append column to the DataTable_ using a SimTK::VectorView.

    \param columnLabel Label of the column to be added. Must not be same as the
                       label of an existing column.
    \param depCol Column vector to be appended to the table.

    \throws InvalidCall If DataTable_ contains no rows at the time of this call.
    \throws InvalidArgument If columnLabel specified already exists in the
                            DataTable_.
    \throws InvalidColumn If the input column contains incorrect number of 
                          rows.                                               */
    void appendColumn(const std::string& columnLabel,
                      const VectorView& depCol) {
        OPENSIM_THROW_IF(getNumRows() == 0,
                         InvalidCall,
                         "DataTable must have one or more rows before we can "
                         "append columns to it.");
        OPENSIM_THROW_IF(hasColumn(columnLabel),
                         InvalidArgument,
                         "Column-label '" + columnLabel + "' already exists in "
                         "the DataTable.");
        OPENSIM_THROW_IF(depCol.nrow() != getNumRows(),
                         IncorrectNumRows,
                         static_cast<size_t>(getNumRows()),
                         static_cast<size_t>(depCol.nrow()));
        
        _depData.resizeKeep(_depData.nrow(), _depData.ncol() + 1);
        _depData.updCol(_depData.ncol() - 1) = depCol;
        appendColumnLabel(columnLabel);
    }

    /** Get dependent column at index.

    \throws ColumnIndexOutOfRange If index is out of range for number of columns
                                  in the table.                               */
    VectorView getDependentColumnAtIndex(size_t index) const {
        OPENSIM_THROW_IF(isColumnIndexOutOfRange(index),
                         ColumnIndexOutOfRange, index, 0,
                         static_cast<size_t>(_depData.ncol() - 1));

        return _depData.col(static_cast<int>(index));
    }

    /** Get dependent Column which has the given column label.                

    \throws KeyNotFound If columnLabel is not found to be label of any existing
                        column.                                               */
    VectorView getDependentColumn(const std::string& columnLabel) const {
        return _depData.col(static_cast<int>(getColumnIndex(columnLabel)));
    }

    /** Update dependent column at index.

    \throws ColumnIndexOutOfRange If index is out of range for number of columns
                                  in the table.                               */
    VectorView updDependentColumnAtIndex(size_t index) {
        OPENSIM_THROW_IF(isColumnIndexOutOfRange(index),
                         ColumnIndexOutOfRange, index, 0,
                         static_cast<size_t>(_depData.ncol() - 1));

        return _depData.updCol(static_cast<int>(index));
    }

    /** Update dependent Column which has the given column label.

    \throws KeyNotFound If columnLabel is not found to be label of any existing
                        column.                                               */
    VectorView updDependentColumn(const std::string& columnLabel) {
        return _depData.updCol(static_cast<int>(getColumnIndex(columnLabel)));
    }

    /** %Set value of the independent column at index.

    \throws RowIndexOutOfRange If rowIndex is out of range.
    \throws InvalidRow If this operation invalidates the row. Validation is
                       performed by derived classes.                          */
    void setIndependentValueAtIndex(size_t rowIndex, const ETX& value) {
        OPENSIM_THROW_IF(isRowIndexOutOfRange(rowIndex),
                         RowIndexOutOfRange, 
                         rowIndex, 0, 
                         static_cast<unsigned>(_indData.size() - 1));

        validateRow(rowIndex, value, _depData.row((int)rowIndex));
        _indData[rowIndex] = value;
    }

    /// @}

    /// @name Matrix accessors/mutators.
    /// Following functions operate on the matrix not including the independent
    /// column.
    /// @{

    /** Get a read-only view to the underlying matrix.                        */
    const MatrixView& getMatrix() const {
        return _depData.getAsMatrixView();
    }

    /** Get a read-only view of a block of the underlying matrix.             

    \throws InvalidArgument If numRows or numColumns is zero.
    \throws RowIndexOutOfRange If one or more rows of the desired block is out
                               of range of the matrix.
    \throws ColumnIndexOutOfRange If one or more columns of the desired block is
                                  out of range of the matrix.                 */
    MatrixView getMatrixBlock(size_t rowStart,
                              size_t columnStart,
                              size_t numRows,
                              size_t numColumns) const {
        OPENSIM_THROW_IF(numRows == 0 || numColumns == 0,
                         InvalidArgument,
                         "Either numRows or numColumns is zero.");
        OPENSIM_THROW_IF(isRowIndexOutOfRange(rowStart),
                         RowIndexOutOfRange,
                         rowStart, 0, 
                         static_cast<unsigned>(_depData.nrow() - 1));
        OPENSIM_THROW_IF(isRowIndexOutOfRange(rowStart + numRows - 1),
                         RowIndexOutOfRange,
                         rowStart + numRows - 1, 0, 
                         static_cast<unsigned>(_depData.nrow() - 1));
        OPENSIM_THROW_IF(isColumnIndexOutOfRange(columnStart),
                         ColumnIndexOutOfRange,
                         columnStart, 0, 
                         static_cast<unsigned>(_depData.ncol() - 1));
        OPENSIM_THROW_IF(isColumnIndexOutOfRange(columnStart + numColumns - 1),
                         ColumnIndexOutOfRange,
                         columnStart + numColumns - 1, 0, 
                         static_cast<unsigned>(_depData.ncol() - 1));

        return _depData.block(static_cast<int>(rowStart),
                              static_cast<int>(columnStart),
                              static_cast<int>(numRows),
                              static_cast<int>(numColumns));
    }

    /** Get a writable view to the underlying matrix.                         */
    MatrixView& updMatrix() {
        return _depData.updAsMatrixView();
    }

    /** Get a writable view of a block of the underlying matrix.

    \throws InvalidArgument If numRows or numColumns is zero.
    \throws RowIndexOutOfRange If one or more rows of the desired block is out
                               of range of the matrix.
    \throws ColumnIndexOutOfRange If one or more columns of the desired block is
                                  out of range of the matrix.                 */
    MatrixView updMatrixBlock(size_t rowStart,
                              size_t columnStart,
                              size_t numRows,
                              size_t numColumns) {
        OPENSIM_THROW_IF(numRows == 0 || numColumns == 0,
                         InvalidArgument,
                         "Either numRows or numColumns is zero.");
        OPENSIM_THROW_IF(isRowIndexOutOfRange(rowStart),
                         RowIndexOutOfRange,
                         rowStart, 0, 
                         static_cast<unsigned>(_depData.nrow() - 1));
        OPENSIM_THROW_IF(isRowIndexOutOfRange(rowStart + numRows - 1),
                         RowIndexOutOfRange,
                         rowStart + numRows - 1, 0, 
                         static_cast<unsigned>(_depData.nrow() - 1));
        OPENSIM_THROW_IF(isColumnIndexOutOfRange(columnStart),
                         ColumnIndexOutOfRange,
                         columnStart, 0, 
                         static_cast<unsigned>(_depData.ncol() - 1));
        OPENSIM_THROW_IF(isColumnIndexOutOfRange(columnStart + numColumns - 1),
                         ColumnIndexOutOfRange,
                         columnStart + numColumns - 1, 0, 
                         static_cast<unsigned>(_depData.ncol() - 1));

        return _depData.updBlock(static_cast<int>(rowStart),
                                 static_cast<int>(columnStart),
                                 static_cast<int>(numRows),
                                 static_cast<int>(numColumns));
    }

    /// @}

    /** Get a string representation of the table, including the key-value pairs
    in the table metadata. Table metadata will be of the form:
    \code
    key => value-converted-to-string
    \endcode
    For example:
    \code
    DataRate => 2000.00000
    Units => mm
    \endcode
    For values in the table metadata that do not support the operation of stream
    insertion (operator<<), the value for metadata will be:
    \code
    key => <cannot-convert-to-string>
    \endcode
    Some examples to call this function:
    \code
    // All rows, all columns.
    auto tableAsString = table.toString();
    // First 5 rows, all columns.
    auto tableAsString = table.toString({0, 1, 2, 3, 4});
    // All rows, 3 columns with specified labels.
    auto tableAsString = table.toString({}, {"col12", "col35", "col4"});
    // Rows 5th, 3rd, 1st (in that order) and columns with specified labels (in 
    // that order).
    auto tableAsString = table.toString({4, 2, 0}, {"col10", "col5", "col2"});
    // Lets say the table has 10 rows. Following will get last 3 rows in the 
    // order specified. All columns.
    auto tableAsString = table.toString({-1, -2, -3})
    \endcode

    \param rows **[Default = all rows]** Sequence of indices of rows to be 
                printed. Rows will be printed exactly in the order specified in 
                the sequence. Index begins at 0 (i.e. first row is 0). Negative
                indices refer to rows starting from last row. Index -1 refers to
                last row, -2 refers to row previous to last row and so on.
                Default behavior is to print all rows. 
    \param columnLabels **[Default = all rows]** Sequence of labels of columns 
                        to be printed. Columns will be printed exactly in the 
                        order specified in the sequence. Default behavior is to 
                        print all columns.
    \param withMetaData **[Default = true]** Whether or not table metadata 
                        should be printed. Default behavior is to print table 
                        metadata.
    \param splitSize **[Default = 25]** Number of rows to print at a time. 
                     Default behavior is to print 25 rows at a time. 
    \param maxWidth **[Default = 80]** Maximum number of characters to print per
                    line. The columns are split accordingly to make the table 
                    readable. This is useful in terminals/consoles with narrow 
                    width. Default behavior is to limit number characters per 
                    line to 80.
    \param precision **[Default = 4]** Precision of the floating-point numbers 
                     printed. Default behavior is to print floating-point 
                     numbers with 4 places to the right of decimal point.     */
    std::string toString(std::vector<int>         rows         = {},
                         std::vector<std::string> columnLabels = {},
                         const bool               withMetaData = true,
                         unsigned                 splitSize    = 25,
                         unsigned                 maxWidth     = 80,
                         unsigned                 precision    = 4) const {
        std::vector<int> cols{};
        for(const auto& label : columnLabels)
            cols.push_back(static_cast<int>(getColumnIndex(label)));
        return toString_impl(rows, cols, withMetaData,
                             splitSize, maxWidth, precision);
    }

protected:
    // Implement toString.
    std::string toString_impl(std::vector<int> rows         = {},
                              std::vector<int> cols         = {},
                              const bool       withMetaData = true,
                              unsigned         splitSize    = 25,
                              unsigned         maxWidth     = 80,
                              unsigned         precision    = 4) const {
        static_assert(std::is_same<ETX, double>::value,
                      "This function can only be called for a table with "
                      "independent column of type 'double'.");
        OPENSIM_THROW_IF(getNumRows() == 0 || getNumColumns() == 0,
                         EmptyTable);

        // Defaults.
        const unsigned    defSplitSize{25};
        const unsigned    defMaxWidth{80};
        const unsigned    defPrecision{4};
        const unsigned    columnSpacing{1};
        const float       excessAllocation{1.25};
        const char        rowNumSepChar{':'};
        const char        fillChar{' '};
        const char        newlineChar{'\n'};
        const std::string indColLabel{"time"};
        const std::string suffixChar{"_"};
        const std::string metaDataSep{" => "};

        // Set all the un-specified parameters to defaults.
        if(splitSize   == 0)
            splitSize  = defSplitSize;
        if(defMaxWidth == 0)
            maxWidth   = defMaxWidth;
        if(precision   == 0)
            precision  = defPrecision;
        if(rows.empty())
            for(size_t i = 0u; i < getNumRows()   ; ++i)
                rows.push_back(i);
        if(cols.empty())
            for(size_t i = 0u; i < getNumColumns(); ++i)
                cols.push_back(i);

        auto toStr = [&] (const double val) {
            std::ostringstream stream{};
            stream << std::fixed << std::setprecision(precision) << val;
            return stream.str();
        };

        std::vector<std::vector<std::string>> table{};

        // Fill up column labels, including row-number label (empty string),
        // time column label and all the column labels from table.
        table.push_back({std::string{}, indColLabel});
        for(int col : cols) {
            if(col < 0)
                col += getNumColumns();
            if(numComponentsPerElement() == 1)
                table.front().push_back(getColumnLabel(col));
            else
                for(unsigned c = 0; c < numComponentsPerElement(); ++c)
                    table.front().push_back(getColumnLabel(col) +
                                            suffixChar +
                                            std::to_string(c + 1));
        }

        // Fill up the rows, including row-number, time column, row data.
        for(int row : rows) {
            if(row < 0)
                row += getNumRows();
            std::vector<std::string> rowData{};
            rowData.push_back(std::to_string(row) + rowNumSepChar);
            rowData.push_back(toStr(getIndependentColumn()[row]));
            for(const auto& col : cols)
                for(const auto& comp :
                        splitElement(getMatrix().getElt(row, col)))
                        rowData.push_back(toStr(comp));
            table.push_back(std::move(rowData));
        }

        // Compute width of each column.
        std::vector<size_t> columnWidths(table.front().size(), 0);
        for(const auto& row : table) {
            for(unsigned col = 0; col < row.size(); ++col)
                columnWidths.at(col) =
                    std::max(columnWidths.at(col),
                             row[col].length() + columnSpacing);
        }
        columnWidths.front() -= 1;

        std::string result{};

        // Fill up metadata.
        if(withMetaData) {
            for(const auto& key : getTableMetaDataKeys()) {
                result.append(key);
                result.append(metaDataSep);
                result.append(getTableMetaDataAsString(key));
                result.push_back(newlineChar);
            }
        }
        
        const size_t totalWidth{std::accumulate(columnWidths.cbegin(),
                                                columnWidths.cend(),
                                                static_cast<size_t>(0))};
        result.reserve(result.capacity() + static_cast<unsigned>(
                       totalWidth * table.size() * excessAllocation));

        // Fill up the result string.
        size_t beginRow{1};
        size_t endRow{std::min(beginRow + splitSize, table.size())};
        while(beginRow < endRow) {
            size_t beginCol{1};
            size_t endCol{columnWidths.size()};
            while(beginCol < endCol) {
                size_t width = std::accumulate(columnWidths.cbegin() + beginCol,
                                               columnWidths.cbegin() + endCol,
                                               static_cast<size_t>(0));
                while(width > maxWidth) {
                    --endCol;
                    width -= columnWidths[endCol];
                }
                result.append(columnWidths[0], fillChar);
                for(unsigned col = beginCol; col < endCol; ++col) {
                    result.append(columnWidths[col] - table[0][col].length(),
                                  fillChar);
                    result.append(table[0][col]);
                }
                result.push_back(newlineChar);
                for(unsigned row = beginRow; row < endRow; ++row) {
                    result.append(columnWidths[0] - table[row][0].length(),
                                  fillChar);
                    result.append(table[row][0]);
                    for(unsigned col = beginCol; col < endCol; ++col) {
                        result.append(columnWidths[col] -
                                      table[row][col].length(), fillChar);
                        result.append(table[row][col]);
                    }
                    result.push_back(newlineChar);
                }
                beginCol = endCol;
                endCol = columnWidths.size();
            }
            beginRow = endRow;
            endRow = std::min(beginRow + splitSize, table.size());
        }
        return result;
    }

    // Split element into constituent components and append the components to
    // the given vector. For example Vec3 has 3 components.
    template<int N>
    static
    void splitElementAndPushBack(std::vector<double>& row,
                                 const SimTK::Vec<N>& elem) {
        for(unsigned i = 0; i < N; ++i)
            row.push_back(elem[i]);
    }
    // Split element into constituent components and append the components to 
    // the given vector. . For example Vec<2, Vec3> has 6 components.
    template<int M, int N>
    static
    void splitElementAndPushBack(std::vector<double>& row,
                                 const SimTK::Vec<M, SimTK::Vec<N>>& elem) {
        for(unsigned i = 0; i < M; ++i)
            for(unsigned j = 0; j < N; ++j)
                row.push_back(elem[i][j]);
    }
    // Unsupported type.
    static
    void splitElementAndPushBack(std::vector<double>&,
                                 ...) {
        static_assert(!std::is_same<ETY, double>::value,
                      "This constructor cannot be used to construct from "
                      "DataTable<double, ThatETY> where ThatETY is an "
                      "unsupported type.");
    }
    template<typename ELT>
    static
    std::vector<double> splitElement(const ELT& elt) {
        std::vector<double> result{};
        splitElementAndPushBack(result, elt);
        return result;
    }
    static
    std::vector<double> splitElement(const double& elt) {
        return {elt};
    }

    template<typename Iter>
    static
    void makeElement_helper(double& elem,
                            Iter begin, Iter end) {
        OPENSIM_THROW_IF(begin == end,
                         InvalidArgument,
                         "Iterators do not produce enough elements."
                         "Expected: 1 Received: 0");
        elem = *begin;
    }
    template<int N, typename Iter>
    static
    void makeElement_helper(SimTK::Vec<N>& elem,
                            Iter begin, Iter end) {
        for(unsigned i = 0; i < N; ++i) {
            OPENSIM_THROW_IF(begin == end,
                             InvalidArgument,
                             "Iterators do not produce enough elements."
                             "Expected: " + std::to_string(N) + " Received: " +
                             std::to_string(i));

            elem[i] = *begin++;
        }
    }
    template<int M, int N, typename Iter>
    static
    void makeElement_helper(SimTK::Vec<M, SimTK::Vec<N>>& elem,
                            Iter begin, Iter end) {
        for(unsigned i = 0; i < M; ++i) {
            for(unsigned j = 0; j < N; ++j) {
                OPENSIM_THROW_IF(begin == end,
                                 InvalidArgument,
                                 "Iterators do not produce enough elements."
                                 "Expected: " + std::to_string(M * N) +
                                 " Received: " + std::to_string((i + 1) * j));

                elem[i][j] = *begin++;
            }
        }
    }
    template<typename Iter>
    static
    ETY makeElement(Iter begin, Iter end) {
        ETY elem{};
        makeElement_helper(elem, begin, end);
        return elem;
    }
    
    
    /** Check if row index is out of range.                                   */
    bool isRowIndexOutOfRange(size_t index) const {
        return index >= _indData.size();
    }

    /** Check if column index is out of range.                                */
    bool isColumnIndexOutOfRange(size_t index) const {
        return index >= static_cast<size_t>(_depData.ncol());
    }

    /** Get number of rows.                                                   */
    size_t implementGetNumRows() const override {
        return _depData.nrow();
    }

    /** Get number of columns.                                                */
    size_t implementGetNumColumns() const override {
        return _depData.ncol();
    }

    /** Validate metadata for independent column.                             
    
    \throws InvalidMetaData If independent column's metadata does not contain
                            a key named "labels".                             */
    void validateIndependentMetaData() const override {
        try {
            _independentMetaData.getValueForKey("labels");
        } catch(KeyNotFound&) {
            OPENSIM_THROW(MissingMetaData, "labels");
        }
    }

    /** Validate metadata for dependent columns.

    \throws InvalidMetaData (1) If metadata for dependent columns does not 
                            contain a key named "labels". (2) If ValueArray
                            for key "labels" does not have length equal to the
                            number of columns in the table. (3) If not all
                            entries in the metadata for dependent columns have
                            the correct length (equal to number of columns).  */
    void validateDependentsMetaData() const override {
        size_t numCols{};
        try {
            numCols = (unsigned)_dependentsMetaData
                                        .getValueArrayForKey("labels").size();
        } catch (KeyNotFound&) {
            OPENSIM_THROW(MissingMetaData, "labels");
        }

        OPENSIM_THROW_IF(numCols == 0,
                         MetaDataLengthZero,"labels");

        OPENSIM_THROW_IF(_depData.ncol() != 0 && 
                         numCols != static_cast<unsigned>(_depData.ncol()),
                         IncorrectMetaDataLength, "labels", 
                         static_cast<size_t>(_depData.ncol()), numCols);

        for(const std::string& key : _dependentsMetaData.getKeys()) {
            OPENSIM_THROW_IF(numCols != 
                             _dependentsMetaData.
                             getValueArrayForKey(key).size(),
                             IncorrectMetaDataLength, key, numCols,
                             _dependentsMetaData.
                             getValueArrayForKey(key).size());
        }
    }

    /** Derived classes optionally can implement this function to validate
    append/update operations.                                                 

    \throws InvalidRow If the given row considered invalid by the derived
                       class.                                                 */
    virtual void validateRow(size_t rowIndex, 
                             const ETX&, 
                             const RowVector&) const {
        // No operation.
    }

    static constexpr
    unsigned numComponentsPerElement_impl(double) {
        return 1;
    }
    template<int M>
    static constexpr
    unsigned numComponentsPerElement_impl(SimTK::Vec<M>) {
        return M;
    }
    template<int M, int N>
    static constexpr
    unsigned numComponentsPerElement_impl(SimTK::Vec<M, SimTK::Vec<N>>) {
        return M * N;
    }

    std::vector<ETX>    _indData;
    SimTK::Matrix_<ETY> _depData;
};  // DataTable_


/** Print DataTable out to a stream.                                          */
template<typename ETX, typename ETY>
std::ostream& operator<<(std::ostream& outStream,
                         const DataTable_<ETX, ETY>& table) {
    return (outStream << table.toString());
}

/** See DataTable_ for details on the interface.                              */
typedef DataTable_<double, double> DataTable;
/** See DataTable_ for details on the interface.                              */
typedef DataTable_<double, SimTK::Vec3> DataTableVec3;

} // namespace OpenSim

#endif //OPENSIM_DATA_TABLE_H_
