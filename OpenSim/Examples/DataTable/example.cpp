/* -------------------------------------------------------------------------- *
 *                            OpenSim:  example.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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

#include "OpenSim/Common/TimeSeriesTable.h"

// Function to do nothing more than  suppress 'un-used variable' warning. 
// Ignore these calls.
template<typename AnyType>
void ignore(AnyType&) {}


// This file demonstrates the use of the OpenSim::DataTable_ class and 
// OpenSim::TimeSeriesTable_ class. See OpenSim::DataTable_ and 
// OpenSim::TimeSeriesTable_ for details on the interface. 
// OpenSim::DataTable_ is a container of a matrix of elements(of any type)
// that supports random-access to the columns and rows through the their
// index. Columns can be labeled with std::string and accessing the columns
// through labels is constant-time on average. DataTable_ can also hold key-
// value pairs as metadata where key is a std::string and value can be of any
// type. OpenSim::TimeSeriesTable_ is a DataTable_ which adds support for
// a timestamp column and rows can be indexed using the timestamps.

int main() {
    // All the examples are shown for three different types of DataTable_.

    constexpr double Epsilon{0.000001};

    // Default construct a DataTable_ of SimTK::Real[alias for double].
    OpenSim::DataTable_<SimTK::Real> dt_real;
    // Default construct a DataTable_ of SimTK::Vec3[a tuple of 3 double(s)].
    OpenSim::DataTable_<SimTK::Vec3> dt_vec3;
    // Default construct a DataTable_ of SimTK::Vec6[a tuple of 6 double(s)].
    OpenSim::DataTable_<SimTK::Vec6> dt_vec6;

    std::vector<SimTK::Real> data_real;
    std::vector<SimTK::Vec3> data_vec3;
    std::vector<SimTK::Vec6> data_vec6;
    for(int i = 0; i < 12; ++i) {
        data_real.push_back(i);
        data_vec3.push_back(SimTK::Vec3{1, 2, 3} + i);
        data_vec6.push_back(SimTK::Vec6{1, 2, 3, 4, 5, 6} + i);
    }

    // Construct a DataTable_ using iterators. We can populate the elements
    // row-wise/RowMajor or column-wise/ColumnMajor using the argument 
    // 'traverseDir'. In order to populate the DataTable_, the function needs 
    // to know the length each row/column depending whether traverseDir is 
    // RowMajor or ColumnMajor. If traverseDir is RowMajor, then the function 
    // needs to know the length of each row to populate the DataTable_. 
    // Similarly if traverseDir is ColumnMajor, then the function needs to know
    // the length each column to populate the DataTable_. This is specified
    // through the argument 'numEntriesInMajor'. The following creates a 4x3 
    // DataTable_ filling the elements column-wise.
    // For this example, it is possible to specify the total number of rows 
    // being added so the constructor can perform some optimization. We will 
    // omit that argument here but if specified, its value would be 4 (rows).
    {
        OpenSim::DataTable_<SimTK::Real> 
            real{data_real.cbegin(),
                data_real.cend(),
                3,
                OpenSim::TraverseDir::ColumnMajor};
        OpenSim::DataTable_<SimTK::Vec3> 
            vec3{data_vec3.cbegin(),
                data_vec3.cend(),
                3,
                OpenSim::TraverseDir::ColumnMajor};
        OpenSim::DataTable_<SimTK::Vec6> 
            vec6{data_vec6.cbegin(),
                data_vec6.cend(),
                3,
                OpenSim::TraverseDir::ColumnMajor};
    }

    // DataTable_ can be constructed by passing the container itself. The
    // following is equivalent to the above calls using iterators but the 
    // function inquires the size of the container if possible and performs
    // optimization. 
    // Using iterators like above offers the advantage of using a smaller set
    // of elements in the container.
    {
        OpenSim::DataTable_<SimTK::Real> 
            real{data_real,
                3,
                OpenSim::TraverseDir::ColumnMajor};
        OpenSim::DataTable_<SimTK::Vec3> 
            vec3{data_vec3,
                3,
                OpenSim::TraverseDir::ColumnMajor};
        OpenSim::DataTable_<SimTK::Vec6> 
            vec6{data_vec6,
                3,
                OpenSim::TraverseDir::ColumnMajor};
    }

    // Add multiple rows at once using a container. When adding rows to an empty
    // DataTable_, the function needs to know the length of each row. This can
    // be specified through the argument numColumns. We have 12 elements in the
    // iterator and the following adds 3 rows of length 4 each. After this,
    // the DataTable will be 3x4.
    // Just like the constructors above, this function also takes iterators and
    // the below call is equivalent to the call using iterators.
    dt_real.addRows(data_real, 4);
    dt_vec3.addRows(data_vec3, 4);
    dt_vec6.addRows(data_vec6, 4);

    std::vector<SimTK::Real> data1_real(4, 1);
    std::vector<SimTK::Vec3> data1_vec3(4, {1, 2, 3});
    std::vector<SimTK::Vec6> data1_vec6(4, {1, 2, 3, 4, 5, 6});

    // A row can also be added to the DataTable_ by feeding a SimTK::RowVector_ 
    // to the addRow() function. After the below calls the DataTable_ will be 
    // of size 4x4.
    SimTK::RowVector_<SimTK::Real> row_real{static_cast<int>(data1_real.size()),
                                            data1_real.data()};
    SimTK::RowVector_<SimTK::Vec3> row_vec3{static_cast<int>(data1_vec3.size()),
                                            data1_vec3.data()};
    SimTK::RowVector_<SimTK::Vec6> row_vec6{static_cast<int>(data1_vec6.size()),
                                            data1_vec6.data()};
    dt_real.addRow(row_real);
    dt_vec3.addRow(row_vec3);
    dt_vec6.addRow(row_vec6);

    // Add a single row to the DataTable using an iterator. Note that this 
    // function is different from the one used above, which is addRows(). The
    // following is equivalent to feeding addRow() the container itself. 
    dt_real.addRow(data1_real.cbegin(), data1_real.cend());
    dt_vec3.addRow(data1_vec3.cbegin(), data1_vec3.cend());
    dt_vec6.addRow(data1_vec6.cbegin(), data1_vec6.cend());

    // Trying to add a row using an iterator that does not produce enough 
    // elements to fill up the row will throw an exception as follows.
    try {
        dt_real.addRow(data1_real.cbegin(), data1_real.cend() - 1);
    } catch(OpenSim::NotEnoughElements&) {}
    try {
        dt_vec3.addRow(data1_vec3.cbegin(), data1_vec3.cend() - 1);
    } catch(OpenSim::NotEnoughElements&) {}
    try {
        dt_vec6.addRow(data1_vec6.cbegin(), data1_vec6.cend() - 1);
    } catch(OpenSim::NotEnoughElements&) {}

    // To suppress the above exception, DataTable can be told to allow for 
    // missing values as follows. In this case, the missing values will be 
    // initialized with SimTK::NaN. See the documentation for addRow() for 
    // details on the arguments. The third argument is ignored in 
    // this call.
    dt_real.addRow(data1_real.cbegin(), data1_real.cend() - 1, 0, true);
    dt_vec3.addRow(data1_vec3.cbegin(), data1_vec3.cend() - 1, 0, true);
    dt_vec6.addRow(data1_vec6.cbegin(), data1_vec6.cend() - 1, 0, true);

    // DataTable has the same interface for adding columns as well.
    // Add a column using a SimTK::Vector_.
    std::vector<SimTK::Real> data2_real(7, 1);
    std::vector<SimTK::Vec3> data2_vec3(7, {1, 2, 3});
    std::vector<SimTK::Vec6> data2_vec6(7, {1, 2, 3, 4, 5, 6});
    SimTK::Vector_<SimTK::Real> col_real{static_cast<int>(data2_real.size()),
                                         data2_real.data()};
    SimTK::Vector_<SimTK::Vec3> col_vec3{static_cast<int>(data2_vec3.size()),
                                         data2_vec3.data()};
    SimTK::Vector_<SimTK::Vec6> col_vec6{static_cast<int>(data2_vec6.size()),
                                         data2_vec6.data()};
    dt_real.addColumn(col_real);
    dt_vec3.addColumn(col_vec3);
    dt_vec6.addColumn(col_vec6);

    // Add a column using an iterator.
    dt_real.addColumn(data2_real.cbegin(), data2_real.cend());
    dt_vec3.addColumn(data2_vec3.cbegin(), data2_vec3.cend());
    dt_vec6.addColumn(data2_vec6.cbegin(), data2_vec6.cend());

    // Add multiple columns using a container. 
    for(int i = 0; i < 7; ++i) {
        data2_real.push_back(i);
        data2_vec3.push_back(SimTK::Vec3{1, 2, 3} + i);
        data2_vec6.push_back(SimTK::Vec6{1, 2, 3, 4, 5, 6} + i);
    }
    dt_real.addColumns(data2_real);
    dt_vec3.addColumns(data2_vec3);
    dt_vec6.addColumns(data2_vec6);

    // Label the columns.
    dt_real.setColumnLabels({"one", "two", "three", "four", 
                             "five", "six", "seven", "eight"});
    dt_vec3.setColumnLabels({"one", "two", "three", "four", 
                             "five", "six", "seven", "eight"});
    dt_vec6.setColumnLabels({"one", "two", "three", "four", 
                             "five", "six", "seven", "eight"});

    // DataTable is copiable. Copy construct DataTable.
    OpenSim::DataTable_<SimTK::Real> dt_real_copy{dt_real};
    OpenSim::DataTable_<SimTK::Vec3> dt_vec3_copy{dt_vec3};
    OpenSim::DataTable_<SimTK::Vec6> dt_vec6_copy{dt_vec6};

    // Concatenate/append an entire DataTable by row. Columns of the input 
    // DataTable are appended to the respective columns based on column labels.
    dt_real.concatenateRows(dt_real_copy);
    dt_vec3.concatenateRows(dt_vec3_copy);
    dt_vec6.concatenateRows(dt_vec6_copy);

    // Copy assign DataTable.
    dt_real_copy = dt_real;
    dt_vec3_copy = dt_vec3;
    dt_vec6_copy = dt_vec6;

    dt_real_copy.changeColumnLabels({"nine", "ten", "eleven", "twelve",
                                 "thirteen", "fourteen", "fifteen", "sixteen"});
    dt_vec3_copy.changeColumnLabels({"nine", "ten", "eleven", "twelve",
                                 "thirteen", "fourteen", "fifteen", "sixteen"});
    dt_vec6_copy.changeColumnLabels({"nine", "ten", "eleven", "twelve",
                                 "thirteen", "fourteen", "fifteen", "sixteen"});

    // Concatenate an entire DataTable by column. Column labels of the input 
    // DataTable must be distinct from exisitng column labels.
    dt_real.concatenateColumns(dt_real_copy);
    dt_vec3.concatenateColumns(dt_vec3_copy);
    dt_vec6.concatenateColumns(dt_vec6_copy);

    // Concatenate two existing DataTable(s) and produce a new one instead of 
    // appending one to another.
    {
        // Construct DataTable of a given size with a default value.
        OpenSim::DataTable_<SimTK::Real> dt1_real{3, 4, 
                                                  SimTK::Real{10}};
        OpenSim::DataTable_<SimTK::Vec3> dt1_vec3{3, 4, 
                                                  SimTK::Vec3{10, 20, 30}};
        OpenSim::DataTable_<SimTK::Vec6> dt1_vec6{3, 4, 
                                                  SimTK::Vec6{10, 20, 30, 
                                                              40, 50, 60}};

        // Construct DataTable of a given size with a default value.
        OpenSim::DataTable_<SimTK::Real> dt2_real{3, 4, 
                                                  SimTK::Real{5}};
        OpenSim::DataTable_<SimTK::Vec3> dt2_vec3{3, 4, 
                                                  SimTK::Vec3{5, 6, 7}};
        OpenSim::DataTable_<SimTK::Vec6> dt2_vec6{3, 4, 
                                                  SimTK::Vec6{5, 6, 7, 
                                                              8, 9, 0}};

        // Label the columns. Concatenation by row requires column labels.
        std::vector<std::string> dt1_labels{"one", "two", "three", "four"};
        dt1_real.setColumnLabels(dt1_labels);
        dt1_vec3.setColumnLabels(dt1_labels);
        dt1_vec6.setColumnLabels(dt1_labels);
        dt2_real.setColumnLabels(dt1_labels);
        dt2_vec3.setColumnLabels(dt1_labels);
        dt2_vec6.setColumnLabels(dt1_labels);

        // Concatenate by row. 
        auto byrow_dt_real = concatenateRows(dt1_real, dt2_real);
        auto byrow_dt_vec3 = concatenateRows(dt1_vec3, dt2_vec3);
        auto byrow_dt_vec6 = concatenateRows(dt1_vec6, dt2_vec6);

        // Change the column labels of one of the tables. Concatenation by
        // column requries the tables to have distinct column labels.
        std::vector<std::string> dt2_labels{"five", "six", "seven", "eight"};
        dt2_real.changeColumnLabels(dt2_labels);
        dt2_vec3.changeColumnLabels(dt2_labels);
        dt2_vec6.changeColumnLabels(dt2_labels);

        // Concatenate by column. 
        auto bycol_dt_real = concatenateColumns(dt1_real, dt2_real);
        auto bycol_dt_vec3 = concatenateColumns(dt1_vec3, dt2_vec3);
        auto bycol_dt_vec6 = concatenateColumns(dt1_vec6, dt2_vec6);
    }

    // Not all columns need to be labeled. 
    dt_real.clearColumnLabels();
    dt_vec3.clearColumnLabels();
    dt_vec6.clearColumnLabels();
    dt_real.setColumnLabel(0, "col-zero");
    dt_vec3.setColumnLabel(0, "col-zero");
    dt_vec6.setColumnLabel(0, "col-zero");
    dt_real.setColumnLabel(2, "col-two");
    dt_vec3.setColumnLabel(2, "col-two");
    dt_vec6.setColumnLabel(2, "col-two");
    dt_real.setColumnLabel(4, "col-four");
    dt_vec3.setColumnLabel(4, "col-four");
    dt_vec6.setColumnLabel(4, "col-four");

    // Retrieve number of rows and cols in the DataTable.
    assert(dt_real.getNumRows() == 14 && dt_real.getNumColumns() == 16);
    assert(dt_vec3.getNumRows() == 14 && dt_vec3.getNumColumns() == 16);
    assert(dt_vec6.getNumRows() == 14 && dt_vec6.getNumColumns() == 16);

    // Get a row/col of by its index. Result of get...() functions is not 
    // writable. See SimTK::RowVectorView_ and SimTK::VectorView for details on 
    // how to use the result. For example, elements of the row/col can be read 
    // using 'operator[]'.
    {
        const auto row2_real = dt_real.getRow(2);
        const auto row2_vec3 = dt_vec3.getRow(2);
        const auto row2_vec6 = dt_vec6.getRow(2);

        const auto col2_real = dt_real.getColumn(2);
        const auto col2_vec3 = dt_vec3.getColumn(2);
        const auto col2_vec6 = dt_vec6.getColumn(2);

        // Columns can also be retrieved using their labels, if they have been
        // given one.
        const auto col2_real_copy = dt_real.getColumn("col-two");
        const auto col2_vec3_copy = dt_vec3.getColumn("col-two");
        const auto col2_vec6_copy = dt_vec6.getColumn("col-two");
        for(int i = 0; i < col2_real.size(); ++i) {
            assert(col2_real[i] == col2_real_copy[i]);
            assert(col2_vec3[i] == col2_vec3_copy[i]);
            assert(col2_vec6[i] == col2_vec6_copy[i]);
        }
    }
    // Update a row/col by retrieving it using its index. We use upd...() 
    // functions to obtain a writable reference.
    {
        auto row2_real = dt_real.updRow(2);
        auto row2_vec3 = dt_vec3.updRow(2);
        auto row2_vec6 = dt_vec6.updRow(2);
        row2_real *= 2;
        row2_vec3 *= 2;
        row2_vec6 *= 2;

        auto col2_real = dt_real.updColumn(2);
        auto col2_vec3 = dt_vec3.updColumn(2);
        auto col2_vec6 = dt_vec6.updColumn(2);
        col2_real *= 2;
        col2_vec3 *= 2;
        col2_vec6 *= 2;

        // Columns can also be updated using their lables, if they have been
        // given one.
        dt_real.updColumn("col-two") *= 2;
        dt_vec3.updColumn("col-two") *= 2;
        dt_vec6.updColumn("col-two") *= 2;
    }

    // Individual elements of the DataTable can be retrieved using row-col pair.
    // Result of get...() functions is not writable.
    {
        auto elem1_real = dt_real.getElt(5, 7); ignore(elem1_real);
        auto elem1_vec3 = dt_vec3.getElt(5, 7); ignore(elem1_vec3);
        auto elem1_vec6 = dt_vec6.getElt(5, 7); ignore(elem1_vec6);

        // Column labels can be used in place of column index, if the columns 
        // have been given labels.
        auto elem2_real = dt_real.getElt(9, "col-four"); ignore(elem2_real);
        auto elem2_vec3 = dt_vec3.getElt(9, "col-four"); ignore(elem2_vec3);
        auto elem2_vec6 = dt_vec6.getElt(9, "col-four"); ignore(elem2_vec6);
    }
    // Update an element by retrieving it using its row-col pair with upd...()
    // functions. The upd...() functions return a writable reference to the 
    // element.
    {
        dt_real.updElt(5, 7) *= 2;
        dt_vec3.updElt(5, 7) *= 2;
        dt_vec6.updElt(5, 7) *= 2;

        // Column labels can be used in place of column index, if the columns 
        // have been given labels.
        dt_real.updElt(9, "col-four") *= 2;
        dt_vec3.updElt(9, "col-four") *= 2;
        dt_vec6.updElt(9, "col-four") *= 2;
    }

    // It is possible to obtain read-only view of a sub-matrix in the 
    // DataTable. Here we obtain 2x2 matrix starting at row 3, col 4.
    // See SimTK::MatrixView_ for details on using the result to query various
    // elements.
    {
        auto submat_real = dt_real.getMatrix(3, 4, 2, 2); ignore(submat_real);
        auto submat_vec3 = dt_vec3.getMatrix(3, 4, 2, 2); ignore(submat_vec3);
        auto submat_vec6 = dt_vec6.getMatrix(3, 4, 2, 2); ignore(submat_vec6);
    }
    // Update a sub-matrix of a DataTable by retrieving it using updMatrix()
    // function which returns a writable reference to the sub-matrix.
    // See SimTK::MatrixView_ for details on supported operations on the
    // result.
    {
        dt_real.updMatrix(3, 4, 2, 2) *= 2;
        dt_vec3.updMatrix(3, 4, 2, 2) *= 2;
        dt_vec6.updMatrix(3, 4, 2, 2) *= 2;
    }

    // Get a copy of the underlying matrix.
    auto mat_real = dt_real.copyAsMatrix(); ignore(mat_real);
    auto mat_vec3 = dt_vec3.copyAsMatrix(); ignore(mat_real);
    auto mat_vec6 = dt_vec6.copyAsMatrix(); ignore(mat_real);

    // It is possible to clear the data in the DataTable if we want to fill it 
    // up with other data later. This also clears the column labels.
    dt_real.clearData();
    dt_vec3.clearData();
    dt_vec6.clearData();

    // TimeSeriesTable_ is a DataTable_ that adds support for a timestamp
    // column that can be used to index rows.
    // Useful type aliases. The first template argument is the same as the one
    // going into DataTable_ -- it is the type of the entries in the matrix. The
    // second template argument is the type of the timestamp column.
    using TsDataTableReal = OpenSim::TimeSeriesTable_<SimTK::Real>;
    using TsDataTableVec3 = OpenSim::TimeSeriesTable_<SimTK::Vec3>;
    using TsDataTableVec6 = OpenSim::TimeSeriesTable_<SimTK::Vec6>;

    // Constructing a TimeSeriesTable_ is same as constructing a DataTable_.
    // Timestamps can be added later to the timestamp column. The below code
    // constructs a TimeSeriesTable_ of size 5x3 where all entries are
    // filled will a given value.
    TsDataTableReal tsdt_real{5, 3, 10};
    TsDataTableVec3 tsdt_vec3{5, 3, SimTK::Vec3{10, 20, 30}};
    TsDataTableVec6 tsdt_vec6{5, 3, SimTK::Vec6{10, 20, 30, 40, 50, 60}};

    // Add timestamps to the timestamp column. Timestamp column must be
    // increasing and before it is accessed, its length must equal the number of
    // rows. Timestamps need not be added all at once but all rows must have an
    // associated timestamp before timestamp column is used.
    // The below code adds 5 timestamps, one for each row. Note the use of
    // "0.0" and "1.0" instead of just "0" and "1". This is required so that
    // all the entries in the list are deduced to be of the same type.
    // Also note that type of the timestamp column and type of the entries in 
    // the matrix can be different and they are in our examples -- Timestamp
    // column entries are of type float and matrix entries are of type
    // SimTK::Real, SimTK::Vec3 and SimTK::Vec6.
    tsdt_real.addTimes({0.0, 0.25, 0.50, 0.75, 1.0});
    tsdt_vec3.addTimes({0.0, 0.25, 0.50, 0.75, 1.0});
    tsdt_vec6.addTimes({0.0, 0.25, 0.50, 0.75, 1.0});

    // Both a timestamp and a row can be added in one call. Multiple timestamps
    // and rows can be added in one call as well using addTimestampsAndRows().
    data_real.clear();
    data_vec3.clear();
    data_vec6.clear();
    data_real.assign(3, 20);
    data_vec3.assign(3, SimTK::Vec3{40, 50, 60});
    data_vec6.assign(3, SimTK::Vec6{70, 80, 90, 100, 110, 120});
    tsdt_real.addTimeAndRow(1.25, data_real);
    tsdt_vec3.addTimeAndRow(1.25, data_vec3);
    tsdt_vec6.addTimeAndRow(1.25, data_vec6);

    // Timestamps can be changed. Following changes timestamps of all the rows
    // at once. It is possible to specify a starting row index to change 
    // timestamps for only a subset of rows. See documentation for details.
    tsdt_real.changeTimes({0.0, 0.2, 0.4, 0.6, 0.8, 1.0});
    tsdt_vec3.changeTimes({0.0, 0.2, 0.4, 0.6, 0.8, 1.0});
    tsdt_vec6.changeTimes({0.0, 0.2, 0.4, 0.6, 0.8, 1.0});

    // Labeling of columns is exactly same as that for DataTable_.
    tsdt_real.setColumnLabels({"col-one", "col-two", "col-three"});
    tsdt_vec3.setColumnLabels({"col-one", "col-two", "col-three"});
    tsdt_vec6.setColumnLabels({"col-one", "col-two", "col-three"});

    // Rows can be indexed on timestamps.
    {
        auto dtrow_real = tsdt_real.getRowAtTime(0.6); ignore(dtrow_real);
        auto dtrow_vec3 = tsdt_vec3.getRowAtTime(0.6); ignore(dtrow_vec3);
        auto dtrow_vec6 = tsdt_vec6.getRowAtTime(0.6); ignore(dtrow_vec6);

        tsdt_real.updRowAtTime(0.4) *= 2;
        tsdt_vec3.updRowAtTime(0.4) *= 2;
        tsdt_vec6.updRowAtTime(0.4) *= 2;
    }

    // Timestamps used to index the rows need not exist in the timestamp
    // column. A row with an approximate match can be obtained as follows.
    {
        using OpenSim::NearestDir;

        // Get a row whose timestamp is less than or equal to given timestamp.
        // Row 2 (3rd row) is returned for below call.
        auto dtrow_real = tsdt_real.getRowAtTime(0.5, 
                                            NearestDir::LessThanEqual);
        ignore(dtrow_real);
        // Get a row whose timestamp is greater than or equal to the given
        // timestamp. Row 3 (4th row) is returned for below call.
        auto dtrow_vec3 = tsdt_real.getRowAtTime(0.5, 
                                            NearestDir::GreaterThanEqual);
        ignore(dtrow_vec3);
        // Get a row whose timestamp is closest in either direction to the
        // given timestamp. Row 3 (4th row) is returned for below call.
        auto dtrow_vec6 = tsdt_real.getRowAtTime(0.5, 
                                            NearestDir::LessOrGreaterThanEqual);
        ignore(dtrow_vec6);

        
        tsdt_real.updRowAtTime(0.5, NearestDir::LessThanEqual)          *= 2;
        tsdt_vec3.updRowAtTime(0.5, NearestDir::GreaterThanEqual)       *= 2;
        tsdt_vec6.updRowAtTime(0.5, NearestDir::LessOrGreaterThanEqual) *= 2;
    }

    // Individual elements can indexed in four different ways:
    // - (row index, column index) pair. Use getElt().
    // - (row index, column label) pair. Use getElt().
    // - (timestamp, column index) pair. Use getEltAtTimestamp().
    // - (timestamp, column label) pair. Use getEltAtTimestamp().
    // For updating the elements, upd...() functions can be used.
    {
        // All the following calls access element (3, 2).
        tsdt_real.updElt(3, 2)                      *= 2;
        tsdt_real.updElt(3, "col-two")              *= 2;
        tsdt_real.updEltAtTime(0.6, 2)         *= 2;
        tsdt_real.updEltAtTime(0.6, "col-two") *= 2;
    }

    // Timestamps can be iterated over.
    for(const auto& time : tsdt_real.getTimes()) 
        // Do somthing with timestamp.
        ignore(time);
    for(const auto& time : tsdt_vec3.getTimes())
        // Do somthing with timestamp.
        ignore(time);
    for(const auto& time : tsdt_vec6.getTimes())
        // Do somthing with timestamp.
        ignore(time);

    // Rows can be iterated over.
    for(auto row : tsdt_real.getRows())
        // Do something with row.
        ignore(row);

    // Columns can be iterated over.
    for(auto column : tsdt_vec3.getColumns())
        // Do something with column.
        ignore(column);

    // Rows can be updated while iterating over them.
    for(auto row : tsdt_vec6.updRows())
        row *= 2;

    // Columns can be updated while iterating over them.
    for(auto column : tsdt_vec3.updColumns())
        column *= 2;

    // DataTable can hold metadata in form of key-value pairs where key is 
    // always a std::string and value can be of any type. All DataTables, 
    // regardless of the underlying type, can hold metadata.
  
    // Create some objects that represent metadata.
    int integer{200};
    std::string string{"metadata"};

    // Insert an int as metadata.
    dt_real.insertMetaData("integer", integer);

    // Insert a std::string as metadata.
    dt_real.insertMetaData("string", string);

    // Insert a SimTK::Matrix_ as metadata.
    dt_real.insertMetaData("simtk_matrix", mat_real);

    // Insert a std::vector<SimTK::Real> as metadata.
    dt_real.insertMetaData("data_real", data_real);

    // Insert another DataTable as metadata.
    dt_real.insertMetaData("datatable", dt_vec3);


    // To retrieve the objects stored as metadata, two things have to be 
    // provided. The key under which the object was stored and the exact type 
    // of the object.

    // Retrieve int from metadata.
    assert(dt_real.getMetaData<int>("integer") == integer);

    // Retrieve std::string from metadata.
    assert(dt_real.getMetaData<std::string>("string") == string);

    // Retrieve SimTK::Matrix_ from metadata.
    const auto& mat_real_copy = 
        dt_real.getMetaData<SimTK::Matrix_<double>>("simtk_matrix");
    for(int r = 0; r < mat_real.nrow(); ++r)
        for(int c = 0; c < mat_real.ncol(); ++c) {
            auto a = mat_real_copy.getElt(r, c);
            auto b = mat_real.getElt(r, c);
            assert(std::abs(a - b) < Epsilon || 
                   (std::isnan(a) && std::isnan(b)));
        }

    // Retrieve std::vector from metadata.
    using DataReal = std::vector<SimTK::Real>;
    auto data_real_copy = dt_real.getMetaData<DataReal>("data_real");

    // Retrieve the datatable from metadata.
    using DataTableVec3 = OpenSim::DataTable_<SimTK::Vec3>;
    auto dt_vec3_cpy = dt_real.getMetaData<DataTableVec3>("datatable");

  
    // DataTables can be stored in standard containers. They will first have to 
    // upcasted to AbstractDataTable in order to do so.

    OpenSim::DataTable_<SimTK::Real> table_real{3, 4, 10};
    OpenSim::DataTable_<SimTK::Vec3> table_vec3{3, 4, {10, 20, 30}};
    OpenSim::DataTable_<SimTK::Vec6> table_vec6{3, 4, {10, 20, 30, 40, 50, 60}};

    // Sequence container. Other containers templated on 
    // OpenSim::AbstractDataTable* should also work.
    std::vector<OpenSim::AbstractDataTable*> vector;

    // Add the DataTables to vector as pointers. The upcast happens implicitly.
    vector.push_back(&table_real);
    vector.push_back(&table_vec3);
    vector.push_back(&table_vec6);

    // Add column labels to all the DataTables through pointers stored in the
    // container.
    for(auto& dt : vector) {
        dt->setColumnLabel(0, "col-zero");
        dt->setColumnLabel(2, "col-two");
    }

    // Check if a column index has label associated.
    for(auto& dt : vector) {
        assert(dt->columnHasLabel(0) == true);
        assert(dt->columnHasLabel(2) == true);
        assert(dt->columnHasLabel(1) == false);
        assert(dt->columnHasLabel(3) == false);
    }

    // Retrieve column labels using get...() method.
    for(auto& dt : vector) {
        assert(dt->getColumnLabel(0) == "col-zero");
        assert(dt->getColumnLabel(2) == "col-two");
    }

    // Check if a column label exists in the DataTable.
    for(auto& dt : vector) {
        assert(dt->hasColumn("col-zero") == true);
        assert(dt->hasColumn("col-two") == true);
    }

    // Retrieve the column index using its label.
    assert(vector[0]->getColumnIndex("col-zero") == 0 &&
           table_real.getColumnIndex("col-zero") == 0);
    assert(vector[2]->getColumnIndex("col-two")  == 2 &&
           table_real.getColumnIndex("col-two")  == 2);
    assert(vector[0]->getColumnIndex("col-zero") == 0 &&
           table_vec3.getColumnIndex("col-zero") == 0);
    assert(vector[2]->getColumnIndex("col-two")  == 2 &&
           table_vec3.getColumnIndex("col-two")  == 2);
    assert(vector[0]->getColumnIndex("col-zero") == 0 &&
           table_vec6.getColumnIndex("col-zero") == 0);
    assert(vector[2]->getColumnIndex("col-two")  == 2 &&
           table_vec6.getColumnIndex("col-two")  == 2);

    // Update column labels using upd...() method. Update can be done using 
    // index or using column label.
    for(auto& dt : vector) {
        dt->changeColumnLabel(0        , "column-zero");
        dt->changeColumnLabel("col-two", "column-two");
    }

    // Clear the column labels.
    for(auto& dt : vector)
        dt->clearColumnLabels();

    return 0;
}
