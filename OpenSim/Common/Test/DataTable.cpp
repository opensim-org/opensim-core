/* -------------------------------------------------------------------------- *
 *                            OpenSim:  DataTable.cpp                         *
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

// Standard headers.
#include <vector>
// Non-standard headers.
#include <OpenSim/Common/DataTable.h>


// Exception thrown when testing code in the file fails.
class OpenSimTestFailed : public std::runtime_error {
public:
    OpenSimTestFailed(const std::string& expl) : runtime_error(expl) {}
    OpenSimTestFailed(const char* expl) : runtime_error(expl) {}
};


// Function that ignores its arguments. Used to suppress compiler warning 
// about unused variable.
template<typename... AnyType>
void ignore(AnyType&&...) {}


// Check if the DataTable is empty. 
template<typename DT>
void checkDataTableLimits(DT& dt, size_t nrow, size_t ncol) {
    // Number of rows and cols must be zeros after default construction.
    if(dt.getNumRows() != nrow) {
        throw OpenSimTestFailed{"dt.getNumRows() = " + 
                std::to_string(dt.getNumRows()) +
                " and nrow = " + std::to_string(nrow)};
    }
    if(dt.getNumColumns() != ncol)
        throw OpenSimTestFailed{"dt.getNumColumns() = " + 
                std::to_string(dt.getNumColumns()) +
                " and ncol = " + std::to_string(ncol)};

    // Retrieving any row and col must throw since DataTable is empty.
    try {
        auto row = dt.getRow(nrow); ignore(row);
    } catch (OpenSim::RowDoesNotExist&) {}
    try {
        auto row = dt.updRow(nrow); ignore(row);
    } catch (OpenSim::RowDoesNotExist&) {}
    try {
        auto col = dt.getColumn(ncol); ignore(col);
    } catch (OpenSim::ColumnDoesNotExist&) {}
    try {
        auto col = dt.getColumn("foo"); ignore(col);
    } catch (OpenSim::ColumnDoesNotExist&) {}
    try {
        auto col = dt.updColumn(ncol); ignore(col);
    } catch (OpenSim::ColumnDoesNotExist&) {}
    try {
        auto col = dt.updColumn("foo"); ignore(col);
    } catch (OpenSim::ColumnDoesNotExist&) {}

    // Retrieving any element should throw since the DataTable is empty.
    try {
        auto elt = dt.getElt(nrow, ncol); ignore(elt);
    } catch (OpenSim::RowDoesNotExist&) {}
    try {
        auto elt = dt.getElt(nrow, "foo"); ignore(elt);
    } catch (OpenSim::RowDoesNotExist&) {}
    try {
        auto elt = dt.updElt(nrow, ncol); ignore(elt);
    } catch (OpenSim::RowDoesNotExist&) {}
    try {
        auto elt = dt.updElt(nrow, "foo"); ignore(elt);
    } catch (OpenSim::RowDoesNotExist&) {}

    auto underlying_matrix = dt.copyAsMatrix(); ignore(underlying_matrix);
}


// Check entries of a row in the DataTable.
template<typename ET>
void checkDataTableRow(OpenSim::DataTable_<ET>& dt, 
                       const std::vector<ET>& data,
                       size_t rownum) {
    constexpr double P_EPS{0.0001};
    constexpr double N_EPS{-0.0001};
    // Check entries of the row against data.
    {
        const auto row = dt.getRow(rownum);
        for(size_t i = 0; i < data.size(); ++i)
            assert(row[static_cast<int>(i)] - data[i] > N_EPS &&
                   row[static_cast<int>(i)] - data[i] < P_EPS);
    }

    // Edit the row.
    {
        auto row = dt.updRow(rownum);
        for(size_t i = 0; i < data.size(); ++i)
            row[static_cast<int>(i)] += 1;
    }

    // Check if editing the row worked.
    {
        const auto row = dt.getRow(rownum);
        for(size_t i = 0; i < data.size(); ++i)
            assert(row[static_cast<int>(i)] - (data[i] + 1) > N_EPS &&
                   row[static_cast<int>(i)] - (data[i] + 1) < P_EPS);
    }

    // Edit individual values back to original.
    for(size_t i = 0; i < data.size(); ++i)
        dt.updElt(rownum, i) -= 1;

    // Check individual elements.
    for(size_t i = 0; i < data.size(); ++i)
        assert(dt.getElt(rownum, i) - data[i] > N_EPS &&
               dt.getElt(rownum, i) - data[i] < P_EPS);
}


// Check entries of a col in the DataTable.
template<typename ET>
void checkDataTableCol(OpenSim::DataTable_<ET>& dt, 
                       const std::vector<ET>& data,
                       size_t colnum) {
    constexpr double P_EPS{0.0001};
    constexpr double N_EPS{-0.0001};

    // Check entries of the col against data.
    {
        const auto col = dt.getColumn(colnum);
        for(size_t i = 0; i < data.size(); ++i)
            assert(col[static_cast<int>(i)] - data[i] > N_EPS &&
                   col[static_cast<int>(i)] - data[i] < P_EPS);
    }

    // Edit the col.
    {
        auto col = dt.updColumn(colnum);
        for(size_t i = 0; i < data.size(); ++i)
            col[static_cast<int>(i)] += 1;
    }

    // Check if editing the col worked.
    {
        const auto col = dt.getColumn(colnum);
        for(size_t i = 0; i < data.size(); ++i)
            assert(col[static_cast<int>(i)] - (data[i] + 1) > N_EPS &&
                   col[static_cast<int>(i)] - (data[i] + 1) < P_EPS);
    }

    // Edit individual values back to original.
    for(size_t i = 0; i < data.size(); ++i)
        dt.updElt(i, colnum) -= 1;

    // Check individual elements.
    for(size_t i = 0; i < data.size(); ++i)
        assert(dt.getElt(i, colnum) - data[i] > N_EPS &&
               dt.getElt(i, colnum) - data[i] < P_EPS);
}


// Test adding rows to an empty DataTable. Use the RowVector for 1st row and 
// use the iterator for the 2nd row.
template<typename ET>
void testAddRow(OpenSim::DataTable_<ET>& dt, 
                const std::vector<ET>& data) {
    size_t orig_nrow{dt.getNumRows()};

    if(dt.getNumRows() > 0) {
        // Copy the original DataTable.
        auto dt_copy1 = dt;

        // Try adding a row with insufficient number of columns.
        try {
            dt_copy1.addRow(data.cbegin(), data.cend() - 1);
        } catch(OpenSim::NotEnoughElements&) {}

        // Copy the original DataTable.
        auto dt_copy2 = dt;

        // Try adding a row with insufficient number of columns with 
        // allow_missing argument set to true.
        dt_copy2.addRow(data.cbegin(), data.cend() - 1, 2, true);
    
        // Check the datatable size.
        checkDataTableLimits(dt_copy2, dt.getNumRows() + 1, dt.getNumColumns());
    } else {
        // Copy the original DataTable.
        auto dt_copy = dt;

        // Create a copy of the input data and replicate it to feed multiple 
        // rows.
        auto data_copy = data;
        data_copy.insert(data_copy.end(), data.cbegin(), data.cend());
        data_copy.insert(data_copy.end(), data.cbegin(), data.cend());

        // Try adding multiple rows without providing number of columns.
        try {
            dt_copy.addRows(data_copy.cbegin(), data_copy.cend());
        } catch(OpenSim::InvalidEntry&) {}
    
        // Try adding multiple rows by providing number of columns to be zero.
        try {
            dt_copy.addRows(data_copy.cbegin(), data_copy.cend(), 0);
        } catch(OpenSim::InvalidEntry&) {}

        // Try adding rows with insufficient number of elements.
        try {
            dt_copy.addRows(data_copy.cbegin(), data_copy.cend() - 1);
        } catch(OpenSim::InvalidEntry&) {}
    
        // Add multiple rows at the same time.
        dt_copy.addRows(data_copy.cbegin(), data_copy.cend(), data.size());

        // Check DataTable size.
        checkDataTableLimits(dt_copy, 3, data.size());

        // Add more rows.
        dt_copy.addRows(data_copy.cbegin(), data_copy.cend());

        // Check DataTable size.
        checkDataTableLimits(dt_copy, 6, data.size());

        // Try adding rows with insufficient number of elements to fill up the 
        // last row.
        try {
            dt_copy.addRows(data_copy.cbegin(), data_copy.cend() - 1);
        } catch(OpenSim::NotEnoughElements&) {}

        // Add rows with insufficient number of elements but specify 
        // allow_missing.
        dt_copy.addRows(data_copy.cbegin(), data_copy.cend() - 1, 0, true);

        // Check DataTable size.
        checkDataTableLimits(dt_copy, 12, data.size());
    }

    // Add a row to the DataTable.
    SimTK::RowVector_<ET> dt_row{static_cast<int>(data.size()), data.data()};
    dt.addRow(dt_row);

    // Check the size of the DataTable.
    checkDataTableLimits(dt, orig_nrow + 1, data.size());

    // Check entries of row 0, which was just added.
    checkDataTableRow(dt, data, orig_nrow);

    // Add another row to the DataTable using iterators. This time add the data
    // in reverse order from the input vector.
    dt.addRow(data.crbegin(), data.crend());

    // Check the size of the DataTable.
    checkDataTableLimits(dt, orig_nrow + 2, data.size());

    // Check entries of row 1, which was just added.
    checkDataTableRow(dt, std::vector<ET>{data.crbegin(), data.crend()}, 
                      orig_nrow + 1);

    // Try adding empty row.
    try {
        dt.addRow(SimTK::RowVector_<ET>{});
    } catch(OpenSim::ZeroElements&) {}
    catch(OpenSim::NumberOfColumnsMismatch&) {}

    // Clear the DataTable.
    dt.clearData();

    // Check the size of the DataTable.
    checkDataTableLimits(dt, 0, 0);

    // Add the first row using iterators. Together, the previous invocations 
    // and this fully cover the code in addRow function.
    dt.addRow(data.cbegin(), data.cend());

    // Check the size of the DataTable.
    checkDataTableLimits(dt, 1, data.size());

    // Check entries of row 0, which was just added.
    checkDataTableRow(dt, data, 0);

    // Clear the DataTable.
    dt.clearData();

    // Check the size of the DataTable.
    checkDataTableLimits(dt, 0, 0);

    // Add the first row using iterators. This time provide a hint on how long
    // the row is.
    dt.addRow(data.cbegin(), data.cend(), data.size());

    // check the size of the DataTable.
    checkDataTableLimits(dt, 1, data.size());

    // Add another row to the DataTable using RowVector.
    dt.addRow(dt_row);

    // Check the size of the DataTable.
    checkDataTableLimits(dt, 2, data.size());

    // Check entries of row 1, which was just added.
    checkDataTableRow(dt, data, 1);

    // Resize the DataTable to 1 column smaller.
    dt.resizeKeep(2, data.size() -  1);

    // Check the size of the DataTable.
    checkDataTableLimits(dt, 2, data.size() - 1);

    // Check entries of row 1.
    checkDataTableRow(dt, decltype(data){data.cbegin(), data.cend() - 1}, 1);

    // Resize the DataTable to 1 column larger.
    dt.resizeKeep(2, data.size());

    // Check the size of the DataTable.
    checkDataTableLimits(dt, 2, data.size());
}


// Test adding cols to a populated DataTable.
template<typename ET>
void testAddCol(OpenSim::DataTable_<ET>& dt,
                const std::vector<ET>& data) {
    size_t orig_ncol{dt.getNumColumns()};

    if(dt.getNumColumns() > 0) {
        // Copy the original DataTable.
        auto dt_copy1 = dt;

        // Try adding a col with insufficient number of columns.
        try {
            dt_copy1.addColumn(data.cbegin(), data.cend() - 1);
        } catch(OpenSim::NotEnoughElements&) {}

        // Copy the original DataTable.
        auto dt_copy2 = dt;

        // Try adding a col with insufficient number of columns with 
        // allow_missing argument set to true.
        dt_copy2.addColumn(data.cbegin(), data.cend() - 1, 2, true);
    
        // Check the datatable size.
        checkDataTableLimits(dt_copy2, dt.getNumRows(), dt.getNumColumns() + 1);
    } else {
        // Copy the original DataTable.
        auto dt_copy = dt;

        // Create a copy of the input data and replicate it to feed multiple 
        // cols.
        auto data_copy = data;
        data_copy.insert(data_copy.end(), data.cbegin(), data.cend());
        data_copy.insert(data_copy.end(), data.cbegin(), data.cend());

        // Try adding multiple cols without providing number of columns.
        try {
            dt_copy.addColumns(data_copy.cbegin(), data_copy.cend());
        } catch(OpenSim::InvalidEntry&) {}
    
        // Try adding multiple cols by providing number of columns to be zero.
        try {
            dt_copy.addColumns(data_copy.cbegin(), data_copy.cend(), 0);
        } catch(OpenSim::InvalidEntry&) {}

        // Try adding cols with insufficient number of elements.
        try {
            dt_copy.addColumns(data_copy.cbegin(), data_copy.cend() - 1);
        } catch(OpenSim::InvalidEntry&) {}
    
        // Add multiple cols at the same time.
        dt_copy.addColumns(data_copy.cbegin(), data_copy.cend(), data.size());

        // Check DataTable size.
        checkDataTableLimits(dt_copy, data.size(), 3);

        // Add more cols.
        dt_copy.addColumns(data_copy.cbegin(), data_copy.cend());

        // Check DataTable size.
        checkDataTableLimits(dt_copy, data.size(), 6);

        // Try adding cols with insufficient number of elements to fill up the 
        // last col.
        try {
            dt_copy.addColumns(data_copy.cbegin(), data_copy.cend() - 1);
        } catch(OpenSim::InvalidEntry&) {}

        // Add cols with insufficient number of elements but specify 
        // allow_missing.
        dt_copy.addColumns(data_copy.cbegin(), data_copy.cend() - 1, 0, true);

        // Check DataTable size.
        checkDataTableLimits(dt_copy, data.size(), 12);
    }

    // Add a col to the DataTable using a SimTK::Vector.
    SimTK::Vector_<ET> dt_col{static_cast<int>(data.size()), data.data()};
    dt.addColumn(dt_col);

    // Check the size of the DataTable.
    checkDataTableLimits(dt, data.size(), orig_ncol + 1);

    // Check entries of row 0, which was just added.
    checkDataTableCol(dt, data, orig_ncol);

    // Add another col to the DataTable using iterators. This time add the data
    // in reverse order from the input vector.
    dt.addColumn(data.crbegin(), data.crend());

    // // Check the size of the DataTable.
    checkDataTableLimits(dt, data.size(), orig_ncol + 2);

    // // Check entries of the col just added.
    checkDataTableCol(dt, std::vector<ET>{data.crbegin(), data.crend()}, 
                      orig_ncol + 1);

    // Try adding emtpy col.
    try {
        dt.addColumn(SimTK::Vector_<ET>{});
    } catch(OpenSim::ZeroElements&) {}

    // Clear the DataTable.
    dt.clearData();

    // Check the size of the DataTable.
    checkDataTableLimits(dt, 0, 0);

    // Add the first col using iterators. Together, the previous invocations
    // and this fully cover the code in the addCol function. Using new_data 
    // that is a repeated version of the data.
    auto new_data = data;
    new_data.insert(new_data.end(), data.cbegin(), data.cend());
    dt.addColumn(new_data.cbegin(), new_data.cend());

    // Check the size of the DataTable.
    checkDataTableLimits(dt, new_data.size(), 1);

    // Clear the DataTable.
    dt.clearData();
  
    // Check the size of the DataTable.
    checkDataTableLimits(dt, 0, 0);

    // Add the first col using iterators. This time provide a hint on how long
    // the col is.
    dt.addColumn(new_data.cbegin(), new_data.cend(), new_data.size());

    // Check the size of the DataTable.
    checkDataTableLimits(dt, new_data.size(), 1);

    // Add another col to the DataTable using Vector.
    decltype(dt_col) new_dt_col{static_cast<int>(new_data.size()), 
            new_data.data()};
    dt.addColumn(new_dt_col);

    // Check the size of the DataTable.
    checkDataTableLimits(dt, new_data.size(), 2);

    // Check entries of the col just added.
    checkDataTableCol(dt, new_data, 1);

    // Resize the DataTable to 1 row smaller.
    dt.resizeKeep(new_data.size() - 1, 2);

    // Check the size of the DataTable.
    checkDataTableLimits(dt, new_data.size() - 1, 2);

    // Check entries of col 1.
    checkDataTableCol(dt, decltype(data){data.cbegin(), data.cend() - 1}, 1);

    // Resize the DataTable to 1 row larger.
    dt.resizeKeep(new_data.size(), 2);

    // Check the size of the DataTable.
    checkDataTableLimits(dt, new_data.size(), 2);
}


// Start with default constructed DataTable and add rows/cols to it.
void test1() {
    // Construct a DataTable of SimTK::Real(alias for double).
    std::cout << "test1 -- Default construct DataTable: Real." << std::endl;
    OpenSim::DataTable_<SimTK::Real> dt_real{};
    // Construct a DataTable of SimTK::Vec3.
    std::cout << "test1 -- Default construct DataTable: Vec3." << std::endl;
    OpenSim::DataTable_<SimTK::Vec3> dt_vec3{};
    // Construct a DataTable of SimTK::Vec6.
    std::cout << "test1 -- Default construct DataTable: Vec6." << std::endl;
    OpenSim::DataTable_<SimTK::Vec6> dt_vec6{};

    // Check the size of the DataTable.
    std::cout << "test1 -- checkDataTableLimits(): Real." << std::endl;
    checkDataTableLimits(dt_real, 0, 0);
    std::cout << "test1 -- checkDataTableLimits(): Vec3." << std::endl;
    checkDataTableLimits(dt_vec3, 0, 0);
    std::cout << "test1 -- checkDataTableLimits(): Vec6." << std::endl;
    checkDataTableLimits(dt_vec6, 0, 0);

    // Copy construct from default constructed DataTable.
    std::cout << "test1 -- Copy construct: Real." << std::endl;
    decltype(dt_real) copy_dt_real{dt_real};
    std::cout << "test1 -- Copy construct: Vec3." << std::endl;
    decltype(dt_vec3) copy_dt_vec3{dt_vec3};
    std::cout << "test1 -- Copy construct: Vec6." << std::endl;
    decltype(dt_vec6) copy_dt_vec6{dt_vec6};

    // Check the size of the DataTable.
    std::cout << "test1 -- checkDataTableLimits(): Real." << std::endl;
    checkDataTableLimits(copy_dt_real, 0, 0);
    std::cout << "test1 -- checkDataTableLimits(): Vec3." << std::endl;
    checkDataTableLimits(copy_dt_vec3, 0, 0);
    std::cout << "test1 -- checkDataTableLimits(): Vec6." << std::endl;
    checkDataTableLimits(copy_dt_vec6, 0, 0);

    // Virtual constructor.
    std::cout << "test1 -- Virtual(clone) constructor: Real." << std::endl;
    OpenSim::AbstractDataTable& abs_dt_real = dt_real;
    auto clone_absdt_real = abs_dt_real.clone();
    auto clone_dt_real = static_cast<decltype(dt_real)&>(*clone_absdt_real);
    std::cout << "test1 -- Virtual(clone) constructor: Vec3." << std::endl;
    OpenSim::AbstractDataTable& abs_dt_vec3 = dt_vec3;
    auto clone_absdt_vec3 = abs_dt_vec3.clone();
    auto clone_dt_vec3 = static_cast<decltype(dt_vec3)&>(*clone_absdt_vec3);
    std::cout << "test1 -- Virtual(clone) constructor: Vec6." << std::endl;
    OpenSim::AbstractDataTable& abs_dt_vec6 = dt_vec6;
    auto clone_absdt_vec6 = abs_dt_vec6.clone();
    auto clone_dt_vec6 = static_cast<decltype(dt_vec6)&>(*clone_absdt_vec6);

    // Check the size of the clone DataTable.
    std::cout << "test1 -- checkDataTableLimits(): Real." << std::endl;
    checkDataTableLimits(clone_dt_real, 0, 0);
    std::cout << "test1 -- checkDataTableLimits(): Vec3." << std::endl;
    checkDataTableLimits(clone_dt_vec3, 0, 0);
    std::cout << "test1 -- checkDataTableLimits(): Vec6." << std::endl;
    checkDataTableLimits(clone_dt_vec6, 0, 0);

    // Move constructors.
    std::cout << "test1 -- Move constructor: Real." << std::endl;
    decltype(dt_real) move_dt_real{std::move(dt_real)};
    std::cout << "test1 -- Move constructor: Vec3." << std::endl;
    decltype(dt_vec3) move_dt_vec3{std::move(dt_vec3)};
    std::cout << "test1 -- Move constructor: Vec6." << std::endl;
    decltype(dt_vec6) move_dt_vec6{std::move(dt_vec6)};

    // Check the size of the clone DataTable.
    std::cout << "test1 -- checkDataTableLimits(): Real." << std::endl;
    checkDataTableLimits(clone_dt_real, 0, 0);
    std::cout << "test1 -- checkDataTableLimits(): Vec3." << std::endl;
    checkDataTableLimits(clone_dt_vec3, 0, 0);
    std::cout << "test1 -- checkDataTableLimits(): Vec6." << std::endl;
    checkDataTableLimits(clone_dt_vec6, 0, 0);

    // Test adding rows to empty DataTable using a SimTK::RowVector. Using 
    // integers only for demostration. The underlying type can hold `double`.
    std::cout << "test1 -- testAddRow(): Real." << std::endl;
    testAddRow(dt_real, std::vector<SimTK::Real>{1, 2, 3});
    std::cout << "test1 -- testAddRow(): Vec3." << std::endl;
    testAddRow(dt_vec3, std::vector<SimTK::Vec3>{{1, 2, 3},
                                                 {4, 5, 6},
                                                 {7, 8, 9}});
    std::cout << "test1 -- testAddRow(): Vec6." << std::endl;
    testAddRow(dt_vec6, 
               std::vector<SimTK::Vec6>{{1, 2, 3, 11, 22, 33},
                                        {4, 5, 6, 44, 55, 66},
                                        {7, 8, 9, 77, 88, 99}});


    // Test adding cols to previously populated DataTable.
    std::cout << "test1 -- testAddCol(): Real." << std::endl;
    testAddCol(dt_real, std::vector<SimTK::Real>{1, 2});
    std::cout << "test1 -- testAddCol(): Vec3." << std::endl;
    testAddCol(dt_vec3, std::vector<SimTK::Vec3>{{1, 2, 3},
                                                 {4, 5, 6}});
    std::cout << "test1 -- testAddCol(): Vec6." << std::endl;
    testAddCol(dt_vec6, std::vector<SimTK::Vec6>{{1, 2, 3, 11, 22, 33},
                                                 {4, 5, 6, 44, 55, 66}});

    // Test adding rows to previously populated DataTable.
    std::cout << "test1 -- testAddRow(): Real." << std::endl;
    testAddRow(dt_real, std::vector<SimTK::Real>{1, 2});
    std::cout << "test1 -- testAddRow(): Vec3." << std::endl;
    testAddRow(dt_vec3, std::vector<SimTK::Vec3>{{1, 2, 3}, 
                                                 {4, 5, 6}});
    std::cout << "test1 -- testAddRow(): Vec6." << std::endl;
    testAddRow(dt_vec6, 
               std::vector<SimTK::Vec6>{{1, 2, 3, 11, 22, 33}, 
                                        {4, 5, 6, 44, 55, 66}});
}


// Start with DataTable constructed with constructor taking number of rows,
// cols and a default value for all the entries. Add rows/cols to the
// constructed DataTable.
void test2() {
    // Construct a DataTable of SimTK::Real(alias for double). Using an integers
    // for demonstration. The underlying type can hold double.
    std::cout << "test2 -- Construct DataTable with default value: Real." 
              << std::endl;
    OpenSim::DataTable_<SimTK::Real> dt_real{3, 4, 10};
    // Construct a DataTable of SimTK::Vec3.
    std::cout << "test2 -- Construct DataTable with default value: Vec3." 
              << std::endl;
    OpenSim::DataTable_<SimTK::Vec3> dt_vec3{3, 4, {10, 20, 30}};
    // Construct a DataTable of SimTK::Vec6.
    std::cout << "test2 -- Construct DataTable with default value: Vec6." 
              << std::endl; 
    OpenSim::DataTable_<SimTK::Vec6> dt_vec6{3, 4, {10, 20, 30, 40, 50, 60}};

    // Check the size of the DataTable.
    std::cout << "test2 -- checkDataTableLimits(): Real." << std::endl;
    checkDataTableLimits(dt_real, 3, 4);
    std::cout << "test2 -- checkDataTableLimits(): Vec3." << std::endl;
    checkDataTableLimits(dt_vec3, 3, 4);
    std::cout << "test2 -- checkDataTableLimits(): Vec6." << std::endl;
    checkDataTableLimits(dt_vec6, 3, 4);

    // Check the entries of few rows of the DataTable.
    std::cout << "test2 -- checkDataTableRow(row 0): Real." << std::endl;
    checkDataTableRow(dt_real, std::vector<SimTK::Real>{10, 10, 10, 10}, 0);
    std::cout << "test2 -- checkDataTableRow(row 2): Real." << std::endl;
    checkDataTableRow(dt_real, std::vector<SimTK::Real>{10, 10, 10, 10}, 2);
    std::cout << "test2 -- checkDataTableRow(row 0): Vec3." << std::endl;
    checkDataTableRow(dt_vec3, std::vector<SimTK::Vec3>{{10, 20, 30},
                                                        {10, 20, 30},
                                                        {10, 20, 30},
                                                        {10, 20, 30}}, 0);
    std::cout << "test2 -- checkDataTableRow(row 2): Vec3." << std::endl;
    checkDataTableRow(dt_vec3, std::vector<SimTK::Vec3>{{10, 20, 30},
                                                        {10, 20, 30},
                                                        {10, 20, 30},
                                                        {10, 20, 30}}, 2);
    std::cout << "test2 -- checkDataTableRow(row 0): Vec6." << std::endl;
    checkDataTableRow(dt_vec6, 
                      std::vector<SimTK::Vec6>{{10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60}}, 0);
    std::cout << "test2 -- checkDataTableRow(row 2): Vec6." << std::endl;
    checkDataTableRow(dt_vec6, 
                      std::vector<SimTK::Vec6>{{10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60}}, 2);

    // Copy construct from default constructed DataTable.
    std::cout << "test2 -- Copy construct: Real." << std::endl;
    decltype(dt_real) copy_dt_real{dt_real};
    std::cout << "test2 -- Copy construct: Vec3." << std::endl;
    decltype(dt_vec3) copy_dt_vec3{dt_vec3};
    std::cout << "test2 -- Copy construct: Vec6." << std::endl;
    decltype(dt_vec6) copy_dt_vec6{dt_vec6};

    // Check the size of the copy constructed DataTable.
    std::cout << "test2 -- checkDataTableLimits(): Real." << std::endl;
    checkDataTableLimits(copy_dt_real, 3, 4);
    std::cout << "test2 -- checkDataTableLimits(): Vec3." << std::endl;
    checkDataTableLimits(copy_dt_vec3, 3, 4);
    std::cout << "test2 -- checkDataTableLimits(): Vec6." << std::endl;
    checkDataTableLimits(copy_dt_vec6, 3, 4);

    // Check the entries of few rows of the copy constructed DataTable.
    std::cout << "test2 -- checkDataTableRow(row 0): Real." << std::endl;
    checkDataTableRow(copy_dt_real, 
                      std::vector<SimTK::Real>{10, 10, 10, 10}, 0);
    std::cout << "test2 -- checkDataTableRow(row 2): Real." << std::endl;
    checkDataTableRow(copy_dt_real, 
                      std::vector<SimTK::Real>{10, 10, 10, 10}, 2);
    std::cout << "test2 -- checkDataTableRow(row 0): Vec3." << std::endl;
    checkDataTableRow(copy_dt_vec3, std::vector<SimTK::Vec3>{{10, 20, 30},
                                                             {10, 20, 30},
                                                             {10, 20, 30},
                                                             {10, 20, 30}}, 0);
    std::cout << "test2 -- checkDataTableRow(row 2): Vec3." << std::endl;
    checkDataTableRow(copy_dt_vec3, std::vector<SimTK::Vec3>{{10, 20, 30},
                                                             {10, 20, 30},
                                                             {10, 20, 30},
                                                             {10, 20, 30}}, 2);
    std::cout << "test2 -- checkDataTableRow(row 0): Vec6." << std::endl;
    checkDataTableRow(copy_dt_vec6, 
                      std::vector<SimTK::Vec6>{{10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60}}, 0);
    std::cout << "test2 -- checkDataTableRow(row 2): Vec6." << std::endl;
    checkDataTableRow(copy_dt_vec6, 
                      std::vector<SimTK::Vec6>{{10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60}}, 2);

    // Virtual constructor.
    std::cout << "test2 -- Virtual(clone) constructor: Real." << std::endl;
    OpenSim::AbstractDataTable& abs_dt_real = dt_real;
    auto clone_absdt_real = abs_dt_real.clone();
    auto clone_dt_real = static_cast<decltype(dt_real)&>(*clone_absdt_real);
    std::cout << "test2 -- Virtual(clone) constructor: Vec3." << std::endl;
    OpenSim::AbstractDataTable& abs_dt_vec3 = dt_vec3;
    auto clone_absdt_vec3 = abs_dt_vec3.clone();
    auto clone_dt_vec3 = static_cast<decltype(dt_vec3)&>(*clone_absdt_vec3);
    std::cout << "test2 -- Virtual(clone) constructor: Vec6." << std::endl;
    OpenSim::AbstractDataTable& abs_dt_vec6 = dt_vec6;
    auto clone_absdt_vec6 = abs_dt_vec6.clone();
    auto clone_dt_vec6 = static_cast<decltype(dt_vec6)&>(*clone_absdt_vec6);

    // Check the size of the clone DataTable.
    std::cout << "test2 -- checkDataTableLimits(): Real." << std::endl;
    checkDataTableLimits(clone_dt_real, 3, 4);
    std::cout << "test2 -- checkDataTableLimits(): Vec3." << std::endl;
    checkDataTableLimits(clone_dt_vec3, 3, 4);
    std::cout << "test2 -- checkDataTableLimits(): Vec6." << std::endl;
    checkDataTableLimits(clone_dt_vec6, 3, 4);

    // Check the entries of few rows of the DataTable.
    std::cout << "test2 -- checkDataTableRow(row 0): Real." << std::endl;
    checkDataTableRow(clone_dt_real, 
                      std::vector<SimTK::Real>{10, 10, 10, 10}, 0);
    std::cout << "test2 -- checkDataTableRow(row 2): Real." << std::endl;
    checkDataTableRow(clone_dt_real, 
                      std::vector<SimTK::Real>{10, 10, 10, 10}, 2);
    std::cout << "test2 -- checkDataTableRow(row 0): Vec3." << std::endl;
    checkDataTableRow(clone_dt_vec3, std::vector<SimTK::Vec3>{{10, 20, 30},
                                                              {10, 20, 30},
                                                              {10, 20, 30},
                                                              {10, 20, 30}}, 0);
    std::cout << "test2 -- checkDataTableRow(row 2): Vec3." << std::endl;
    checkDataTableRow(clone_dt_vec3, std::vector<SimTK::Vec3>{{10, 20, 30},
                                                              {10, 20, 30},
                                                              {10, 20, 30},
                                                              {10, 20, 30}}, 2);
    std::cout << "test2 -- checkDataTableRow(row 0): Vec6." << std::endl;
    checkDataTableRow(clone_dt_vec6, 
                      std::vector<SimTK::Vec6>{{10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60}}, 0);
    std::cout << "test2 -- checkDataTableRow(row 2): Vec6." << std::endl;
    checkDataTableRow(clone_dt_vec6, 
                      std::vector<SimTK::Vec6>{{10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60}}, 2);

    // Move constructor.
    std::cout << "test2 -- Move constructor: Real." << std::endl;
    decltype(dt_real) move_dt_real{std::move(dt_real)};
    std::cout << "test2 -- Move constructor: Vec3." << std::endl;
    decltype(dt_vec3) move_dt_vec3{std::move(dt_vec3)};
    std::cout << "test2 -- Move constructor: Vec6." << std::endl;
    decltype(dt_vec6) move_dt_vec6{std::move(dt_vec6)};

    // Check the size of the DataTable created.
    std::cout << "test2 -- checkDataTableLimits(): Real." << std::endl;
    checkDataTableLimits(move_dt_real, 3, 4);
    std::cout << "test2 -- checkDataTableLimits(): Vec3." << std::endl;
    checkDataTableLimits(move_dt_vec3, 3, 4);
    std::cout << "test2 -- checkDataTableLimits(): Vec6." << std::endl;
    checkDataTableLimits(move_dt_vec6, 3, 4);

    // Check the entries of few rows of the DataTable.
    std::cout << "test2 -- checkDataTableRow(row 0): Real." << std::endl;
    checkDataTableRow(move_dt_real, 
                      std::vector<SimTK::Real>{10, 10, 10, 10}, 0);
    std::cout << "test2 -- checkDataTableRow(row 2): Real." << std::endl;
    checkDataTableRow(move_dt_real, 
                      std::vector<SimTK::Real>{10, 10, 10, 10}, 2);
    std::cout << "test2 -- checkDataTableRow(row 0): Vec3." << std::endl;
    checkDataTableRow(move_dt_vec3, std::vector<SimTK::Vec3>{{10, 20, 30},
                                                             {10, 20, 30},
                                                             {10, 20, 30},
                                                             {10, 20, 30}}, 0);
    std::cout << "test2 -- checkDataTableRow(row 2): Vec3." << std::endl;
    checkDataTableRow(move_dt_vec3, std::vector<SimTK::Vec3>{{10, 20, 30},
                                                             {10, 20, 30},
                                                             {10, 20, 30},     
                                                             {10, 20, 30}}, 2);
    std::cout << "test2 -- checkDataTableRow(row 0): Vec6." << std::endl;
    checkDataTableRow(move_dt_vec6, 
                      std::vector<SimTK::Vec6>{{10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60}}, 0);
    std::cout << "test2 -- checkDataTableRow(row 2): Vec6." << std::endl;
    checkDataTableRow(move_dt_vec6, 
                      std::vector<SimTK::Vec6>{{10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60},
                                               {10, 20, 30, 40, 50, 60}}, 2);

    // Test adding rows to the DataTable.
    std::cout << "test2 -- testAddRow(): Real." << std::endl;
    testAddRow(dt_real, std::vector<SimTK::Real>{1, 2, 3, 4});
    std::cout << "test2 -- testAddRow(): Vec3." << std::endl;
    testAddRow(dt_vec3, std::vector<SimTK::Vec3>{{ 1,  2,  3}, 
                                                 { 4,  5,  6},
                                                 { 7,  8,  9},
                                                 {10, 11, 12}});
    std::cout << "test2 -- testAddRow(): Vec6." << std::endl;
    testAddRow(dt_vec6, 
               std::vector<SimTK::Vec6>{{ 1,  2,  3,  11,  22,  33},
                                        { 4,  5,  6,  44,  55,  66},
                                        { 7,  8,  9,  77,  88,  99},
                                        {10, 11, 12, 110, 120, 140}});

    // Test adding cols to previously populated DataTable.
    std::cout << "test2 -- testAddCol(): Real." << std::endl;
    testAddCol(dt_real, std::vector<SimTK::Real>{1, 2});
    std::cout << "test2 -- testAddCol(): Vec3." << std::endl;
    testAddCol(dt_vec3, std::vector<SimTK::Vec3>{{1, 2, 3},
                                                 {4, 5, 6}});
    std::cout << "test2 -- testAddCol(): Vec6." << std::endl;
    testAddCol(dt_vec6, std::vector<SimTK::Vec6>{{1, 2, 3, 11, 22, 33},
                                                 {4, 5, 6, 44, 55, 66}});

    // Test adding rows to previously populated DataTable.
    std::cout << "test2 -- testAddRow(): Real." << std::endl;
    testAddRow(dt_real, std::vector<SimTK::Real>{1, 2});
    std::cout << "test2 -- testAddRow(): Vec3." << std::endl;
    testAddRow(dt_vec3, std::vector<SimTK::Vec3>{{1, 2, 3}, 
                                                 {4, 5, 6}});
    std::cout << "test2 -- testAddRow(): Vec6." << std::endl;
    testAddRow(dt_vec6, 
               std::vector<SimTK::Vec6>{{1, 2, 3, 11, 22, 33}, 
                                        {4, 5, 6, 44, 55, 66}});
}


// Start with DataTable constructed with constructor taking iterators,
// cols and a default value for all the entries. Add rows/cols to the
// constructed DataTable.
void test3() {
    std::vector<SimTK::Real> data_real{};
    for(int i = 0; i < 12; ++i)
        data_real.push_back(i);

    std::vector<SimTK::Vec3> data_vec3{};
    for(int i = 0; i < 12; ++i)
        data_vec3.push_back(SimTK::Vec3{1, 2, 3} + i);

    std::vector<SimTK::Vec6> data_vec6{};
    for(int i = 0; i < 12; ++i)
        data_vec6.push_back(SimTK::Vec6{1, 2, 3, 4, 5, 6} + i);

    {
        // Try constructing a DataTable iterators but providing insufficient 
        // number of elements to populate the whole table.
        std::cout << "test3 -- Construct DataTable with iterators rowwise"
                  << " [NotEnoughElements]: Real." << std::endl;
        try {
            OpenSim::DataTable_<SimTK::Real> dt_real{data_real.cbegin(), 
                                                     data_real.cend() - 2,
                                                     4};  
        } catch(OpenSim::NotEnoughElements&) {
            OpenSim::DataTable_<SimTK::Real> 
                dt_real{data_real.cbegin(), 
                        data_real.cend() - 2,
                        4, 
                        OpenSim::InputItDim::RowWise,
                        true};  
        }
        std::cout << "test3 -- Construct DataTable with iterators rowwise"
                  << " [NotEnoughElements]: Vec3." << std::endl;
        try {
            OpenSim::DataTable_<SimTK::Vec3> dt_vec3{data_vec3.cbegin(), 
                                                     data_vec3.cend() - 2,
                                                     4};  
        } catch(OpenSim::NotEnoughElements&) {
            OpenSim::DataTable_<SimTK::Vec3> 
                dt_vec3{data_vec3.cbegin(), 
                        data_vec3.cend() - 2,
                        4,
                        OpenSim::InputItDim::RowWise,
                        true};
        }
        std::cout << "test3 -- Construct DataTable with iterators rowwise"
                  << " [NotEnoughElements]: Vec6." << std::endl;
        try {
            OpenSim::DataTable_<SimTK::Vec6> dt_vec6{data_vec6.cbegin(), 
                                                     data_vec6.cend() - 2,
                                                     4};  
        } catch(OpenSim::NotEnoughElements&) {
            OpenSim::DataTable_<SimTK::Vec6> 
                dt_vec6{data_vec6.cbegin(), 
                        data_vec6.cend() - 2,
                        4,
                        OpenSim::InputItDim::RowWise,
                        true};
        }
    }
    {
        // Try constructing a DataTable iterators but providing insufficient 
        // number of elements to populate the whole table.
        std::cout << "test3 -- Construct DataTable with iterators colwise"
                  << " [NotEnoughElements]: Real." << std::endl;
        try {
            OpenSim::DataTable_<SimTK::Real> 
                dt_real{data_real.cbegin(), 
                        data_real.cend() - 2,
                        4,
                        OpenSim::InputItDim::ColumnWise};
        } catch(OpenSim::NotEnoughElements&) {
            OpenSim::DataTable_<SimTK::Real> 
                dt_real{data_real.cbegin(), 
                        data_real.cend() - 2,
                        4,
                        OpenSim::InputItDim::ColumnWise,
                        true};
        }
        std::cout << "test3 -- Construct DataTable with iterators colwise"
                  << " [NotEnoughElements]: Vec3." << std::endl;
        try {
            OpenSim::DataTable_<SimTK::Vec3> 
                dt_vec3{data_vec3.cbegin(), 
                        data_vec3.cend() - 2,
                        4,
                        OpenSim::InputItDim::ColumnWise};
        } catch(OpenSim::NotEnoughElements&) {
            OpenSim::DataTable_<SimTK::Vec3> 
                dt_vec3{data_vec3.cbegin(), 
                        data_vec3.cend() - 2,
                        4,
                        OpenSim::InputItDim::ColumnWise,
                        true};
        }
        std::cout << "test3 -- Construct DataTable with iterators colwise"
                  << " [NotEnoughElements]: Vec6." << std::endl;
        try {
            OpenSim::DataTable_<SimTK::Vec6> 
                dt_vec6{data_vec6.cbegin(), 
                        data_vec6.cend() - 2,
                        4,
                        OpenSim::InputItDim::ColumnWise};
        } catch(OpenSim::NotEnoughElements&) {
            OpenSim::DataTable_<SimTK::Vec6> 
                dt_vec6{data_vec6.cbegin(), 
                        data_vec6.cend() - 2,
                        4,
                        OpenSim::InputItDim::ColumnWise,
                        true};
        }
    }

    {
        // Construct a DataTable using iterators. Using integers for 
        // demostration. The underlyig type can hold double.
        std::cout << "test3 -- Construct DataTable with iterators colwise:" 
                  << " Real." << std::endl;
        OpenSim::DataTable_<SimTK::Real> 
            dt_real{data_real.cbegin(), 
                    data_real.cend(),
                    4,
                    OpenSim::InputItDim::ColumnWise};
        std::cout << "test3 -- Construct DataTable with iterators colwise:" 
                  <<" Vec3." << std::endl;
        OpenSim::DataTable_<SimTK::Vec3> 
            dt_vec3{data_vec3.cbegin(),
                    data_vec3.cend(),
                    4,
                    OpenSim::InputItDim::ColumnWise};
        std::cout << "test3 -- Construct DataTable with iterators colwise:" 
                  << " Vec6." << std::endl;
        OpenSim::DataTable_<SimTK::Vec6> 
            dt_vec6{data_vec6.cbegin(),
                    data_vec6.cend(), 
                    4,
                    OpenSim::InputItDim::ColumnWise};

        // Check the size of the DataTable.
        std::cout << "test3 -- checkDataTableLimits(): Real." << std::endl;
        checkDataTableLimits(dt_real, 4, 3);
        std::cout << "test3 -- checkDataTableLimits(): Vec3." << std::endl;
        checkDataTableLimits(dt_vec3, 4, 3);
        std::cout << "test3 -- checkDataTableLimits(): Vec6." << std::endl;
        checkDataTableLimits(dt_vec6, 4, 3);

        // Check the entries of few cows of the DataTable.
        std::cout << "test3 -- checkDataTableCol(col 0): Real." << std::endl;
        checkDataTableCol(dt_real, decltype(data_real){data_real.cbegin(),
                                                       data_real.cbegin() + 4}, 
                          0);
        std::cout << "test3 -- checkDataTableCol(col 2): Real." << std::endl;
        checkDataTableCol(dt_real, decltype(data_real){data_real.cend() - 4,
                                                       data_real.cend()}, 2);
        std::cout << "test3 -- checkDataTableCol(col 0): Vec3." << std::endl;
        checkDataTableCol(dt_vec3, decltype(data_vec3){data_vec3.cbegin(),
                                                       data_vec3.cbegin() + 4}, 
                          0);
        std::cout << "test3 -- checkDataTableCol(col 2): Vec3." << std::endl;
        checkDataTableCol(dt_vec3, decltype(data_vec3){data_vec3.cend() - 4,
                                                       data_vec3.cend()}, 2);
        std::cout << "test3 -- checkDataTableCol(col 0): Vec6." << std::endl;
        checkDataTableCol(dt_vec6, decltype(data_vec6){data_vec6.cbegin(),
                                                       data_vec6.cbegin() + 4}, 
                          0);
        std::cout << "test3 -- checkDataTableCol(col 2): Vec6." << std::endl;
        checkDataTableCol(dt_vec6, decltype(data_vec6){data_vec6.cend() - 4,
                                                       data_vec6.cend()}, 2);
    }

    // Construct a DataTable using iteratorsl. Using integers for demostration. 
    // The underlyig type can hold double.
    std::cout << "test3 -- Construct DataTable with iterators rowwise: Real."
              << std::endl;
    OpenSim::DataTable_<SimTK::Real> dt_real{data_real.cbegin(), 
                                             data_real.cend(),
                                             4};
    std::cout << "test3 -- Construct DataTable with iterators rowwise: Vec3."
              << std::endl;
    OpenSim::DataTable_<SimTK::Vec3> dt_vec3{data_vec3.cbegin(),
                                             data_vec3.cend(),
                                             4};
    std::cout << "test3 -- Construct DataTable with iterators rowwise: Vec6."
              << std::endl;
    OpenSim::DataTable_<SimTK::Vec6> dt_vec6{data_vec6.cbegin(),
                                             data_vec6.cend(), 
                                             4};

    // Check the size of the DataTable.
    std::cout << "test3 -- checkDataTableLimits(): Real." << std::endl;
    checkDataTableLimits(dt_real, 3, 4);
    std::cout << "test3 -- checkDataTableLimits(): Vec3." << std::endl;
    checkDataTableLimits(dt_vec3, 3, 4);
    std::cout << "test3 -- checkDataTableLimits(): Vec6." << std::endl;
    checkDataTableLimits(dt_vec6, 3, 4);

    // Check the entries of few rows of the DataTable.
    std::cout << "test3 -- checkDataTableRow(row 0): Real." << std::endl;
    checkDataTableRow(dt_real, decltype(data_real){data_real.cbegin(),
                                                   data_real.cbegin() + 4}, 0);
    std::cout << "test3 -- checkDataTableRow(row 2): Real." << std::endl;
    checkDataTableRow(dt_real, decltype(data_real){data_real.cend() - 4,
                                                   data_real.cend()}, 2);
    std::cout << "test3 -- checkDataTableRow(row 0): Vec3." << std::endl;
    checkDataTableRow(dt_vec3, decltype(data_vec3){data_vec3.cbegin(),
                                                   data_vec3.cbegin() + 4}, 0);
    std::cout << "test3 -- checkDataTableRow(row 2): Vec3." << std::endl;
    checkDataTableRow(dt_vec3, decltype(data_vec3){data_vec3.cend() - 4,
                                                   data_vec3.cend()}, 2);
    std::cout << "test3 -- checkDataTableRow(row 0): Vec6." << std::endl;
    checkDataTableRow(dt_vec6, decltype(data_vec6){data_vec6.cbegin(),
                                                   data_vec6.cbegin() + 4}, 0);
    std::cout << "test3 -- checkDataTableRow(row 2): Vec6." << std::endl;
    checkDataTableRow(dt_vec6, decltype(data_vec6){data_vec6.cend() - 4,
                                                   data_vec6.cend()}, 2);

  
    // Copy construct from default constructed DataTable.
    std::cout << "test3 -- Copy construct: Real." << std::endl;
    decltype(dt_real) copy_dt_real{dt_real};
    std::cout << "test3 -- Copy construct: Vec3." << std::endl;
    decltype(dt_vec3) copy_dt_vec3{dt_vec3};
    std::cout << "test3 -- Copy construct: Vec6." << std::endl;
    decltype(dt_vec6) copy_dt_vec6{dt_vec6};

    // Check the size of the DataTable.
    std::cout << "test3 -- checkDataTableLimits(): Real." << std::endl;
    checkDataTableLimits(dt_real, 3, 4);
    std::cout << "test3 -- checkDataTableLimits(): Vec3." << std::endl;
    checkDataTableLimits(dt_vec3, 3, 4);
    std::cout << "test3 -- checkDataTableLimits(): Vec6." << std::endl;
    checkDataTableLimits(dt_vec6, 3, 4);

    // Check the entries of few rows of the DataTable.
    std::cout << "test3 -- checkDataTableRow(row 0): Real." << std::endl;
    checkDataTableRow(copy_dt_real, 
                      decltype(data_real){data_real.cbegin(), 
                                          data_real.cbegin() + 4}, 0);
    std::cout << "test3 -- checkDataTableRow(row 2): Real." << std::endl;
    checkDataTableRow(copy_dt_real, 
                      decltype(data_real){data_real.cend() - 4,
                                          data_real.cend()}, 2);
    std::cout << "test3 -- checkDataTableRow(row 0): Vec3." << std::endl;
    checkDataTableRow(copy_dt_vec3, 
                      decltype(data_vec3){data_vec3.cbegin(),
                                          data_vec3.cbegin() + 4}, 0);
    std::cout << "test3 -- checkDataTableRow(row 2): Vec3." << std::endl;
    checkDataTableRow(copy_dt_vec3, 
                      decltype(data_vec3){data_vec3.cend() - 4,
                                          data_vec3.cend()}, 2);
    std::cout << "test3 -- checkDataTableRow(row 0): Vec6." << std::endl;
    checkDataTableRow(copy_dt_vec6, 
                      decltype(data_vec6){data_vec6.cbegin(),
                                          data_vec6.cbegin() + 4}, 0);
    std::cout << "test3 -- checkDataTableRow(row 2): Vec6." << std::endl;
    checkDataTableRow(copy_dt_vec6, 
                      decltype(data_vec6){data_vec6.cend() - 4,
                                          data_vec6.cend()}, 2);

    // Virtual constructor.
    std::cout << "test3 -- Virtual(clone) constructor: Real." << std::endl;
    OpenSim::AbstractDataTable& abs_dt_real = dt_real;
    auto clone_absdt_real = abs_dt_real.clone();
    auto clone_dt_real = static_cast<decltype(dt_real)&>(*clone_absdt_real);
    std::cout << "test3 -- Virtual(clone) constructor: Vec3." << std::endl;
    OpenSim::AbstractDataTable& abs_dt_vec3 = dt_vec3;
    auto clone_absdt_vec3 = abs_dt_vec3.clone();
    auto clone_dt_vec3 = static_cast<decltype(dt_vec3)&>(*clone_absdt_vec3);
    std::cout << "test3 -- Virtual(clone) constructor: Vec6." << std::endl;
    OpenSim::AbstractDataTable& abs_dt_vec6 = dt_vec6;
    auto clone_absdt_vec6 = abs_dt_vec6.clone();
    auto clone_dt_vec6 = static_cast<decltype(dt_vec6)&>(*clone_absdt_vec6);

    // Check the size of the DataTable.
    std::cout << "test3 -- checkDataTableLimits(): Real." << std::endl;
    checkDataTableLimits(dt_real, 3, 4);
    std::cout << "test3 -- checkDataTableLimits(): Vec3." << std::endl;
    checkDataTableLimits(dt_vec3, 3, 4);
    std::cout << "test3 -- checkDataTableLimits(): Vec6." << std::endl;
    checkDataTableLimits(dt_vec6, 3, 4);

    // Check the entries of few rows of the DataTable.
    std::cout << "test3 -- checkDataTableRow(row 0): Real." << std::endl;
    checkDataTableRow(clone_dt_real, 
                      decltype(data_real){data_real.cbegin(), 
                                          data_real.cbegin() + 4}, 0);
    std::cout << "test3 -- checkDataTableRow(row 2): Real." << std::endl;
    checkDataTableRow(clone_dt_real, 
                      decltype(data_real){data_real.cend() - 4,
                                          data_real.cend()}, 2);
    std::cout << "test3 -- checkDataTableRow(row 0): Vec3." << std::endl;
    checkDataTableRow(clone_dt_vec3, 
                      decltype(data_vec3){data_vec3.cbegin(),
                                          data_vec3.cbegin() + 4}, 0);
    std::cout << "test3 -- checkDataTableRow(row 2): Vec3." << std::endl;
    checkDataTableRow(clone_dt_vec3, 
                      decltype(data_vec3){data_vec3.cend() - 4,
                                          data_vec3.cend()}, 2);
    std::cout << "test3 -- checkDataTableRow(row 0): Vec6." << std::endl;
    checkDataTableRow(clone_dt_vec6, 
                      decltype(data_vec6){data_vec6.cbegin(),
                                          data_vec6.cbegin() + 4}, 0);
    std::cout << "test3 -- checkDataTableRow(row 2): Vec6." << std::endl;
    checkDataTableRow(clone_dt_vec6, 
                      decltype(data_vec6){data_vec6.cend() - 4,
                                          data_vec6.cend()}, 2);

    // Move constructor.
    std::cout << "test3 -- Move constructor: Real." << std::endl;
    decltype(dt_real) move_dt_real{std::move(dt_real)};
    std::cout << "test3 -- Move constructor: Vec3." << std::endl;
    decltype(dt_vec3) move_dt_vec3{std::move(dt_vec3)};
    std::cout << "test3 -- Move constructor: Vec6." << std::endl;
    decltype(dt_vec6) move_dt_vec6{std::move(dt_vec6)};

    // Check the size of the DataTable.
    std::cout << "test3 -- checkDataTableLimits(): Real." << std::endl;
    checkDataTableLimits(dt_real, 3, 4);
    std::cout << "test3 -- checkDataTableLimits(): Vec3." << std::endl;
    checkDataTableLimits(dt_vec3, 3, 4);
    std::cout << "test3 -- checkDataTableLimits(): Vec6." << std::endl;
    checkDataTableLimits(dt_vec6, 3, 4);

    // Check the entries of few rows of the DataTable.
    std::cout << "test3 -- checkDataTableRow(row 0): Real." << std::endl;
    checkDataTableRow(clone_dt_real, 
                      decltype(data_real){data_real.cbegin(), 
                                          data_real.cbegin() + 4}, 0);
    std::cout << "test3 -- checkDataTableRow(row 2): Real." << std::endl;
    checkDataTableRow(clone_dt_real, 
                      decltype(data_real){data_real.cend() - 4,
                                          data_real.cend()}, 2);
    std::cout << "test3 -- checkDataTableRow(row 0): Vec3." << std::endl;
    checkDataTableRow(clone_dt_vec3, 
                      decltype(data_vec3){data_vec3.cbegin(),
                                          data_vec3.cbegin() + 4}, 0);
    std::cout << "test3 -- checkDataTableRow(row 2): Vec3." << std::endl;
    checkDataTableRow(clone_dt_vec3, 
                      decltype(data_vec3){data_vec3.cend() - 4,
                                          data_vec3.cend()}, 2);
    std::cout << "test3 -- checkDataTableRow(row 0): Vec6." << std::endl;
    checkDataTableRow(clone_dt_vec6, 
                      decltype(data_vec6){data_vec6.cbegin(),
                                          data_vec6.cbegin() + 4}, 0);
    std::cout << "test3 -- checkDataTableRow(row 2): Vec6." << std::endl;
    checkDataTableRow(clone_dt_vec6, 
                      decltype(data_vec6){data_vec6.cend() - 4,
                                          data_vec6.cend()}, 2);

    // Test adding rows to the DataTable.
    std::cout << "test3 -- testAddRow(): Real." << std::endl;
    testAddRow(dt_real, std::vector<SimTK::Real>{1, 2, 3, 4});
    std::cout << "test3 -- testAddRow(): Vec3." << std::endl;
    testAddRow(dt_vec3, std::vector<SimTK::Vec3>{{ 1,  2,  3}, 
                                                 { 4,  5,  6},
                                                 { 7,  8,  9},
                                                 {10, 11, 12}});
    std::cout << "test3 -- testAddRow(): Vec6." << std::endl;
    testAddRow(dt_vec6, 
               std::vector<SimTK::Vec6>{{ 1,  2,  3,  11,  22,  33},
                                        { 4,  5,  6,  44,  55,  66},
                                        { 7,  8,  9,  77,  88,  99},
                                        {10, 11, 12, 110, 120, 140}});

    // Test adding cols to previously populated DataTable.
    std::cout << "test3 -- testAddCol(): Real." << std::endl;
    testAddCol(dt_real, std::vector<SimTK::Real>{1, 2});
    std::cout << "test3 -- testAddCol(): Vec3." << std::endl;
    testAddCol(dt_vec3, std::vector<SimTK::Vec3>{{1, 2, 3},
                                                 {4, 5, 6}});
    std::cout << "test3 -- testAddCol(): Vec6." << std::endl;
    testAddCol(dt_vec6, std::vector<SimTK::Vec6>{{1, 2, 3, 11, 22, 33},
                                                 {4, 5, 6, 44, 55, 66}});

    // Test adding rows to previously populated DataTable.
    std::cout << "test3 -- testAddRow(): Real." << std::endl;
    testAddRow(dt_real, std::vector<SimTK::Real>{1, 2});
    std::cout << "test3 -- testAddRow(): Vec3." << std::endl;
    testAddRow(dt_vec3, std::vector<SimTK::Vec3>{{1, 2, 3}, 
                                                 {4, 5, 6}});
    std::cout << "test3 -- testAddRow(): Vec6." << std::endl;
    testAddRow(dt_vec6, 
               std::vector<SimTK::Vec6>{{1, 2, 3, 11, 22, 33}, 
                                        {4, 5, 6, 44, 55, 66}});
}


// Test binding DataTables together.
void test4() {
    std::vector<SimTK::Real> data_real{};
    for(int i = 0; i < 12; ++i)
        data_real.push_back(SimTK::Real{100});

    std::vector<SimTK::Vec3> data_vec3{};
    for(int i = 0; i < 12; ++i)
        data_vec3.push_back(SimTK::Vec3{100, 200, 300});

    std::vector<SimTK::Vec6> data_vec6{};
    for(int i = 0; i < 12; ++i)
        data_vec6.push_back(SimTK::Vec6{100, 200, 300, 400, 500, 600});
  
    {
        // Construct DataTable 1.
        std::cout << "test4 -- Constructor DataTable 1: Real." << std::endl;
        OpenSim::DataTable_<SimTK::Real> dt1_real{3, 4, 10};
        std::cout << "test4 -- Constructor DataTable 1: Vec3." << std::endl;
        OpenSim::DataTable_<SimTK::Vec3> dt1_vec3{3, 4, {10, 20, 30}};
        std::cout << "test4 -- Constructor DataTable 1: Vec6." << std::endl;
        OpenSim::DataTable_<SimTK::Vec6> dt1_vec6{3, 4, {10, 20, 30, 
                                                         40, 50, 60}};

        // Construct DataTable 2 with SimtTK::Real
        std::cout << "test4 -- Construct DataTable 2: Real." << std::endl;
        OpenSim::DataTable_<SimTK::Real> 
            dt2_real{data_real.cbegin(), 
                     data_real.cend(),
                     3,
                     OpenSim::InputItDim::ColumnWise};
        std::cout << "test4 -- Construct DataTable 2: Vec3." << std::endl;
        OpenSim::DataTable_<SimTK::Vec3> 
            dt2_vec3{data_vec3.cbegin(),
                     data_vec3.cend(),
                     3,
                     OpenSim::InputItDim::ColumnWise};
        std::cout << "test4 -- Construct DataTable 2: Vec6." << std::endl;
        OpenSim::DataTable_<SimTK::Vec6> 
            dt2_vec6{data_vec6.cbegin(),
                     data_vec6.cend(), 
                     3,
                     OpenSim::InputItDim::ColumnWise};
  
        // Bind DataTable 2 to DataTable 1 by row.
        std::cout << "test4 -- dt1.concatenateRows(dt2): Real."  << std::endl;
        dt1_real.concatenateRows(dt2_real);
        std::cout << "test4 -- dt1.concatenateRows(dt2): Vec3." << std::endl;
        dt1_vec3.concatenateRows(dt2_vec3);
        std::cout << "test4 -- dt1.concatenateRows(dt2): Vec6." << std::endl;
        dt1_vec6.concatenateRows(dt2_vec6);


        // Check the size of the DataTable.
        std::cout << "test4 -- checkDataTableLimits(): Real." << std::endl;
        checkDataTableLimits(dt1_real, 6, 4);
        std::cout << "test4 -- checkDataTableLimits(): Vec3." << std::endl;
        checkDataTableLimits(dt1_vec3, 6, 4);
        std::cout << "test4 -- checkDataTableLimits(): Vec6." << std::endl;
        checkDataTableLimits(dt1_vec6, 6, 4);
 
        // Check the entries of the DataTable.
        std::cout << "test4 -- checkDataTableCol(col 0): Real." << std::endl;
        checkDataTableCol(dt1_real, 
                          decltype(data_real){10, 10, 10, 100, 100, 100}, 0);
        std::cout << "test4 -- checkDataTableCol(col 2): Real." << std::endl;
        checkDataTableCol(dt1_real, 
                          decltype(data_real){10, 10, 10, 100, 100, 100}, 2);
        std::cout << "test4 -- checkDataTableCol(col 0): Vec3." << std::endl;
        checkDataTableCol(dt1_vec3, decltype(data_vec3){{ 10,  20,  30},
                                                        { 10,  20,  30},
                                                        { 10,  20,  30},
                                                        {100, 200, 300},
                                                        {100, 200, 300},
                                                        {100, 200, 300}}, 0);
        std::cout << "test4 -- checkDataTableCol(col 2): Vec3." << std::endl;
        checkDataTableCol(dt1_vec3, decltype(data_vec3){{ 10,  20,  30},
                                                        { 10,  20,  30},
                                                        { 10,  20,  30},
                                                        {100, 200, 300},
                                                        {100, 200, 300},
                                                        {100, 200, 300}}, 2);
        std::cout << "test4 -- checkDataTableCol(col 0): Vec6." << std::endl;
        checkDataTableCol(dt1_vec6, 
                          decltype(data_vec6){{10, 20, 30, 40, 50, 60},
                                              {10, 20, 30, 40, 50, 60},
                                              {10, 20, 30, 40, 50, 60},
                                              {100, 200, 300, 400, 500, 600},
                                              {100, 200, 300, 400, 500, 600},
                                              {100, 200, 300, 400, 500, 600}}, 
                          0);
        std::cout << "test4 -- checkDataTableCol(col 2): Vec6." << std::endl;
        checkDataTableCol(dt1_vec6, 
                          decltype(data_vec6){{10, 20, 30, 40, 50, 60},
                                              {10, 20, 30, 40, 50, 60},
                                              {10, 20, 30, 40, 50, 60},
                                              {100, 200, 300, 400, 500, 600},
                                              {100, 200, 300, 400, 500, 600},
                                              {100, 200, 300, 400, 500, 600}}, 
                          2);

        // Try binding DataTable 1 to DataTable 2 by col.
        std::cout << "test4 -- dt2.concatenateColumns(dt1): Real." 
                  << std::endl;
        try {
            dt2_real.concatenateColumns(dt1_real);
        } catch(OpenSim::NumberOfRowsMismatch&) {}
        std::cout << "test4 -- dt2.concatenateColumns(dt1): Vec3." 
                  << std::endl;
        try {
            dt2_vec3.concatenateColumns(dt1_vec3);
        } catch(OpenSim::NumberOfRowsMismatch&) {}
        std::cout << "test4 -- dt2.concatenateColumns(dt1): Vec6." 
                  << std::endl;
        try {
            dt2_vec6.concatenateColumns(dt1_vec6);
        } catch(OpenSim::NumberOfRowsMismatch&) {}
    }

    // Construct DataTable 1.
    std::cout << "test4 -- Constructor DataTable 1: Real.\n";
    OpenSim::DataTable_<SimTK::Real> dt1_real{3, 4, 10};
    std::cout << "test4 -- Constructor DataTable 1: Vec3.\n";
    OpenSim::DataTable_<SimTK::Vec3> dt1_vec3{3, 4, {10, 20, 30}};
    std::cout << "test4 -- Constructor DataTable 1: Vec6.\n";
    OpenSim::DataTable_<SimTK::Vec6> dt1_vec6{3, 4, {10, 20, 30, 40, 50, 60}};

    // Construct DataTable 2 with SimtTK::Real
    std::cout << "test4 -- Construct DataTable 2: Real.\n";
    OpenSim::DataTable_<SimTK::Real> 
        dt2_real{data_real.cbegin(), 
                 data_real.cend(),
                 3,
                 OpenSim::InputItDim::ColumnWise};
    std::cout << "test4 -- Construct DataTable 2: Vec3.\n";
    OpenSim::DataTable_<SimTK::Vec3> 
        dt2_vec3{data_vec3.cbegin(),
                 data_vec3.cend(),
                 3,
                 OpenSim::InputItDim::ColumnWise};
    std::cout << "test4 -- Construct DataTable 2: Vec6.\n";
    OpenSim::DataTable_<SimTK::Vec6> 
        dt2_vec6{data_vec6.cbegin(),
                 data_vec6.cend(), 
                 3,
                 OpenSim::InputItDim::ColumnWise};
    // Bind DataTable 2 to DataTable 1 by col.
    std::cout << "test4 -- dt1.concatenateColumns(dt2): Real.\n";
    dt1_real.concatenateColumns(dt2_real);
    std::cout << "test4 -- dt1.concatenateColumns(dt2): Vec3.\n";
    dt1_vec3.concatenateColumns(dt2_vec3);
    std::cout << "test4 -- dt1.concatenateColumns(dt2): Vec6.\n";
    dt1_vec6.concatenateColumns(dt2_vec6);

    // Check the size of the DataTable.
    std::cout << "test4 -- checkDataTableLimits(): Real.\n";
    checkDataTableLimits(dt1_real, 3, 8);
    std::cout << "test4 -- checkDataTableLimits(): Vec3.\n";
    checkDataTableLimits(dt1_vec3, 3, 8);
    std::cout << "test4 -- checkDataTableLimits(): Vec6.\n";
    checkDataTableLimits(dt1_vec6, 3, 8);

    // Check the entries of the DataTable.
    std::cout << "test4 -- checkDataTableRow(row 0): Real.\n";
    checkDataTableRow(dt1_real, 
                      decltype(data_real){10, 10, 10, 10, 
                                          100, 100, 100, 100}, 0);
    std::cout << "test4 -- checkDataTableRow(row 2): Real.\n";
    checkDataTableRow(dt1_real, 
                      decltype(data_real){10, 10, 10, 10, 
                                          100, 100, 100, 100}, 2);
    std::cout << "test4 -- checkDataTableRow(row 0): Vec3.\n";
    checkDataTableRow(dt1_vec3, decltype(data_vec3){{ 10,  20,  30},
                                                    { 10,  20,  30},
                                                    { 10,  20,  30},
                                                    { 10,  20,  30},
                                                    {100, 200, 300},
                                                    {100, 200, 300},
                                                    {100, 200, 300},
                                                    {100, 200, 300}}, 0);
    std::cout << "test4 -- checkDataTableRow(row 2): Vec3.\n";
    checkDataTableRow(dt1_vec3, decltype(data_vec3){{ 10,  20,  30},
                                                    { 10,  20,  30},
                                                    { 10,  20,  30},
                                                    { 10,  20,  30},
                                                    {100, 200, 300},
                                                    {100, 200, 300},
                                                    {100, 200, 300},
                                                    {100, 200, 300}}, 2);
    std::cout << "test4 -- checkDataTableRow(row 0): Vec6.\n";
    checkDataTableRow(dt1_vec6, 
                      decltype(data_vec6){{10, 20, 30, 40, 50, 60},
                                          {10, 20, 30, 40, 50, 60},
                                          {10, 20, 30, 40, 50, 60},
                                          {10, 20, 30, 40, 50, 60},
                                          {100, 200, 300, 400, 500, 600},
                                          {100, 200, 300, 400, 500, 600},
                                          {100, 200, 300, 400, 500, 600},
                                          {100, 200, 300, 400, 500, 600}}, 0);
    std::cout << "test4 -- checkDataTableRow(row 2): Vec6.\n";
    checkDataTableRow(dt1_vec6, 
                      decltype(data_vec6){{10, 20, 30, 40, 50, 60},
                                          {10, 20, 30, 40, 50, 60},
                                          {10, 20, 30, 40, 50, 60},
                                          {10, 20, 30, 40, 50, 60},
                                          {100, 200, 300, 400, 500, 600},
                                          {100, 200, 300, 400, 500, 600},
                                          {100, 200, 300, 400, 500, 600},
                                          {100, 200, 300, 400, 500, 600}}, 2);

    // Try binding DataTable 1 to DataTable 2 by row.
    std::cout << "test4 -- dt2.concatenateRows(dt1): Real.\n";
    try {
        dt2_real.concatenateRows(dt1_real);
    } catch(OpenSim::NumberOfColumnsMismatch&) {}
    std::cout << "test4 -- dt2.concatenateRows(dt1): Vec3.\n";
    try {
        dt2_vec3.concatenateRows(dt1_vec3);
    } catch(OpenSim::NumberOfColumnsMismatch&) {}
    std::cout << "test4 -- dt2.concatenateRows(dt1): Vec6.\n";
    try {
        dt2_vec6.concatenateRows(dt1_vec6);
    } catch(OpenSim::NumberOfColumnsMismatch&) {}

    // Try binding DataTable 2 to itself by row.
    std::cout << "test4 -- dt2.concatenateRows(dt2): Real.\n";
    try {
        dt2_real.concatenateRows(dt2_real);
    } catch(OpenSim::InvalidEntry&) {}
    std::cout << "test4 -- dt2.concatenateRows(dt2): Vec3.\n";
    try {
        dt2_vec3.concatenateRows(dt2_vec3);
    } catch(OpenSim::InvalidEntry&) {}
    std::cout << "test4 -- dt2.concatenateRows(dt2): Vec6.\n";
    try {
        dt2_vec6.concatenateRows(dt2_vec6);
    } catch(OpenSim::InvalidEntry&) {}

    // Try binding DataTable 2 to itself by col.
    std::cout << "test4 -- dt2.concatenateColumns(dt2): Real.\n";
    try {
        dt2_real.concatenateColumns(dt2_real);
    } catch(OpenSim::InvalidEntry&) {}
    std::cout << "test4 -- dt2.concatenateColumns(dt2): Vec3.\n";
    try {
        dt2_vec3.concatenateColumns(dt2_vec3);
    } catch(OpenSim::InvalidEntry&) {}
    std::cout << "test4 -- dt2.concatenateColumns(dt2): Vec6.\n";
    try {
        dt2_vec6.concatenateColumns(dt2_vec6);
    } catch(OpenSim::InvalidEntry&) {}
}


// Compute the number of elements produced by an iterator pair.
template<typename Container>
size_t numElems(const Container& container) {
    size_t numelems{0};
    auto first = container.begin();
    auto last = container.end();
    while(first != last) {
        ++numelems;
        ++first;
    }
    return numelems;
}


// Test colum label interface.
void test5() {
    // Construct DataTable.
    std::cout << "test5 -- Constructor DataTable 1: Real.\n";
    OpenSim::DataTable_<SimTK::Real> dt_real{3, 4, 10};
    std::cout << "test5 -- Constructor DataTable 1: Vec3.\n";
    OpenSim::DataTable_<SimTK::Vec3> dt_vec3{3, 4, {10, 20, 30}};
    std::cout << "test5 -- Constructor DataTable 1: Vec6.\n";
    OpenSim::DataTable_<SimTK::Vec6> dt_vec6{3, 4, {10, 20, 30, 40, 50, 60}};

    // Check if column labels exist.
    std::cout << "test5 -- columnHasLabel(): Real.\n";
    for(size_t i = 0; i < dt_real.getNumColumns(); ++i)
        assert(dt_real.columnHasLabel(i) == false);
    std::cout << "test5 -- columnHasLabel(): Vec3.\n";
    for(size_t i = 0; i < dt_vec3.getNumColumns(); ++i)
        assert(dt_vec3.columnHasLabel(i) == false);
    std::cout << "test5 -- columnHaslabel(): Vec6.\n";
    for(size_t i = 0; i < dt_vec6.getNumColumns(); ++i)
        assert(dt_vec6.columnHasLabel(i) == false);

    // Try checking for columns that don't exist.
    std::cout << "test5 -- columnHasLabel()[ColumnDoesNotExist]: Real.\n";
    try {
        bool ans = dt_real.columnHasLabel(5); ignore(ans);
    } catch(OpenSim::ColumnDoesNotExist&) {}
    std::cout << "test5 -- columnHasLabel()[ColumnDoesNotExist]: Vec3.\n";
    try {
        bool ans = dt_vec3.columnHasLabel(5); ignore(ans);
    } catch(OpenSim::ColumnDoesNotExist&) {}
    std::cout << "test5 -- columnHasLabel()[ColumnDoesNotExist]: Vec6.\n";
    try {
        bool ans = dt_vec6.columnHasLabel(5); ignore(ans);
    } catch(OpenSim::ColumnDoesNotExist&) {}

    // Try checking for column that does not exist.
    std::cout << "test5 -- hasColumn(key): Real.\n";
    assert(dt_real.hasColumn("no-such-column") == false);
    std::cout << "test5 -- hasColumn(key): Vec3.\n";
    assert(dt_vec3.hasColumn("no-such-column") == false);
    std::cout << "test5 -- hasColumn(key): Vec6.\n";
    assert(dt_vec6.hasColumn("no-such-column") == false);

    // Get column label by index. No column should have a label at this point.
    std::cout << "test5 -- getColumnLabel()[ColumnHasNoLabel]: Real.\n";
    for(size_t i = 0; i < dt_real.getNumColumns(); ++i)
        try {
            auto label = dt_real.getColumnLabel(i); ignore(label);
        } catch(OpenSim::ColumnHasNoLabel&) {}
    std::cout << "test5 -- getColumnLabel()[ColumnHasNoLabel]: Vec3.\n";
    for(size_t i = 0; i < dt_vec3.getNumColumns(); ++i)
        try {
            auto label = dt_vec3.getColumnLabel(i); ignore(label);
        } catch(OpenSim::ColumnHasNoLabel&) {}
    std::cout << "test5 -- getColumnLabel()[ColumnHasNoLabel]: Vec6.\n";
    for(size_t i = 0; i < dt_vec6.getNumColumns(); ++i)
        try {
            auto label = dt_vec6.getColumnLabel(i); ignore(label);
        } catch(OpenSim::ColumnHasNoLabel&) {}

    // Get all column labels. There should be no column labels.
    std::cout << "test5 -- getcolumnLabels(): Real.\n";
    auto labels = dt_real.getColumnLabels(); 
    assert(labels.begin() == labels.end());
    std::cout << "test5 -- getcolumnLabels(): Vec3.\n";
    labels = dt_vec3.getColumnLabels();
    assert(labels.begin() == labels.end());
    std::cout << "test5 -- getcolumnLabels(): Vec6.\n";
    labels = dt_vec6.getColumnLabels();
    assert(labels.begin() == labels.end());

    // Get index of a column label that does not exist.
    std::cout << "test5 -- getColumnIndex()[ColumnDoesNotExist]: Real.\n";
    try {
        auto ind = dt_real.getColumnIndex("no-such-column"); ignore(ind);
    } catch(OpenSim::ColumnDoesNotExist&) {}
    std::cout << "test5 -- getColumnIndex()[ColumnDoesNotExist]: Vec3.\n";
    try {
        auto ind = dt_vec3.getColumnIndex("no-such-column"); ignore(ind);
    } catch(OpenSim::ColumnDoesNotExist&) {}
    std::cout << "test5 -- getColumnIndex()[ColumnDoesNotExist]: Vec6.\n";
    try {
        auto ind = dt_vec6.getColumnIndex("no-such-column"); ignore(ind);
    } catch(OpenSim::ColumnDoesNotExist&) {}

    // Try inserting column labels to columns that do not exist.
    std::cout << "test5 -- setColumnLabel()[ColumnDoesNotExist]: Real.\n";
    try {
        dt_real.setColumnLabel(4, "col-does-not-exist");
    } catch(OpenSim::ColumnDoesNotExist&) {}
    std::cout << "test5 -- setColumnLabel()[ColumnDoesNotExist]: Vec3.\n";
    try {
        dt_vec3.setColumnLabel(4, "col-does-not-exist");
    } catch(OpenSim::ColumnDoesNotExist&) {}
    std::cout << "test5 -- setColumnLabel()[ColumnDoesNotExist]: Vec6.\n";
    try {
        dt_vec6.setColumnLabel(4, "col-does-not-exist");
    } catch(OpenSim::ColumnDoesNotExist&) {}

    // Try editing column label for a column that does not exist.
    std::cout << "test5 -- changeColumnLabel()[ColumnDoesNotExist]: Real.\n";
    try {
        dt_real.changeColumnLabel(4, "ColFour");
    } catch(OpenSim::ColumnDoesNotExist&) {}
    std::cout << "test5 -- changeColumnLabel()[ColumnDoesNotExist]: Vec3.\n";
    try {
        dt_vec3.changeColumnLabel(4, "ColFour");
    } catch(OpenSim::ColumnDoesNotExist&) {}
    std::cout << "test5 -- changeColumnLabel()[ColumnDoesNotExist]: Vec6.\n";
    try {
        dt_vec6.changeColumnLabel(4, "ColFour");
    } catch(OpenSim::ColumnDoesNotExist&) {}

    // Try editing column label for a column that does not have a label yet.
    std::cout << "test5 -- changeColumnLabel()[ColumnHasNoLabel]: Real.\n";
    try {
        dt_real.changeColumnLabel(0, "ColZero");
    } catch(OpenSim::ColumnHasNoLabel&) {}
    std::cout << "test5 -- changeColumnLabel()[ColumnHasNoLabel]: Vec3.\n";
    try {
        dt_vec3.changeColumnLabel(0, "ColZero");
    } catch(OpenSim::ColumnHasNoLabel&) {}
    std::cout << "test5 -- changeColumnLabel()[ColumnHasNoLabel]: Vec6.\n";
    try {
        dt_vec6.changeColumnLabel(0, "ColZero");
    } catch(OpenSim::ColumnHasNoLabel&) {}
  
    // Insert some column labels.
    std::cout << "test5 -- setColumnLabel(): Real.\n";
    std::string collabel{"ColZero"};
    dt_real.setColumnLabel(0, collabel);
    dt_real.setColumnLabel(2, "ColTwo");
    std::cout << "test5 -- setColumnLabel(): Vec3.\n";
    dt_vec3.setColumnLabel(0, collabel);
    dt_vec3.setColumnLabel(2, "ColTwo");
    std::cout << "test5 -- setColumnLabel(): Vec6.\n";
    dt_vec6.setColumnLabel(0, collabel);
    dt_vec6.setColumnLabel(2, "ColTwo");

    // Insert more column labels.
    std::vector<std::pair<std::string, size_t>> col_data{{"ColOne", 1}, 
                                                         {"ColThree", 3}};
    std::cout << "test5 -- setColumnLabels(): Real.\n";
    dt_real.setColumnLabels(col_data.cbegin(), col_data.cend());
    std::cout << "test5 -- setColumnLabels(): Vec3.\n";
    dt_vec3.setColumnLabels(col_data.cbegin(), col_data.cend());
    std::cout << "test5 -- setColumnLabels(): Vec6.\n";
    dt_vec6.setColumnLabels(col_data.cbegin(), col_data.cend());

    // Check the number of column labels.
    std::cout << "test5 -- Number of column labels: Real.\n";
    assert(numElems(dt_real.getColumnLabels()) == 4);
    std::cout << "test5 -- Number of column labels: Vec3.\n";
    assert(numElems(dt_vec3.getColumnLabels()) == 4);
    std::cout << "test5 -- Number of column labels: Vec6.\n";
    assert(numElems(dt_vec6.getColumnLabels()) == 4);

    // Check existence of column labels.
    std::cout << "test5 -- columnHasLabel(): Real.\n";
    for(size_t i = 0; i < 4; ++i)
        assert(dt_real.columnHasLabel(i) == true);
    std::cout << "test5 -- columnHasLabel(): Vec3.\n";
    for(size_t i = 0; i < 4; ++i)
        assert(dt_vec3.columnHasLabel(i) == true);
    std::cout << "test5 -- columnHasLabel(): Vec6.\n";
    for(size_t i = 0; i < 4; ++i)
        assert(dt_vec6.columnHasLabel(i) == true);

    // Check the column labels.
    std::cout << "test5 -- getColumnLabel(): Real.\n";
    assert(dt_real.getColumnLabel(0) == "ColZero");
    assert(dt_real.getColumnLabel(1) == "ColOne");
    assert(dt_real.getColumnLabel(2) == "ColTwo");
    assert(dt_real.getColumnLabel(3) == "ColThree");
    std::cout << "test5 -- getColumnLabel(): Vec3.\n";
    assert(dt_vec3.getColumnLabel(0) == "ColZero");
    assert(dt_vec3.getColumnLabel(1) == "ColOne");
    assert(dt_vec3.getColumnLabel(2) == "ColTwo");
    assert(dt_vec3.getColumnLabel(3) == "ColThree");
    std::cout << "test5 -- getColumnLabel(): Vec6.\n";
    assert(dt_vec6.getColumnLabel(0) == "ColZero");
    assert(dt_vec6.getColumnLabel(1) == "ColOne");
    assert(dt_vec6.getColumnLabel(2) == "ColTwo");
    assert(dt_vec6.getColumnLabel(3) == "ColThree");

    // Check column index against label.
    std::cout << "test5 -- getColumnIndex(): Real.\n";
    assert(dt_real.getColumnIndex("ColZero")  == 0);
    assert(dt_real.getColumnIndex("ColOne")   == 1);
    assert(dt_real.getColumnIndex("ColTwo")   == 2);
    assert(dt_real.getColumnIndex("ColThree") == 3);
    std::cout << "test5 -- getColumnIndex(): Vec3.\n";
    assert(dt_vec3.getColumnIndex("ColZero")  == 0);
    assert(dt_vec3.getColumnIndex("ColOne")   == 1);
    assert(dt_vec3.getColumnIndex("ColTwo")   == 2);
    assert(dt_vec3.getColumnIndex("ColThree") == 3);
    std::cout << "test5 -- getColumnIndex(): Vec6.\n";
    assert(dt_vec6.getColumnIndex("ColZero")  == 0);
    assert(dt_vec6.getColumnIndex("ColOne")   == 1);
    assert(dt_vec6.getColumnIndex("ColTwo")   == 2);
    assert(dt_vec6.getColumnIndex("ColThree") == 3);

    // Try inserting labels for columns that don't exist.
    std::cout << "test5 -- setColumnLabel()[ColumnDoesNotExist]: Real.\n";
    try {
        dt_real.setColumnLabel(4, "col-does-not-exist");
    } catch(OpenSim::ColumnDoesNotExist&) {}
    std::cout << "test5 -- setColumnLabel()[ColumnDoesNotExist]: Vec3.\n";
    try {
        dt_vec3.setColumnLabel(4, "col-does-not-exist");
    } catch(OpenSim::ColumnDoesNotExist&) {}
    std::cout << "test5 -- setColumnLabel()[ColumnDoesNotExist]: Vec6.\n";
    try {
        dt_vec6.setColumnLabel(4, "col-does-not-exist");
    } catch(OpenSim::ColumnDoesNotExist&) {}

    // Try inserting lables for columns that already have labels.
    std::cout << "test5 -- setColumnLabel()[ColumnHasLabel]: Real.\n";
    try{
        dt_real.setColumnLabel(0, "col-has-label");
    } catch(OpenSim::ColumnHasLabel&) {}
    std::cout << "test5 -- setColumnLabel()[ColumnHasLabel]: Vec3.\n";
    try{
        dt_vec3.setColumnLabel(0, "col-has-label");
    } catch(OpenSim::ColumnHasLabel&) {}
    std::cout << "test5 -- setColumnLabel()[ColumnHasLabel]: Vec6.\n";
    try{
        dt_vec6.setColumnLabel(0, "col-has-label");
    } catch(OpenSim::ColumnHasLabel&) {}

    // Try inserting labels to for columns that don't exist. This time use
    // iterators.
    std::vector<std::pair<std::string, size_t>> col_data1{{"ColFour", 4},
                                                          {"ColFive", 5}};
    std::cout << "test5 -- setColumnLabels()[ColumnDoesNotExist]: Real.\n";
    try {
        dt_real.setColumnLabels(col_data1.cbegin(), col_data1.cend());
    } catch(OpenSim::ColumnDoesNotExist&) {}
    std::cout << "test5 -- setColumnLabels()[ColumnDoesNotExist]: Vec3.\n";
    try {
        dt_vec3.setColumnLabels(col_data1.cbegin(), col_data1.cend());
    } catch(OpenSim::ColumnDoesNotExist&) {}
    std::cout << "test5 -- setColumnLabels()[ColumnDoesNotExist]: Vec6.\n";
    try {
        dt_vec6.setColumnLabels(col_data1.cbegin(), col_data1.cend());
    } catch(OpenSim::ColumnDoesNotExist&) {}

    // Try inserting labels for columns that already have labels. This time use
    // iterators.
    std::cout << "test5 -- setColumnLabels()[ColumnHasLabel]: Real.\n";
    try {
        dt_real.setColumnLabels(col_data.cbegin(), col_data.cend());
    } catch(OpenSim::ColumnHasLabel&) {}
    std::cout << "test5 -- setColumnLabels()[ColumnHasLabel]: Vec3.\n";
    try {
        dt_vec3.setColumnLabels(col_data.cbegin(), col_data.cend());
    } catch(OpenSim::ColumnHasLabel&) {}
    std::cout << "test5 -- setColumnLabels()[ColumnHasLabel]: Vec6.\n";
    try {
        dt_vec6.setColumnLabels(col_data.cbegin(), col_data.cend());
    } catch(OpenSim::ColumnHasLabel&) {}

    // Try updating column label of a column that does not exist using its 
    // index.
    std::cout << "test5 -- changeColumnLabel(ind)[ColumnDoesNotExist]: Real.\n";
    try {
        dt_real.changeColumnLabel(4, "col-does-not-exist");
    } catch(OpenSim::ColumnDoesNotExist&) {}
    std::cout << "test5 -- changeColumnLabel(ind)[ColumnDoesNotExist]: Vec3.\n";
    try {
        dt_vec3.changeColumnLabel(4, "col-does-not-exist");
    } catch(OpenSim::ColumnDoesNotExist&) {}
    std::cout << "test5 -- changeColumnLabel(ind)[ColumnDoesNotExist]: Vec6.\n";
    try {
        dt_vec6.changeColumnLabel(4, "col-does-not-exist");
    } catch(OpenSim::ColumnDoesNotExist&) {}

    // Try updating column label of a column that does not exist using its 
    // label.
    std::cout << "test5 -- changeColumnLabel(label)[ColumnDoesNotExist]: Real.\n";
    try {
        dt_real.changeColumnLabel("col-does-not-exist", "foo");
    } catch(OpenSim::ColumnDoesNotExist&) {}
    std::cout << "test5 -- changeColumnLabel(label)[ColumnDoesNotExist]: Vec3.\n";
    try {
        dt_vec3.changeColumnLabel("col-does-not-exist", "foo");
    } catch(OpenSim::ColumnDoesNotExist&) {}
    std::cout << "test5 -- changeColumnLabel(label)[ColumnDoesNotExist]: Vec6.\n";
    try {
        dt_vec6.changeColumnLabel("col-does-not-exist", "foo");
    } catch(OpenSim::ColumnDoesNotExist&) {}

    // Update column label using column index.
    std::cout << "test5 -- changeColumnLabel(ind): Real.\n";
    dt_real.changeColumnLabel(0, "updColZero");
    std::cout << "test5 -- changeColumnLabel(ind): Vec3.\n";
    dt_vec3.changeColumnLabel(0, "updColZero");
    std::cout << "test5 -- changeColumnLabel(ind): Vec6.\n";
    dt_vec6.changeColumnLabel(0, "updColZero");

    // Update column label using old column label.
    std::cout << "test5 -- changeColumnLabel(label): Real.\n";
    dt_real.changeColumnLabel("updColZero", "ColZero");
    std::cout << "test5 -- changeColumnLabel(label): Vec3.\n";
    dt_vec3.changeColumnLabel("updColZero", "ColZero");
    std::cout << "test5 -- changeColumnLabel(label): Vec6.\n";
    dt_vec6.changeColumnLabel("updColZero", "ColZero");
  
    // Check the number of column labels.
    std::cout << "test5 -- Number of column labels: Real.\n";
    assert(numElems(dt_real.getColumnLabels()) == 4);
    std::cout << "test5 -- Number of column labels: Vec3.\n";
    assert(numElems(dt_vec3.getColumnLabels()) == 4);
    std::cout << "test5 -- Number of column labels: Vec6.\n";
    assert(numElems(dt_vec6.getColumnLabels()) == 4);

    // Check existence of column labels.
    std::cout << "test5 -- columnHasLabel(): Real.\n";
    for(size_t i = 0; i < 4; ++i)
        assert(dt_real.columnHasLabel(i) == true);
    std::cout << "test5 -- colunHasLabel(): Vec3.\n";
    for(size_t i = 0; i < 4; ++i)
        assert(dt_vec3.columnHasLabel(i) == true);
    std::cout << "test5 -- columnHasLabel(): Vec6.\n";
    for(size_t i = 0; i < 4; ++i)
        assert(dt_vec6.columnHasLabel(i) == true);

    // Check the column labels.
    std::cout << "test5 -- getColumnLabel(): Real.\n";
    assert(dt_real.getColumnLabel(0) == "ColZero");
    assert(dt_real.getColumnLabel(1) == "ColOne");
    assert(dt_real.getColumnLabel(2) == "ColTwo");
    assert(dt_real.getColumnLabel(3) == "ColThree");
    std::cout << "test5 -- getColumnLabel(): Vec3.\n";
    assert(dt_vec3.getColumnLabel(0) == "ColZero");
    assert(dt_vec3.getColumnLabel(1) == "ColOne");
    assert(dt_vec3.getColumnLabel(2) == "ColTwo");
    assert(dt_vec3.getColumnLabel(3) == "ColThree");
    std::cout << "test5 -- getColumnLabel(): Vec6.\n";
    assert(dt_vec6.getColumnLabel(0) == "ColZero");
    assert(dt_vec6.getColumnLabel(1) == "ColOne");
    assert(dt_vec6.getColumnLabel(2) == "ColTwo");
    assert(dt_vec6.getColumnLabel(3) == "ColThree");

    // Check column index against label.
    std::cout << "test5 -- getColumnIndex(): Real.\n";
    assert(dt_real.getColumnIndex("ColZero")  == 0);
    assert(dt_real.getColumnIndex("ColOne")   == 1);
    assert(dt_real.getColumnIndex("ColTwo")   == 2);
    assert(dt_real.getColumnIndex("ColThree") == 3);
    std::cout << "test5 -- getColumnIndex(): Vec3.\n";
    assert(dt_vec3.getColumnIndex("ColZero")  == 0);
    assert(dt_vec3.getColumnIndex("ColOne")   == 1);
    assert(dt_vec3.getColumnIndex("ColTwo")   == 2);
    assert(dt_vec3.getColumnIndex("ColThree") == 3);
    std::cout << "test5 -- getColumnIndex(): Vec6.\n";
    assert(dt_vec6.getColumnIndex("ColZero")  == 0);
    assert(dt_vec6.getColumnIndex("ColOne")   == 1);
    assert(dt_vec6.getColumnIndex("ColTwo")   == 2);
    assert(dt_vec6.getColumnIndex("ColThree") == 3);

    // Clear the column labels.
    std::cout << "test5 -- clearColumnLabels(): Real.\n";
    dt_real.clearColumnLabels();
    std::cout << "test5 -- clearColumnLabels(): Vec3.\n";
    dt_vec3.clearColumnLabels();
    std::cout << "test5 -- clearColumnLabels(): Vec6.\n";
    dt_vec6.clearColumnLabels();
  
    // Check the number of column labels.
    std::cout << "test5 -- Number of column labels: Real.\n";
    assert(numElems(dt_real.getColumnLabels()) == 0);
    std::cout << "test5 -- Number of column labels: Vec3.\n";
    assert(numElems(dt_vec3.getColumnLabels()) == 0);
    std::cout << "test5 -- Number of column labels: Vec6.\n";
    assert(numElems(dt_vec6.getColumnLabels()) == 0);
}


void test6() {
    // Construct DataTable.
    std::cout << "test6 -- Constructor DataTable 1: Real.\n";
    OpenSim::DataTable_<SimTK::Real> dt_real{3, 4, 10};
    std::cout << "test6 -- Constructor DataTable 1: Vec3.\n";
    OpenSim::DataTable_<SimTK::Vec3> dt_vec3{3, 4, {10, 20, 30}};
    std::cout << "test6 -- Constructor DataTable 1: Vec6.\n";
    OpenSim::DataTable_<SimTK::Vec6> dt_vec6{3, 4, {10, 20, 30, 40, 50, 60}};

    // Check metadata size in the DataTable.
    std::cout << "test6 -- getMetaDataSize(): Real.\n";
    assert(dt_real.getMetaDataSize() == 0);
    std::cout << "test6 -- getMetaDataSize(): Vec3.\n";
    assert(dt_vec3.getMetaDataSize() == 0);
    std::cout << "test6 -- getMetaDataSize(): Vec6.\n";
    assert(dt_vec6.getMetaDataSize() == 0);

    // Check if metadata is empty in the DataTable.
    std::cout << "test6 -- isMetaDataEmpty(): Real.\n";
    assert(dt_real.isMetaDataEmpty() == true);
    std::cout << "test6 -- isMetaDataEmpty(): Vec3.\n";
    assert(dt_vec3.isMetaDataEmpty() == true);
    std::cout << "test6 -- isMetaDataEmpty(): Vec6.\n";
    assert(dt_vec6.isMetaDataEmpty() == true);

    // Insert metadata into DataTable using lvalues.
    int integer = 1000000;
    std::vector<int> vector{1, 2, 3, 4};
    std::vector<int> vector_r{4, 3, 2, 1};
    std::string string{"metadata"};
    std::cout << "test6 -- insertMetaData(lvalue): Real.\n";
    dt_real.insertMetaData("integer_l", integer);
    dt_real.insertMetaData("vector_l" , vector);
    dt_real.insertMetaData("string_l" , string);
    std::cout << "test6 -- insertMetaData(lvalue): Vec3.\n";
    dt_vec3.insertMetaData("integer_l", integer);
    dt_vec3.insertMetaData("vector_l" , vector);
    dt_vec3.insertMetaData("string_l" , string);
    std::cout << "test6 -- insertMetaData(lvalue): Vec6.\n";
    dt_vec6.insertMetaData("integer_l", integer);
    dt_vec6.insertMetaData("vector_l" , vector);
    dt_vec6.insertMetaData("string_l" , string);

    // Insert metadata into DataTable using rvalues.
    std::cout << "test6 -- insertMetaData(rvalue): Real.\n";
    dt_real.insertMetaData("integer_r", 1000000);
    dt_real.insertMetaData("vector_r" , std::vector<int>{4, 3, 2, 1});
    dt_real.insertMetaData("string_r" , std::string{"metadata"});
    std::cout << "test6 -- insertMetaData(rvalue): Vec3.\n";
    dt_vec3.insertMetaData("integer_r", 1000000);
    dt_vec3.insertMetaData("vector_r" , std::vector<int>{4, 3, 2, 1});
    dt_vec3.insertMetaData("string_r" , std::string{"metadata"});
    std::cout << "test6 -- insertMetaData(rvalue): Vec6.\n";
    dt_vec6.insertMetaData("integer_r", 1000000);
    dt_vec6.insertMetaData("vector_r" , std::vector<int>{4, 3, 2, 1});
    dt_vec6.insertMetaData("string_r" , std::string{"metadata"});

    // Check the size of metadata.
    std::cout << "test6 -- getMetaDataSize(): Real.\n";
    assert(dt_real.getMetaDataSize() == 6);
    std::cout << "test6 -- getMetaDataSize(): Vec3.\n";
    assert(dt_vec3.getMetaDataSize() == 6);
    std::cout << "test6 -- getMetaDataSize(): Vec6.\n";
    assert(dt_vec6.getMetaDataSize() == 6);

    // Check metadata values.
    std::cout << "test6 -- getMetaData(): Real.\n";
    assert(dt_real.getMetaData<decltype(integer)>("integer_l") == integer);
    assert(dt_real.getMetaData<decltype(vector)>("vector_l")   == vector);
    assert(dt_real.getMetaData<decltype(string)>("string_l")   == string);
    assert(dt_real.getMetaData<decltype(integer)>("integer_r") == integer);
    assert(dt_real.getMetaData<decltype(vector)>("vector_r")   == vector_r);
    assert(dt_real.getMetaData<decltype(string)>("string_r")   == string);
    std::cout << "test6 -- getMetaData(): Vec3.\n";
    assert(dt_vec3.getMetaData<decltype(integer)>("integer_l") == integer);
    assert(dt_vec3.getMetaData<decltype(vector)>("vector_l")   == vector);
    assert(dt_vec3.getMetaData<decltype(string)>("string_l")   == string);
    assert(dt_real.getMetaData<decltype(integer)>("integer_r") == integer);
    assert(dt_real.getMetaData<decltype(vector)>("vector_r")   == vector_r);
    assert(dt_real.getMetaData<decltype(string)>("string_r")   == string);
    std::cout << "test6 -- getMetaData(): Vec6.\n";
    assert(dt_vec6.getMetaData<decltype(integer)>("integer_l") == integer);
    assert(dt_vec6.getMetaData<decltype(vector)>("vector_l")   == vector);
    assert(dt_vec6.getMetaData<decltype(string)>("string_l")   == string);
    assert(dt_real.getMetaData<decltype(integer)>("integer_r") == integer);
    assert(dt_real.getMetaData<decltype(vector)>("vector_r")   == vector_r);
    assert(dt_real.getMetaData<decltype(string)>("string_r")   == string);

    // Update metadata values.
    std::cout << "test6 -- updMetaData(): Real.\n";
    dt_real.updMetaData<decltype(integer)>("integer_l") = 1;
    dt_real.updMetaData<decltype(vector)>("vector_r").push_back(5);
    dt_real.updMetaData<decltype(string)>("string_l") = "updmetadata";
    std::cout << "test6 -- updMetaData(): Vec3.\n";
    dt_vec3.updMetaData<decltype(integer)>("integer_l") = 1;
    dt_vec3.updMetaData<decltype(vector)>("vector_r").push_back(5);
    dt_vec3.updMetaData<decltype(string)>("string_l") = "updmetadata";
    std::cout << "test6 -- updMetaData(): Vec6.\n";
    dt_vec6.updMetaData<decltype(integer)>("integer_l") = 1;
    dt_vec6.updMetaData<decltype(vector)>("vector_r").push_back(5);
    dt_vec6.updMetaData<decltype(string)>("string_l") = "updmetadata";

    // Check the size of metadata.
    std::cout << "test6 -- getMetaDataSize(): Real.\n";
    assert(dt_real.getMetaDataSize() == 6);
    std::cout << "test6 -- getMetaDataSize(): Vec3.\n";
    assert(dt_vec3.getMetaDataSize() == 6);
    std::cout << "test6 -- getMetaDataSize(): Vec6.\n";
    assert(dt_vec6.getMetaDataSize() == 6);

    integer = 1;
    vector_r.push_back(5);
    string = "updmetadata";

    // Check metadata values.
    std::cout << "test6 -- getMetaData(): Real.\n";
    assert(dt_real.getMetaData<decltype(integer)>("integer_l") == integer);
    assert(dt_real.getMetaData<decltype(string)>("string_l")   == string);
    assert(dt_real.getMetaData<decltype(vector)>("vector_r")   == vector_r);
    std::cout << "test6 -- getMetaData(): Vec3.\n";
    assert(dt_vec3.getMetaData<decltype(integer)>("integer_l") == integer);
    assert(dt_vec3.getMetaData<decltype(string)>("string_l")   == string);
    assert(dt_real.getMetaData<decltype(vector)>("vector_r")   == vector_r);
    std::cout << "test6 -- getMetaData(): Vec6.\n";
    assert(dt_vec6.getMetaData<decltype(integer)>("integer_l") == integer);
    assert(dt_vec6.getMetaData<decltype(string)>("string_l")   == string);
    assert(dt_real.getMetaData<decltype(vector)>("vector_r")   == vector_r);

    // Pop some metadata values.
    std::cout << "test6 -- popMetaData(): Real.\n";
    assert(dt_real.popMetaData<decltype(string)>("string_l") == string);
    assert(dt_real.popMetaData<decltype(vector)>("vector_l") == vector);
    std::cout << "test6 -- popMetaData(): Vec3.\n";
    assert(dt_vec3.popMetaData<decltype(string)>("string_l") == string);
    assert(dt_vec3.popMetaData<decltype(vector)>("vector_l") == vector);
    std::cout << "test6 -- popMetaData(): Vec6.\n";
    assert(dt_vec6.popMetaData<decltype(string)>("string_l") == string);
    assert(dt_vec6.popMetaData<decltype(vector)>("vector_l") == vector);
  
    // Check the size of metadata.
    std::cout << "test6 -- getMetaDataSize(): Real.\n";
    assert(dt_real.getMetaDataSize() == 4);
    std::cout << "test6 -- getMetaDataSize(): Vec3.\n";
    assert(dt_vec3.getMetaDataSize() == 4);
    std::cout << "test6 -- getMetaDataSize(): Vec6.\n";
    assert(dt_vec6.getMetaDataSize() == 4);

    // Remove some metadata values.
    std::cout << "test6 -- removeMetaData(): Real.\n";
    dt_real.removeMetaData("integer_r");
    dt_real.removeMetaData("integer_l");
    std::cout << "test6 -- removeMetaData(): Vec3.\n";
    dt_vec3.removeMetaData("integer_r");
    dt_vec3.removeMetaData("integer_l");
    std::cout << "test6 -- removeMetaData(): Vec6.\n";
    dt_vec6.removeMetaData("integer_r");
    dt_vec6.removeMetaData("integer_l");

    // Check the size of metadata.
    std::cout << "test6 -- getMetaDataSize(): Real.\n";
    assert(dt_real.getMetaDataSize() == 2);
    std::cout << "test6 -- getMetaDataSize(): Vec3.\n";
    assert(dt_vec3.getMetaDataSize() == 2);
    std::cout << "test6 -- getMetaDataSize(): Vec6.\n";
    assert(dt_vec6.getMetaDataSize() == 2);

    // Check if metadata is empty in the DataTable.
    std::cout << "test6 -- isMetaDataEmpty(): Real.\n";
    assert(dt_real.isMetaDataEmpty() == false);
    std::cout << "test6 -- isMetaDataEmpty(): Vec3.\n";
    assert(dt_vec3.isMetaDataEmpty() == false);
    std::cout << "test6 -- isMetaDataEmpty(): Vec6.\n";
    assert(dt_vec6.isMetaDataEmpty() == false);

    // Check existence of metadata.
    std::cout << "test6 -- hasMetaData(): Real.\n";
    assert(dt_real.hasMetaData("integer_l") == false);
    assert(dt_real.hasMetaData("string_l") == false);
    assert(dt_real.hasMetaData("vector_l") == false);
    assert(dt_real.hasMetaData("integer_r") == false);
    assert(dt_real.hasMetaData("string_r") == true);
    assert(dt_real.hasMetaData("vector_r") == true);
    std::cout << "test6 -- hasMetaData(): Vec3.\n";
    assert(dt_vec3.hasMetaData("integer_l") == false);
    assert(dt_vec3.hasMetaData("string_l") == false);
    assert(dt_vec3.hasMetaData("vector_l") == false);
    assert(dt_vec3.hasMetaData("integer_r") == false);
    assert(dt_vec3.hasMetaData("string_r") == true);
    assert(dt_vec3.hasMetaData("vector_r") == true);
    std::cout << "test6 -- hasMetaData(): Vec6.\n";
    assert(dt_vec6.hasMetaData("integer_l") == false);
    assert(dt_vec6.hasMetaData("string_l") == false);
    assert(dt_vec6.hasMetaData("vector_l") == false);
    assert(dt_vec6.hasMetaData("integer_r") == false);
    assert(dt_vec6.hasMetaData("string_r") == true);
    assert(dt_vec6.hasMetaData("vector_r") == true);

    // Clear metadata.
    std::cout << "test6 -- clearMetaData(): Real.\n";
    dt_real.clearMetaData();
    std::cout << "test6 -- clearMetaData(): Vec3.\n";
    dt_vec3.clearMetaData();
    std::cout << "test6 -- clearMetaData(): Vec6.\n";
    dt_vec6.clearMetaData();

    // Check the size of metadata.
    std::cout << "test6 -- getMetaDataSize(): Real.\n";
    assert(dt_real.getMetaDataSize() == 0);
    std::cout << "test6 -- getMetaDataSize(): Vec3.\n";
    assert(dt_vec3.getMetaDataSize() == 0);
    std::cout << "test6 -- getMetaDataSize(): Vec6.\n";
    assert(dt_vec6.getMetaDataSize() == 0);
}


// Test copy construction, move construction, copy assignment and move
// assignment.
void test7() {
    constexpr double EPSILON{0.0001};

    // Construct DataTable.
    std::cout << "test7 -- Constructor DataTable: Real.\n";
    OpenSim::DataTable_<SimTK::Real> dt_real{3, 4, 10};
    std::cout << "test7 -- Constructor DataTable: Vec3.\n";
    OpenSim::DataTable_<SimTK::Vec3> dt_vec3{3, 4, {10, 20, 30}};
    std::cout << "test7 -- Constructor DataTable: Vec6.\n";
    OpenSim::DataTable_<SimTK::Vec6> dt_vec6{3, 4, {10, 20, 30, 40, 50, 60}};

    // Add column labels.
    std::cout << "test7 -- setColumnLabel(): Real.\n";
    dt_real.setColumnLabel(0, "zero");
    dt_real.setColumnLabel(2, "two");
    std::cout << "test7 -- setColumnLabel(): Vec3.\n";
    dt_vec3.setColumnLabel(0, "zero");
    dt_vec3.setColumnLabel(2, "two");
    std::cout << "test7 -- setColumnLabel(): Vec6.\n";
    dt_vec6.setColumnLabel(0, "zero");
    dt_vec6.setColumnLabel(2, "two");

    // Add metadata.
    std::string string{"metadata"};
    std::vector<int> vector{1, 2, 3, 4};
    std::cout << "test7 -- insertMetaData(): Real.\n";
    dt_real.insertMetaData("int", 100);
    dt_real.insertMetaData("string", string);
    dt_real.insertMetaData("vector", vector);
    std::cout << "test7 -- insertMetaData(): Vec3.\n";
    dt_vec3.insertMetaData("int", 100);
    dt_vec3.insertMetaData("string", string);
    dt_vec3.insertMetaData("vector", vector);
    std::cout << "test7 -- insertMetaData(): Vec6.\n";
    dt_vec6.insertMetaData("int", 100);
    dt_vec6.insertMetaData("string", string);
    dt_vec6.insertMetaData("vector", vector);

    // Copy construct.
    std::cout << "test7 -- Copy construct: Real.\n";
    auto dt_real_copy = dt_real;
    std::cout << "test7 -- Copy construct: Vec3.\n";
    auto dt_vec3_copy = dt_vec3;
    std::cout << "test7 -- Copy construct: Vec6.\n";
    auto dt_vec6_copy = dt_vec6;

    // Verify the copied data is the same as original.
    std::cout << "test7 -- Check copied data: Real.\n";
    for(size_t r = 0; r < dt_real_copy.getNumRows(); ++r)
        for(size_t c = 0; c < dt_real_copy.getNumColumns(); ++c)
            assert(std::abs(dt_real_copy.getElt(r, c) - 10) < EPSILON);
    std::cout << "test7 -- Check copied data: Vec3.\n";
    for(size_t r = 0; r < dt_vec3.getNumRows(); ++r)
        for(size_t c = 0; c < dt_vec3_copy.getNumColumns(); ++c)
            assert(dt_vec3_copy.getElt(r, c) == SimTK::Vec3(10, 20, 30));
    std::cout << "test7 -- Check copied data: Vec6.\n";
    for(size_t r = 0; r < dt_vec6_copy.getNumRows(); ++r)
        for(size_t c = 0; c < dt_vec6_copy.getNumColumns(); ++c)
            assert(dt_vec6_copy.getElt(r, c) == SimTK::Vec6(10, 20, 30, 
                                                            40, 50, 60));

    // Verify the copied column labels.
    std::cout << "test7 -- Check copied column labels: Real.\n";
    assert(dt_real_copy.getColumnLabel(0) == "zero");
    assert(dt_real_copy.getColumnLabel(2) == "two");
    std::cout << "test7 -- Check copied column labels: Vec3.\n";
    assert(dt_vec3_copy.getColumnLabel(0) == "zero");
    assert(dt_vec3_copy.getColumnLabel(2) == "two");
    std::cout << "test7 -- Check copied column labels: Vec6.\n";
    assert(dt_vec6_copy.getColumnLabel(0) == "zero");
    assert(dt_vec6_copy.getColumnLabel(2) == "two");

    // Verify the copied metadata.
    std::cout << "test7 -- Check copied metadata: Real.\n";
    assert(dt_real_copy.getMetaData<int>("int") == 100);
    assert(dt_real_copy.getMetaData<decltype(string)>("string") == string);
    assert(dt_real_copy.getMetaData<decltype(vector)>("vector") == vector);
    std::cout << "test7 -- Check copied metadata: Vec3.\n";
    assert(dt_vec3_copy.getMetaData<int>("int") == 100);
    assert(dt_vec3_copy.getMetaData<decltype(string)>("string") == string);
    assert(dt_vec3_copy.getMetaData<decltype(vector)>("vector") == vector);
    std::cout << "test7 -- Check copied metadata: Vec6.\n";
    assert(dt_vec6_copy.getMetaData<int>("int") == 100);
    assert(dt_vec6_copy.getMetaData<decltype(string)>("string") == string);
    assert(dt_vec6_copy.getMetaData<decltype(vector)>("vector") == vector);

    // Edit the original data.
    std::cout << "test7 -- Edit the original data: Real.\n";
    dt_real.updRow(1) = 100;
    std::cout << "test7 -- Edit the original data: Vec3.\n";
    dt_vec3.updRow(1) = SimTK::Vec3{100, 200, 300};
    std::cout << "test7 -- Edit the original data: Vec6.\n";
    dt_vec6.updRow(1) = SimTK::Vec6{100, 200, 300, 400, 500, 600};

    // Edit the original column labels.
    std::cout << "test7 -- changeColumnLabel(): Real.\n";
    dt_real.changeColumnLabel(0, "col_zero");
    dt_real.changeColumnLabel(2, "col_two");
    std::cout << "test7 -- changeColumnLabel(): Vec3.\n";
    dt_vec3.changeColumnLabel(0, "col_zero");
    dt_vec3.changeColumnLabel(2, "col_two");
    std::cout << "test7 -- changeColumnLabel(): Vec6.\n";
    dt_vec6.changeColumnLabel(0, "col_zero");
    dt_vec6.changeColumnLabel(2, "col_two");

    // Edit the original metadata.
    std::cout << "test7 -- updMetaData(): Real.\n";
    dt_real.updMetaData<int>("int") = 200;
    dt_real.updMetaData<decltype(string)>("string") = string + string;
    dt_real.updMetaData<decltype(vector)>("vector") = {10, 20, 30, 40};
    std::cout << "test7 -- updMetaData(): Vec3.\n";
    dt_vec3.updMetaData<int>("int") = 200;
    dt_vec3.updMetaData<decltype(string)>("string") = string + string;
    dt_vec3.updMetaData<decltype(vector)>("vector") = {10, 20, 30, 40};
    std::cout << "test7 -- updMetaData(): Vec6.\n";
    dt_vec6.updMetaData<int>("int") = 200;
    dt_vec6.updMetaData<decltype(string)>("string") = string + string;
    dt_vec6.updMetaData<decltype(vector)>("vector") = {10, 20, 30, 40};
  
    // Verify the copied data is unchanged.
    std::cout << "test7 -- Check copied data: Real.\n";
    for(size_t r = 0; r < dt_real_copy.getNumRows(); ++r)
        for(size_t c = 0; c < dt_real_copy.getNumColumns(); ++c)
            assert(std::abs(dt_real_copy.getElt(r, c) - 10) < EPSILON);
    std::cout << "test7 -- Check copied data: Vec3.\n";
    for(size_t r = 0; r < dt_vec3.getNumRows(); ++r)
        for(size_t c = 0; c < dt_vec3_copy.getNumColumns(); ++c)
            assert(dt_vec3_copy.getElt(r, c) == SimTK::Vec3(10, 20, 30));
    std::cout << "test7 -- Check copied data: Vec6.\n";
    for(size_t r = 0; r < dt_vec6_copy.getNumRows(); ++r)
        for(size_t c = 0; c < dt_vec6_copy.getNumColumns(); ++c)
            assert(dt_vec6_copy.getElt(r, c) == SimTK::Vec6(10, 20, 30, 
                                                            40, 50, 60));

    // Verify that copied column labels are unchanged.
    std::cout << "test7 -- Check copied column labels: Real.\n";
    assert(dt_real_copy.getColumnLabel(0) == "zero");
    assert(dt_real_copy.getColumnLabel(2) == "two");
    std::cout << "test7 -- Check copied column labels: Vec3.\n";
    assert(dt_vec3_copy.getColumnLabel(0) == "zero");
    assert(dt_vec3_copy.getColumnLabel(2) == "two");
    std::cout << "test7 -- Check copied column labels: Vec6.\n";
    assert(dt_vec6_copy.getColumnLabel(0) == "zero");
    assert(dt_vec6_copy.getColumnLabel(2) == "two");

    // Verify the copied metadata is unchanged.
    std::cout << "test7 -- Check copied metadata: Real.\n";
    assert(dt_real_copy.getMetaData<int>("int") == 100);
    assert(dt_real_copy.getMetaData<decltype(string)>("string") == string);
    assert(dt_real_copy.getMetaData<decltype(vector)>("vector") == vector);
    std::cout << "test7 -- Check copied metadata: Vec3.\n";
    assert(dt_vec3_copy.getMetaData<int>("int") == 100);
    assert(dt_vec3_copy.getMetaData<decltype(string)>("string") == string);
    assert(dt_vec3_copy.getMetaData<decltype(vector)>("vector") == vector);
    std::cout << "test7 -- Check copied metadata: Vec6.\n";
    assert(dt_vec6_copy.getMetaData<int>("int") == 100);
    assert(dt_vec6_copy.getMetaData<decltype(string)>("string") == string);
    assert(dt_vec6_copy.getMetaData<decltype(vector)>("vector") == vector);

    // Assign the copy to the original so original gets back its initial state.
    std::cout << "test7 -- operator=(): Real.\n";
    dt_real = dt_real_copy;
    std::cout << "test7 -- operator=(): Vec3.\n";
    dt_vec3 = dt_vec3_copy;
    std::cout << "test7 -- operator=(): Vec6.\n";
    dt_vec6 = dt_vec6_copy;

    // Verify the orignal got back its data.
    std::cout << "test7 -- Check original data: Real.\n";
    for(size_t r = 0; r < dt_real.getNumRows(); ++r)
        for(size_t c = 0; c < dt_real.getNumColumns(); ++c)
            assert(std::abs(dt_real.getElt(r, c) - 10) < EPSILON);
    std::cout << "test7 -- Check original data: Vec3.\n";
    for(size_t r = 0; r < dt_vec3.getNumRows(); ++r)
        for(size_t c = 0; c < dt_vec3.getNumColumns(); ++c)
            assert(dt_vec3.getElt(r, c) == SimTK::Vec3(10, 20, 30));
    std::cout << "test7 -- Check original data: Vec6.\n";
    for(size_t r = 0; r < dt_vec6.getNumRows(); ++r)
        for(size_t c = 0; c < dt_vec6.getNumColumns(); ++c)
            assert(dt_vec6.getElt(r, c) == SimTK::Vec6(10, 20, 30, 40, 50, 60));

    // Verify that original got back its column labels.
    std::cout << "test7 -- Check original column labels: Real.\n";
    assert(dt_real.getColumnLabel(0) == "zero");
    assert(dt_real.getColumnLabel(2) == "two");
    std::cout << "test7 -- Check original column labels: Vec3.\n";
    assert(dt_vec3.getColumnLabel(0) == "zero");
    assert(dt_vec3.getColumnLabel(2) == "two");
    std::cout << "test7 -- Check original column labels: Vec6.\n";
    assert(dt_vec6.getColumnLabel(0) == "zero");
    assert(dt_vec6.getColumnLabel(2) == "two");

    // Verify the original got back its metadata.
    std::cout << "test7 -- Check original metadata: Real.\n";
    assert(dt_real.getMetaData<int>("int") == 100);
    assert(dt_real.getMetaData<decltype(string)>("string") == string);
    assert(dt_real.getMetaData<decltype(vector)>("vector") == vector);
    std::cout << "test7 -- Check original metadata: Vec3.\n";
    assert(dt_vec3.getMetaData<int>("int") == 100);
    assert(dt_vec3.getMetaData<decltype(string)>("string") == string);
    assert(dt_vec3.getMetaData<decltype(vector)>("vector") == vector);
    std::cout << "test7 -- Check original metadata: Vec6.\n";
    assert(dt_vec6.getMetaData<int>("int") == 100);
    assert(dt_vec6.getMetaData<decltype(string)>("string") == string);
    assert(dt_vec6.getMetaData<decltype(vector)>("vector") == vector);

    // Move construct using the copy of the original.
    std::cout << "test7 -- Move constructor: Real.\n";
    decltype(dt_real) dt_real_move{std::move(dt_real_copy)};
    std::cout << "test7 -- Move constructor: Vec3.\n";
    decltype(dt_vec3) dt_vec3_move{std::move(dt_vec3_copy)};
    std::cout << "test7 -- Move constructor: Vec6.\n";
    decltype(dt_vec6) dt_vec6_move{std::move(dt_vec6_copy)};

    // Verify the moved data is the same as original.
    std::cout << "test7 -- Check moved data: Real.\n";
    for(size_t r = 0; r < dt_real_move.getNumRows(); ++r)
        for(size_t c = 0; c < dt_real_move.getNumColumns(); ++c)
            assert(std::abs(dt_real_move.getElt(r, c) - 10) < EPSILON);
    std::cout << "test7 -- Check moved data: Vec3.\n";
    for(size_t r = 0; r < dt_vec3.getNumRows(); ++r)
        for(size_t c = 0; c < dt_vec3_move.getNumColumns(); ++c)
            assert(dt_vec3_move.getElt(r, c) == SimTK::Vec3(10, 20, 30));
    std::cout << "test7 -- Check moved data: Vec6.\n";
    for(size_t r = 0; r < dt_vec6_move.getNumRows(); ++r)
        for(size_t c = 0; c < dt_vec6_move.getNumColumns(); ++c)
            assert(dt_vec6_move.getElt(r, c) == SimTK::Vec6(10, 20, 30, 
                                                            40, 50, 60));

    // Verify the moved column labels.
    std::cout << "test7 -- Check moved column labels: Real.\n";
    assert(dt_real_move.getColumnLabel(0) == "zero");
    assert(dt_real_move.getColumnLabel(2) == "two");
    std::cout << "test7 -- Check moved column labels: Vec3.\n";
    assert(dt_vec3_move.getColumnLabel(0) == "zero");
    assert(dt_vec3_move.getColumnLabel(2) == "two");
    std::cout << "test7 -- Check moved column labels: Vec6.\n";
    assert(dt_vec6_move.getColumnLabel(0) == "zero");
    assert(dt_vec6_move.getColumnLabel(2) == "two");

    // Verify the moved metadata.
    std::cout << "test7 -- Check moved metadata: Real.\n";
    assert(dt_real_move.getMetaData<int>("int") == 100);
    assert(dt_real_move.getMetaData<decltype(string)>("string") == string);
    assert(dt_real_move.getMetaData<decltype(vector)>("vector") == vector);
    std::cout << "test7 -- Check moved metadata: Vec3.\n";
    assert(dt_vec3_move.getMetaData<int>("int") == 100);
    assert(dt_vec3_move.getMetaData<decltype(string)>("string") == string);
    assert(dt_vec3_move.getMetaData<decltype(vector)>("vector") == vector);
    std::cout << "test7 -- Check moved metadata: Vec6.\n";
    assert(dt_vec6_move.getMetaData<int>("int") == 100);
    assert(dt_vec6_move.getMetaData<decltype(string)>("string") == string);
    assert(dt_vec6_move.getMetaData<decltype(vector)>("vector") == vector);

    // Edit the original data.
    std::cout << "test7 -- Edit the original data: Real.\n";
    dt_real.updRow(1) = 100;
    std::cout << "test7 -- Edit the original data: Vec3.\n";
    dt_vec3.updRow(1) = SimTK::Vec3{100, 200, 300};
    std::cout << "test7 -- Edit the original data: Vec6.\n";
    dt_vec6.updRow(1) = SimTK::Vec6{100, 200, 300, 400, 500, 600};

    // Edit the original column labels.
    std::cout << "test7 -- changeColumnLabel(): Real.\n";
    dt_real.changeColumnLabel(0, "col_zero");
    dt_real.changeColumnLabel(2, "col_two");
    std::cout << "test7 -- changeColumnLabel(): Vec3.\n";
    dt_vec3.changeColumnLabel(0, "col_zero");
    dt_vec3.changeColumnLabel(2, "col_two");
    std::cout << "test7 -- changeColumnLabel(): Vec6.\n";
    dt_vec6.changeColumnLabel(0, "col_zero");
    dt_vec6.changeColumnLabel(2, "col_two");

    // Edit the original metadata.
    std::cout << "test7 -- updMetaData(): Real.\n";
    dt_real.updMetaData<int>("int") = 200;
    dt_real.updMetaData<decltype(string)>("string") = string + string;
    dt_real.updMetaData<decltype(vector)>("vector") = {10, 20, 30, 40};
    std::cout << "test7 -- updMetaData(): Vec3.\n";
    dt_vec3.updMetaData<int>("int") = 200;
    dt_vec3.updMetaData<decltype(string)>("string") = string + string;
    dt_vec3.updMetaData<decltype(vector)>("vector") = {10, 20, 30, 40};
    std::cout << "test7 -- updMetaData(): Vec6.\n";
    dt_vec6.updMetaData<int>("int") = 200;
    dt_vec6.updMetaData<decltype(string)>("string") = string + string;
    dt_vec6.updMetaData<decltype(vector)>("vector") = {10, 20, 30, 40};

    // Move assign the 'moved' data to the orignal so original gets back its
    // initial state.
    std::cout << "test7 -- Move assignment: Real.\n";
    dt_real = std::move(dt_real_move);
    std::cout << "test7 -- Move assignment: Vec3.\n";
    dt_vec3 = std::move(dt_vec3_move);
    std::cout << "test7 -- Move assignment: Vec6.\n";
    dt_vec6 = std::move(dt_vec6_move);
  
    // Verify the orignal got back its data.
    std::cout << "test7 -- Check original data: Real.\n";
    for(size_t r = 0; r < dt_real.getNumRows(); ++r)
        for(size_t c = 0; c < dt_real.getNumColumns(); ++c)
            assert(std::abs(dt_real.getElt(r, c) - 10) < EPSILON);
    std::cout << "test7 -- Check original data: Vec3.\n";
    for(size_t r = 0; r < dt_vec3.getNumRows(); ++r)
        for(size_t c = 0; c < dt_vec3.getNumColumns(); ++c)
            assert(dt_vec3.getElt(r, c) == SimTK::Vec3(10, 20, 30));
    std::cout << "test7 -- Check original data: Vec6.\n";
    for(size_t r = 0; r < dt_vec6.getNumRows(); ++r)
        for(size_t c = 0; c < dt_vec6.getNumColumns(); ++c)
            assert(dt_vec6.getElt(r, c) == SimTK::Vec6(10, 20, 30, 40, 50, 60));

    // Verify that original got back its column labels.
    std::cout << "test7 -- Check original column labels: Real.\n";
    assert(dt_real.getColumnLabel(0) == "zero");
    assert(dt_real.getColumnLabel(2) == "two");
    std::cout << "test7 -- Check original column labels: Vec3.\n";
    assert(dt_vec3.getColumnLabel(0) == "zero");
    assert(dt_vec3.getColumnLabel(2) == "two");
    std::cout << "test7 -- Check original column labels: Vec6.\n";
    assert(dt_vec6.getColumnLabel(0) == "zero");
    assert(dt_vec6.getColumnLabel(2) == "two");

    // Verify the original got back its metadata.
    std::cout << "test7 -- Check original metadata: Real.\n";
    assert(dt_real.getMetaData<int>("int") == 100);
    assert(dt_real.getMetaData<decltype(string)>("string") == string);
    assert(dt_real.getMetaData<decltype(vector)>("vector") == vector);
    std::cout << "test7 -- Check original metadata: Vec3.\n";
    assert(dt_vec3.getMetaData<int>("int") == 100);
    assert(dt_vec3.getMetaData<decltype(string)>("string") == string);
    assert(dt_vec3.getMetaData<decltype(vector)>("vector") == vector);
    std::cout << "test7 -- Check original metadata: Vec6.\n";
    assert(dt_vec6.getMetaData<int>("int") == 100);
    assert(dt_vec6.getMetaData<decltype(string)>("string") == string);
    assert(dt_vec6.getMetaData<decltype(vector)>("vector") == vector);
}


// Test adding DataTable to containers.
void test8() {
    // Construct DataTables.
    std::cout << "test8 -- Construct DataTable with default value: Real.\n";
    OpenSim::DataTable_<SimTK::Real> dt_real{3, 4, 10};
    std::cout << "test8 -- Construct DataTable with default value: Vec3.\n";
    OpenSim::DataTable_<SimTK::Vec3> dt_vec3{3, 4, {10, 20, 30}};
    std::cout << "test8 -- Construct DataTable with default value: Vec6.\n"; 
    OpenSim::DataTable_<SimTK::Vec6> dt_vec6{3, 4, {10, 20, 30, 40, 50, 60}};

    // Sequence container.
    std::vector<OpenSim::AbstractDataTable*> vector{};

    // Add the DataTables to vector.
    std::cout << "test8 -- push_back to std::vector: Real.\n";
    vector.push_back(&dt_real);
    std::cout << "test8 -- push_back to std::vector: Vec3.\n";
    vector.push_back(&dt_vec3);
    std::cout << "test8 -- push_back to std::vector: Vec6.\n";
    vector.push_back(&dt_vec6);

    // Add column labels to all the DataTables in the container.
    std::cout << "test8 -- Add column labels.\n";
    for(auto& dt : vector) {
        dt->setColumnLabel(0, "col-zero");
        dt->setColumnLabel(2, "col-two");
    }

    // Check existence of column labels.
    std::cout << "test8 -- Check existence of col labels: Real.\n";
    assert(dt_real.columnHasLabel(0) == true);
    assert(dt_real.columnHasLabel(2) == true);
    assert(dt_real.columnHasLabel(1) == false);
    assert(dt_real.columnHasLabel(3) == false);
    std::cout << "test8 -- Check existence of col labels: Vec3.\n";
    assert(dt_vec3.columnHasLabel(0) == true);
    assert(dt_vec3.columnHasLabel(2) == true);
    assert(dt_vec3.columnHasLabel(1) == false);
    assert(dt_vec3.columnHasLabel(3) == false);
    std::cout << "test8 -- Check existence of col labels: Vec6.\n";
    assert(dt_vec6.columnHasLabel(0) == true);
    assert(dt_vec6.columnHasLabel(2) == true);
    assert(dt_vec6.columnHasLabel(1) == false);
    assert(dt_vec6.columnHasLabel(3) == false);

    // Check col labels.
    std::cout << "test8 -- Check col labels: Real.\n";
    assert(vector[0]->getColumnLabel(0) == "col-zero" &&
           dt_real.getColumnLabel(0)    == "col-zero");
    assert(vector[0]->getColumnLabel(2) == "col-two" &&
           dt_real.getColumnLabel(2)    == "col-two");
    std::cout << "test8 -- Check col labels: Vec3.\n";
    assert(vector[1]->getColumnLabel(0) == "col-zero" &&
           dt_vec3.getColumnLabel(0)    == "col-zero");
    assert(vector[1]->getColumnLabel(2) == "col-two" &&
           dt_vec3.getColumnLabel(2)    == "col-two");
    std::cout << "test8 -- Check col labels: Vec6.\n";
    assert(vector[2]->getColumnLabel(0) == "col-zero" &&
           dt_vec6.getColumnLabel(0)    == "col-zero");
    assert(vector[2]->getColumnLabel(2) == "col-two" &&
           dt_vec6.getColumnLabel(2)    == "col-two");

    // Check col indices.
    std::cout << "test8 -- Check col indices: Real.\n";
    assert(vector[0]->getColumnIndex("col-zero") == 0 &&
           dt_real.getColumnIndex("col-zero")    == 0);
    assert(vector[2]->getColumnIndex("col-two") == 2 &&
           dt_real.getColumnIndex("col-two")    == 2);
    std::cout << "test8 -- Check col indices: Vec3.\n";
    assert(vector[0]->getColumnIndex("col-zero") == 0 &&
           dt_vec3.getColumnIndex("col-zero")    == 0);
    assert(vector[2]->getColumnIndex("col-two") == 2 &&
           dt_vec3.getColumnIndex("col-two")    == 2);
    std::cout << "test8 -- Check col indices: Vec6.\n";
    assert(vector[0]->getColumnIndex("col-zero") == 0 &&
           dt_vec6.getColumnIndex("col-zero")    == 0);
    assert(vector[2]->getColumnIndex("col-two") == 2 &&
           dt_vec6.getColumnIndex("col-two")    == 2);

    // Update col labels.
    std::cout << "test8 -- Update column labels.\n";
    for(auto& dt : vector) {
        dt->changeColumnLabel(0, "column-zero");
        dt->changeColumnLabel("col-two", "column-two");
    }

    // Check col labels.
    std::cout << "test8 -- Check col labels: Real.\n";
    assert(vector[0]->getColumnLabel(0) == dt_real.getColumnLabel(0));
    assert(vector[0]->getColumnLabel(2) == dt_real.getColumnLabel(2));
    std::cout << "test8 -- Check col labels: Vec3.\n";
    assert(vector[1]->getColumnLabel(0) == dt_vec3.getColumnLabel(0));
    assert(vector[1]->getColumnLabel(2) == dt_vec3.getColumnLabel(2));
    std::cout << "test8 -- Check col labels: Vec6.\n";
    assert(vector[2]->getColumnLabel(0) == dt_vec6.getColumnLabel(0));
    assert(vector[2]->getColumnLabel(2) == dt_vec6.getColumnLabel(2));

    // Check the col labels.
    std::cout << "test8 -- Check col labels: Real.\n";
    assert(vector[0]->getColumnLabel(0) == "column-zero" &&
           dt_real.getColumnLabel(0)    == "column-zero");
    assert(vector[0]->getColumnLabel(2) == "column-two" &&
           dt_real.getColumnLabel(2)    == "column-two");
    std::cout << "test8 -- Check col labels: Vec3.\n";
    assert(vector[1]->getColumnLabel(0) == "column-zero" &&
           dt_vec3.getColumnLabel(0)    == "column-zero");
    assert(vector[1]->getColumnLabel(2) == "column-two" &&
           dt_vec3.getColumnLabel(2)    == "column-two");
    std::cout << "test8 -- Check col labels: Vec6.\n";
    assert(vector[2]->getColumnLabel(0) == "column-zero" &&
           dt_vec6.getColumnLabel(0)    == "column-zero");
    assert(vector[2]->getColumnLabel(2) == "column-two" &&
           dt_vec6.getColumnLabel(2)    == "column-two");

    // Clear the column labels.
    std::cout << "test8 -- Clear column labels.\n";
    for(auto& dt : vector)
        dt->clearColumnLabels();
}


int main() {
    test1();

    test2();

    test3();

    test4();

    test5();

    test6();

    test7();

    test8();

    std::vector<SimTK::Real> tsdt_data{1, 1, 1, 1, 
                                       2, 2, 2, 2, 
                                       3, 3, 3, 3};
    OpenSim::TimeSeriesDataTable_<SimTK::Real, float> 
        tsdt{tsdt_data.cbegin(),
             tsdt_data.cend(),
             4,
             OpenSim::InputItDim::RowWise};

    std::vector<double> tmp{4, 4, 4, 4};
    tsdt.addRow(tmp);

    std::cout << tsdt.getNumRows() << std::endl;

    std::vector<std::string> tsdt_labels{"col-one", "col-two", "col-three", 
                                         "col-four"};
    tsdt.setColumnLabels(tsdt_labels);

    tsdt.addTimestamps(std::vector<long>{1, 2, 3});

    return 0;
}
