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
#include <sstream>
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
  if(dt.getNumCols() != ncol)
    throw OpenSimTestFailed{"dt.getNumCols() = " + 
                            std::to_string(dt.getNumCols()) +
                            " and ncol = " + std::to_string(ncol)};

  // Retrieving any row and col must throw since DataTable is empty.
  try {
    auto row = dt.getRow(nrow); ignore(row);
  } catch (SimTK::Exception::IndexOutOfRange&) {}
  try {
    auto row = dt.updRow(nrow); ignore(row);
  } catch (SimTK::Exception::IndexOutOfRange&) {}
  try {
    auto col = dt.getCol(ncol); ignore(col);
  } catch (SimTK::Exception::IndexOutOfRange&) {}
  try {
    auto col = dt.getCol("foo"); ignore(col);
  } catch (OpenSim::ColumnDoesNotExist&) {}
  try {
    auto col = dt.updCol(ncol); ignore(col);
  } catch (SimTK::Exception::IndexOutOfRange&) {}
  try {
    auto col = dt.updCol("foo"); ignore(col);
  } catch (OpenSim::ColumnDoesNotExist&) {}

  // Retrieving any element should throw since the DataTable is empty.
  try {
    auto elt = dt.getElt(nrow, ncol); ignore(elt);
  } catch (SimTK::Exception::IndexOutOfRange&) {}
  try {
    auto elt = dt.getElt(nrow, "foo"); ignore(elt);
  } catch (OpenSim::ColumnDoesNotExist&) {}
  try {
    auto elt = dt.updElt(nrow, ncol); ignore(elt);
  } catch (SimTK::Exception::IndexOutOfRange&) {}
  try {
    auto elt = dt.updElt(nrow, "foo"); ignore(elt);
  } catch (OpenSim::ColumnDoesNotExist&) {}

  auto underlying_matrix = dt.getAsMatrix(); ignore(underlying_matrix);
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
    const auto col = dt.getCol(colnum);
    for(size_t i = 0; i < data.size(); ++i)
      assert(col[static_cast<int>(i)] - data[i] > N_EPS &&
             col[static_cast<int>(i)] - data[i] < P_EPS);
  }

  // Edit the col.
  {
    auto col = dt.updCol(colnum);
    for(size_t i = 0; i < data.size(); ++i)
      col[static_cast<int>(i)] += 1;
  }

  // Check if editing the col worked.
  {
    const auto col = dt.getCol(colnum);
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

  // Add the row to the empty DataTable.
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
    catch(OpenSim::InvalidEntry&) {}

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
}


// Test adding cols to a populated DataTable. Use the 
template<typename ET>
void testAddCol(OpenSim::DataTable_<ET>& dt,
                const std::vector<ET>& data) {
  size_t orig_ncol{dt.getNumCols()};

  // Add a col to the DataTable using a SimTK::Vector.
  SimTK::Vector_<ET> dt_col{static_cast<int>(data.size()), data.data()};
  dt.addCol(dt_col);

  // Check the size of the DataTable.
  checkDataTableLimits(dt, data.size(), orig_ncol + 1);

  // Check entries of row 0, which was just added.
  checkDataTableCol(dt, data, orig_ncol);

  // Add another col to the DataTable using iterators. This time add the data
  // in reverse order from the input vector.
  dt.addCol(data.crbegin(), data.crend());

  // // Check the size of the DataTable.
  checkDataTableLimits(dt, data.size(), orig_ncol + 2);

  // // Check entries of the col just added.
  checkDataTableCol(dt, std::vector<ET>{data.crbegin(), data.crend()}, 
                    orig_ncol + 1);

  // Try adding emtpy col.
  try {
    dt.addCol(SimTK::Vector_<ET>{});
  } catch(OpenSim::ZeroElements&) {}

  // Clear the DataTable.
  dt.clearData();

  // Check the size of the DataTable.
  checkDataTableLimits(dt, 0, 0);

  // Add the first col using iterators. Together, the previous invocations
  // and this fully cover the code in the addCol function. Using new_data 
  // that is a repeated version of the data.
  auto new_data = data;
  new_data.insert(new_data.cend(), data.cbegin(), data.cend());
  dt.addCol(new_data.cbegin(), new_data.cend());

  // Check the size of the DataTable.
  checkDataTableLimits(dt, new_data.size(), 1);

  // Clear the DataTable.
  dt.clearData();
  
  // Check the size of the DataTable.
  checkDataTableLimits(dt, 0, 0);

  // Add the first col using iterators. This time provide a hint on how long
  // the col is.
  dt.addCol(new_data.cbegin(), new_data.cend(), new_data.size());

  // Check the size of the DataTable.
  checkDataTableLimits(dt, new_data.size(), 1);

  // Add another col to the DataTable using Vector.
  decltype(dt_col) new_dt_col{static_cast<int>(new_data.size()), 
                              new_data.data()};
  dt.addCol(new_dt_col);

  // Check the size of the DataTable.
  checkDataTableLimits(dt, new_data.size(), 2);

  // Check entries of the col just added.
  checkDataTableCol(dt, new_data, 1);
}


// Start with default constructed DataTable and add rows/cols to it.
void test1() {
  // Construct a DataTable of SimTK::Real(alias for double).
  std::cout << "test1 -- Default construct DataTable: Real.\n";
  OpenSim::DataTable_<SimTK::Real> dt_real{};
  // Construct a DataTable of SimTK::Vec3.
  std::cout << "test1 -- Default construct DataTable: Vec3.\n";
  OpenSim::DataTable_<SimTK::Vec3> dt_vec3{};
  // Construct a DataTable of SimTK::Vec6.
  std::cout << "test1 -- Default construct DataTable: Vec6.\n";
  OpenSim::DataTable_<SimTK::Vec6> dt_vec6{};

  // Check the size of the DataTable.
  std::cout << "test1 -- checkDataTableLimits(): Real.\n";
  checkDataTableLimits(dt_real, 0, 0);
  std::cout << "test1 -- checkDataTableLimits(): Vec3.\n";
  checkDataTableLimits(dt_vec3, 0, 0);
  std::cout << "test1 -- checkDataTableLimits(): Vec6.\n";
  checkDataTableLimits(dt_vec6, 0, 0);

  // Copy construct from default constructed DataTable.
  std::cout << "test1 -- Copy construct: Real.\n";
  decltype(dt_real) copy_dt_real{dt_real};
  std::cout << "test1 -- Copy construct: Vec3.\n";
  decltype(dt_vec3) copy_dt_vec3{dt_vec3};
  std::cout << "test1 -- Copy construct: Vec6.\n";
  decltype(dt_vec6) copy_dt_vec6{dt_vec6};

  // Check the size of the DataTable.
  std::cout << "test1 -- checkDataTableLimits(): Real.\n";
  checkDataTableLimits(copy_dt_real, 0, 0);
  std::cout << "test1 -- checkDataTableLimits(): Vec3.\n";
  checkDataTableLimits(copy_dt_vec3, 0, 0);
  std::cout << "test1 -- checkDataTableLimits(): Vec6.\n";
  checkDataTableLimits(copy_dt_vec6, 0, 0);

  // Virtual constructor.
  std::cout << "test1 -- Virtual(clone) constructor: Real.\n";
  OpenSim::AbstractDataTable& abs_dt_real{dt_real};
  auto clone_absdt_real = abs_dt_real.clone();
  auto clone_dt_real = static_cast<decltype(dt_real)&>(*clone_absdt_real);
  std::cout << "test1 -- Virtual(clone) constructor: Vec3.\n";
  OpenSim::AbstractDataTable& abs_dt_vec3{dt_vec3};
  auto clone_absdt_vec3 = abs_dt_vec3.clone();
  auto clone_dt_vec3 = static_cast<decltype(dt_vec3)&>(*clone_absdt_vec3);
  std::cout << "test1 -- Virtual(clone) constructor: Vec6.\n";
  OpenSim::AbstractDataTable& abs_dt_vec6{dt_vec6};
  auto clone_absdt_vec6 = abs_dt_vec6.clone();
  auto clone_dt_vec6 = static_cast<decltype(dt_vec6)&>(*clone_absdt_vec6);

  // Check the size of the clone DataTable.
  std::cout << "test1 -- checkDataTableLimits(): Real.\n";
  checkDataTableLimits(clone_dt_real, 0, 0);
  std::cout << "test1 -- checkDataTableLimits(): Vec3.\n";
  checkDataTableLimits(clone_dt_vec3, 0, 0);
  std::cout << "test1 -- checkDataTableLimits(): Vec6.\n";
  checkDataTableLimits(clone_dt_vec6, 0, 0);

  // Move constructors.
  std::cout << "test1 -- Move constructor: Real.\n";
  decltype(dt_real) move_dt_real{std::move(dt_real)};
  std::cout << "test1 -- Move constructor: Vec3.\n";
  decltype(dt_vec3) move_dt_vec3{std::move(dt_vec3)};
  std::cout << "test1 -- Move constructor: Vec6.\n";
  decltype(dt_vec6) move_dt_vec6{std::move(dt_vec6)};

  // Check the size of the clone DataTable.
  std::cout << "test1 -- checkDataTableLimits(): Real.\n";
  checkDataTableLimits(clone_dt_real, 0, 0);
  std::cout << "test1 -- checkDataTableLimits(): Vec3.\n";
  checkDataTableLimits(clone_dt_vec3, 0, 0);
  std::cout << "test1 -- checkDataTableLimits(): Vec6.\n";
  checkDataTableLimits(clone_dt_vec6, 0, 0);

  // Test adding rows to empty DataTable using a SimTK::RowVector. Using 
  // integers only for demostration. The underlying type can hold `double`.
  std::cout << "test1 -- testAddRow(): Real.\n";
  testAddRow(dt_real, std::vector<SimTK::Real>{1, 2, 3});
  std::cout << "test1 -- testAddRow(): Vec3.\n";
  testAddRow(dt_vec3, std::vector<SimTK::Vec3>{{1, 2, 3},
                                               {4, 5, 6},
                                               {7, 8, 9}});
  std::cout << "test1 -- testAddRow(): Vec6.\n";
  testAddRow(dt_vec6, 
             std::vector<SimTK::Vec6>{{1, 2, 3, 11, 22, 33},
                                      {4, 5, 6, 44, 55, 66},
                                      {7, 8, 9, 77, 88, 99}});


  // Test adding cols to previously populated DataTable.
  std::cout << "test1 -- testAddCol(): Real.\n";
  testAddCol(dt_real, std::vector<SimTK::Real>{1, 2});
  std::cout << "test1 -- testAddCol(): Vec3.\n";
  testAddCol(dt_vec3, std::vector<SimTK::Vec3>{{1, 2, 3},
                                               {4, 5, 6}});
  std::cout << "test1 -- testAddCol(): Vec6.\n";
  testAddCol(dt_vec6, std::vector<SimTK::Vec6>{{1, 2, 3, 11, 22, 33},
                                               {4, 5, 6, 44, 55, 66}});

  // Test adding rows to previously populated DataTable.
  std::cout << "test1 -- testAddRow(): Real.\n";
  testAddRow(dt_real, std::vector<SimTK::Real>{1, 2});
  std::cout << "test1 -- testAddRow(): Vec3.\n";
  testAddRow(dt_vec3, std::vector<SimTK::Vec3>{{1, 2, 3}, 
                                               {4, 5, 6}});
  std::cout << "test1 -- testAddRow(): Vec6.\n";
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
  std::cout << "test2 -- Construct DataTable with default value: Real.\n";
  OpenSim::DataTable_<SimTK::Real> dt_real{3, 4, 10};
  // Construct a DataTable of SimTK::Vec3.
  std::cout << "test2 -- Construct DataTable with default value: Vec3.\n";
  OpenSim::DataTable_<SimTK::Vec3> dt_vec3{3, 4, {10, 20, 30}};
  // Construct a DataTable of SimTK::Vec6.
  std::cout << "test2 -- Construct DataTable with default value: Vec6.\n"; 
  OpenSim::DataTable_<SimTK::Vec6> dt_vec6{3, 4, {10, 20, 30, 40, 50, 60}};

  // Check the size of the DataTable.
  std::cout << "test2 -- checkDataTableLimits(): Real.\n";
  checkDataTableLimits(dt_real, 3, 4);
  std::cout << "test2 -- checkDataTableLimits(): Vec3.\n";
  checkDataTableLimits(dt_vec3, 3, 4);
  std::cout << "test2 -- checkDataTableLimits(): Vec6.\n";
  checkDataTableLimits(dt_vec6, 3, 4);

  // Check the entries of few rows of the DataTable.
  std::cout << "test2 -- checkDataTableRow(row 0): Real.\n";
  checkDataTableRow(dt_real, std::vector<SimTK::Real>{10, 10, 10, 10}, 0);
  std::cout << "test2 -- checkDataTableRow(row 2): Real.\n";
  checkDataTableRow(dt_real, std::vector<SimTK::Real>{10, 10, 10, 10}, 2);
  std::cout << "test2 -- checkDataTableRow(row 0): Vec3.\n";
  checkDataTableRow(dt_vec3, std::vector<SimTK::Vec3>{{10, 20, 30},
                                                      {10, 20, 30},
                                                      {10, 20, 30},
                                                      {10, 20, 30}}, 0);
  std::cout << "test2 -- checkDataTableRow(row 2): Vec3.\n";
  checkDataTableRow(dt_vec3, std::vector<SimTK::Vec3>{{10, 20, 30},
                                                      {10, 20, 30},
                                                      {10, 20, 30},
                                                      {10, 20, 30}}, 2);
  std::cout << "test2 -- checkDataTableRow(row 0): Vec6.\n";
  checkDataTableRow(dt_vec6, 
                    std::vector<SimTK::Vec6>{{10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60}}, 0);
  std::cout << "test2 -- checkDataTableRow(row 2): Vec6.\n";
  checkDataTableRow(dt_vec6, 
                    std::vector<SimTK::Vec6>{{10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60}}, 2);

  // Copy construct from default constructed DataTable.
  std::cout << "test2 -- Copy construct: Real.\n";
  decltype(dt_real) copy_dt_real{dt_real};
  std::cout << "test2 -- Copy construct: Vec3.\n";
  decltype(dt_vec3) copy_dt_vec3{dt_vec3};
  std::cout << "test2 -- Copy construct: Vec6.\n";
  decltype(dt_vec6) copy_dt_vec6{dt_vec6};

  // Check the size of the copy constructed DataTable.
  std::cout << "test2 -- checkDataTableLimits(): Real.\n";
  checkDataTableLimits(copy_dt_real, 3, 4);
  std::cout << "test2 -- checkDataTableLimits(): Vec3.\n";
  checkDataTableLimits(copy_dt_vec3, 3, 4);
  std::cout << "test2 -- checkDataTableLimits(): Vec6.\n";
  checkDataTableLimits(copy_dt_vec6, 3, 4);

  // Check the entries of few rows of the copy constructed DataTable.
  std::cout << "test2 -- checkDataTableRow(row 0): Real.\n";
  checkDataTableRow(copy_dt_real, std::vector<SimTK::Real>{10, 10, 10, 10}, 0);
  std::cout << "test2 -- checkDataTableRow(row 2): Real.\n";
  checkDataTableRow(copy_dt_real, std::vector<SimTK::Real>{10, 10, 10, 10}, 2);
  std::cout << "test2 -- checkDataTableRow(row 0): Vec3.\n";
  checkDataTableRow(copy_dt_vec3, std::vector<SimTK::Vec3>{{10, 20, 30},
                                                           {10, 20, 30},
                                                           {10, 20, 30},
                                                           {10, 20, 30}}, 0);
  std::cout << "test2 -- checkDataTableRow(row 2): Vec3.\n";
  checkDataTableRow(copy_dt_vec3, std::vector<SimTK::Vec3>{{10, 20, 30},
                                                           {10, 20, 30},
                                                           {10, 20, 30},
                                                           {10, 20, 30}}, 2);
  std::cout << "test2 -- checkDataTableRow(row 0): Vec6.\n";
  checkDataTableRow(copy_dt_vec6, 
                    std::vector<SimTK::Vec6>{{10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60}}, 0);
  std::cout << "test2 -- checkDataTableRow(row 2): Vec6.\n";
  checkDataTableRow(copy_dt_vec6, 
                    std::vector<SimTK::Vec6>{{10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60}}, 2);

  // Virtual constructor.
  std::cout << "test2 -- Virtual(clone) constructor: Real.\n";
  OpenSim::AbstractDataTable& abs_dt_real{dt_real};
  auto clone_absdt_real = abs_dt_real.clone();
  auto clone_dt_real = static_cast<decltype(dt_real)&>(*clone_absdt_real);
  std::cout << "test2 -- Virtual(clone) constructor: Vec3.\n";
  OpenSim::AbstractDataTable& abs_dt_vec3{dt_vec3};
  auto clone_absdt_vec3 = abs_dt_vec3.clone();
  auto clone_dt_vec3 = static_cast<decltype(dt_vec3)&>(*clone_absdt_vec3);
  std::cout << "test2 -- Virtual(clone) constructor: Vec6.\n";
  OpenSim::AbstractDataTable& abs_dt_vec6{dt_vec6};
  auto clone_absdt_vec6 = abs_dt_vec6.clone();
  auto clone_dt_vec6 = static_cast<decltype(dt_vec6)&>(*clone_absdt_vec6);

  // Check the size of the clone DataTable.
  std::cout << "test2 -- checkDataTableLimits(): Real.\n";
  checkDataTableLimits(clone_dt_real, 3, 4);
  std::cout << "test2 -- checkDataTableLimits(): Vec3.\n";
  checkDataTableLimits(clone_dt_vec3, 3, 4);
  std::cout << "test2 -- checkDataTableLimits(): Vec6.\n";
  checkDataTableLimits(clone_dt_vec6, 3, 4);

  // Check the entries of few rows of the DataTable.
  std::cout << "test2 -- checkDataTableRow(row 0): Real.\n";
  checkDataTableRow(clone_dt_real, std::vector<SimTK::Real>{10, 10, 10, 10}, 0);
  std::cout << "test2 -- checkDataTableRow(row 2): Real.\n";
  checkDataTableRow(clone_dt_real, std::vector<SimTK::Real>{10, 10, 10, 10}, 2);
  std::cout << "test2 -- checkDataTableRow(row 0): Vec3.\n";
  checkDataTableRow(clone_dt_vec3, std::vector<SimTK::Vec3>{{10, 20, 30},
                                                            {10, 20, 30},
                                                            {10, 20, 30},
                                                            {10, 20, 30}}, 0);
  std::cout << "test2 -- checkDataTableRow(row 2): Vec3.\n";
  checkDataTableRow(clone_dt_vec3, std::vector<SimTK::Vec3>{{10, 20, 30},
                                                            {10, 20, 30},
                                                            {10, 20, 30},
                                                            {10, 20, 30}}, 2);
  std::cout << "test2 -- checkDataTableRow(row 0): Vec6.\n";
  checkDataTableRow(clone_dt_vec6, 
                    std::vector<SimTK::Vec6>{{10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60}}, 0);
  std::cout << "test2 -- checkDataTableRow(row 2): Vec6.\n";
  checkDataTableRow(clone_dt_vec6, 
                    std::vector<SimTK::Vec6>{{10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60}}, 2);

  // Move constructor.
  std::cout << "test2 -- Move constructor: Real.\n";
  decltype(dt_real) move_dt_real{std::move(dt_real)};
  std::cout << "test2 -- Move constructor: Vec3.\n";
  decltype(dt_vec3) move_dt_vec3{std::move(dt_vec3)};
  std::cout << "test2 -- Move constructor: Vec6.\n";
  decltype(dt_vec6) move_dt_vec6{std::move(dt_vec6)};

  // Check the size of the DataTable created.
  std::cout << "test2 -- checkDataTableLimits(): Real.\n";
  checkDataTableLimits(move_dt_real, 3, 4);
  std::cout << "test2 -- checkDataTableLimits(): Vec3.\n";
  checkDataTableLimits(move_dt_vec3, 3, 4);
  std::cout << "test2 -- checkDataTableLimits(): Vec6.\n";
  checkDataTableLimits(move_dt_vec6, 3, 4);

  // Check the entries of few rows of the DataTable.
  std::cout << "test2 -- checkDataTableRow(row 0): Real.\n";
  checkDataTableRow(move_dt_real, std::vector<SimTK::Real>{10, 10, 10, 10}, 0);
  std::cout << "test2 -- checkDataTableRow(row 2): Real.\n";
  checkDataTableRow(move_dt_real, std::vector<SimTK::Real>{10, 10, 10, 10}, 2);
  std::cout << "test2 -- checkDataTableRow(row 0): Vec3.\n";
  checkDataTableRow(move_dt_vec3, std::vector<SimTK::Vec3>{{10, 20, 30},
                                                           {10, 20, 30},
                                                           {10, 20, 30},
                                                           {10, 20, 30}}, 0);
  std::cout << "test2 -- checkDataTableRow(row 2): Vec3.\n";
  checkDataTableRow(move_dt_vec3, std::vector<SimTK::Vec3>{{10, 20, 30},
                                                           {10, 20, 30},
                                                           {10, 20, 30},
                                                           {10, 20, 30}}, 2);
  std::cout << "test2 -- checkDataTableRow(row 0): Vec6.\n";
  checkDataTableRow(move_dt_vec6, 
                    std::vector<SimTK::Vec6>{{10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60}}, 0);
  std::cout << "test2 -- checkDataTableRow(row 2): Vec6.\n";
  checkDataTableRow(move_dt_vec6, 
                    std::vector<SimTK::Vec6>{{10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60},
                                             {10, 20, 30, 40, 50, 60}}, 2);

  // Test adding rows to the DataTable.
  std::cout << "test2 -- testAddRow(): Real.\n";
  testAddRow(dt_real, std::vector<SimTK::Real>{1, 2, 3, 4});
  std::cout << "test2 -- testAddRow(): Vec3.\n";
  testAddRow(dt_vec3, std::vector<SimTK::Vec3>{{ 1,  2,  3}, 
                                               { 4,  5,  6},
                                               { 7,  8,  9},
                                               {10, 11, 12}});
  std::cout << "test2 -- testAddRow(): Vec6.\n";
  testAddRow(dt_vec6, 
             std::vector<SimTK::Vec6>{{ 1,  2,  3,  11,  22,  33},
                                      { 4,  5,  6,  44,  55,  66},
                                      { 7,  8,  9,  77,  88,  99},
                                      {10, 11, 12, 110, 120, 140}});

  // Test adding cols to previously populated DataTable.
  std::cout << "test2 -- testAddCol(): Real.\n";
  testAddCol(dt_real, std::vector<SimTK::Real>{1, 2});
  std::cout << "test2 -- testAddCol(): Vec3.\n";
  testAddCol(dt_vec3, std::vector<SimTK::Vec3>{{1, 2, 3},
                                               {4, 5, 6}});
  std::cout << "test2 -- testAddCol(): Vec6.\n";
  testAddCol(dt_vec6, std::vector<SimTK::Vec6>{{1, 2, 3, 11, 22, 33},
                                               {4, 5, 6, 44, 55, 66}});

  // Test adding rows to previously populated DataTable.
  std::cout << "test2 -- testAddRow(): Real.\n";
  testAddRow(dt_real, std::vector<SimTK::Real>{1, 2});
  std::cout << "test2 -- testAddRow(): Vec3.\n";
  testAddRow(dt_vec3, std::vector<SimTK::Vec3>{{1, 2, 3}, 
                                               {4, 5, 6}});
  std::cout << "test2 -- testAddRow(): Vec6.\n";
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
  // Construct a DataTable using iteratorsl. Using integers for demostration. 
  // The underlyig type can hold double.
  std::cout << "test3 -- Construct DataTable with iterators colwise: Real.\n";
  OpenSim::DataTable_<SimTK::Real> dt_real{data_real.cbegin(), 
                                           data_real.cend(),
                                           4,
                                           OpenSim::COLWISE};
  std::cout << "test3 -- Construct DataTable with iterators colwise: Vec3.\n";
  OpenSim::DataTable_<SimTK::Vec3> dt_vec3{data_vec3.cbegin(),
                                           data_vec3.cend(),
                                           4,
                                           OpenSim::COLWISE};
  std::cout << "test3 -- Construct DataTable with iterators colwise: Vec6.\n";
  OpenSim::DataTable_<SimTK::Vec6> dt_vec6{data_vec6.cbegin(),
                                           data_vec6.cend(), 
                                           4,
                                           OpenSim::COLWISE};

  // Check the size of the DataTable.
  std::cout << "test3 -- checkDataTableLimits(): Real.\n";
  checkDataTableLimits(dt_real, 4, 3);
  std::cout << "test3 -- checkDataTableLimits(): Vec3.\n";
  checkDataTableLimits(dt_vec3, 4, 3);
  std::cout << "test3 -- checkDataTableLimits(): Vec6.\n";
  checkDataTableLimits(dt_vec6, 4, 3);

  // Check the entries of few cows of the DataTable.
  std::cout << "test3 -- checkDataTableCol(col 0): Real.\n";
  checkDataTableCol(dt_real, decltype(data_real){data_real.cbegin(),
                                                 data_real.cbegin() + 4}, 0);
  std::cout << "test3 -- checkDataTableCol(col 2): Real.\n";
  checkDataTableCol(dt_real, decltype(data_real){data_real.cend() - 4,
                                                 data_real.cend()}, 2);
  std::cout << "test3 -- checkDataTableCol(col 0): Vec3.\n";
  checkDataTableCol(dt_vec3, decltype(data_vec3){data_vec3.cbegin(),
                                                 data_vec3.cbegin() + 4}, 0);
  std::cout << "test3 -- checkDataTableCol(col 2): Vec3.\n";
  checkDataTableCol(dt_vec3, decltype(data_vec3){data_vec3.cend() - 4,
                                                 data_vec3.cend()}, 2);
  std::cout << "test3 -- checkDataTableCol(col 0): Vec6.\n";
  checkDataTableCol(dt_vec6, decltype(data_vec6){data_vec6.cbegin(),
                                                 data_vec6.cbegin() + 4}, 0);
  std::cout << "test3 -- checkDataTableCol(col 2): Vec6.\n";
  checkDataTableCol(dt_vec6, decltype(data_vec6){data_vec6.cend() - 4,
                                                 data_vec6.cend()}, 2);
  }

  // Construct a DataTable using iteratorsl. Using integers for demostration. 
  // The underlyig type can hold double.
  std::cout << "test3 -- Construct DataTable with iterators rowwise: Real.\n";
  OpenSim::DataTable_<SimTK::Real> dt_real{data_real.cbegin(), 
                                           data_real.cend(),
                                           4};
  std::cout << "test3 -- Construct DataTable with iterators rowwise: Vec3.\n";
  OpenSim::DataTable_<SimTK::Vec3> dt_vec3{data_vec3.cbegin(),
                                           data_vec3.cend(),
                                           4};
  std::cout << "test3 -- Construct DataTable with iterators rowwise: Vec6.\n";
  OpenSim::DataTable_<SimTK::Vec6> dt_vec6{data_vec6.cbegin(),
                                           data_vec6.cend(), 
                                           4};

  // Check the size of the DataTable.
  std::cout << "test3 -- checkDataTableLimits(): Real.\n";
  checkDataTableLimits(dt_real, 3, 4);
  std::cout << "test3 -- checkDataTableLimits(): Vec3.\n";
  checkDataTableLimits(dt_vec3, 3, 4);
  std::cout << "test3 -- checkDataTableLimits(): Vec6.\n";
  checkDataTableLimits(dt_vec6, 3, 4);

  // Check the entries of few rows of the DataTable.
  std::cout << "test3 -- checkDataTableRow(row 0): Real.\n";
  checkDataTableRow(dt_real, decltype(data_real){data_real.cbegin(),
                                                 data_real.cbegin() + 4}, 0);
  std::cout << "test3 -- checkDataTableRow(row 2): Real.\n";
  checkDataTableRow(dt_real, decltype(data_real){data_real.cend() - 4,
                                                 data_real.cend()}, 2);
  std::cout << "test3 -- checkDataTableRow(row 0): Vec3.\n";
  checkDataTableRow(dt_vec3, decltype(data_vec3){data_vec3.cbegin(),
                                                 data_vec3.cbegin() + 4}, 0);
  std::cout << "test3 -- checkDataTableRow(row 2): Vec3.\n";
  checkDataTableRow(dt_vec3, decltype(data_vec3){data_vec3.cend() - 4,
                                                 data_vec3.cend()}, 2);
  std::cout << "test3 -- checkDataTableRow(row 0): Vec6.\n";
  checkDataTableRow(dt_vec6, decltype(data_vec6){data_vec6.cbegin(),
                                                 data_vec6.cbegin() + 4}, 0);
  std::cout << "test3 -- checkDataTableRow(row 2): Vec6.\n";
  checkDataTableRow(dt_vec6, decltype(data_vec6){data_vec6.cend() - 4,
                                                 data_vec6.cend()}, 2);

  
  // Copy construct from default constructed DataTable.
  std::cout << "test3 -- Copy construct: Real.\n";
  decltype(dt_real) copy_dt_real{dt_real};
  std::cout << "test3 -- Copy construct: Vec3.\n";
  decltype(dt_vec3) copy_dt_vec3{dt_vec3};
  std::cout << "test3 -- Copy construct: Vec6.\n";
  decltype(dt_vec6) copy_dt_vec6{dt_vec6};

  // Check the size of the DataTable.
  std::cout << "test3 -- checkDataTableLimits(): Real.\n";
  checkDataTableLimits(dt_real, 3, 4);
  std::cout << "test3 -- checkDataTableLimits(): Vec3.\n";
  checkDataTableLimits(dt_vec3, 3, 4);
  std::cout << "test3 -- checkDataTableLimits(): Vec6.\n";
  checkDataTableLimits(dt_vec6, 3, 4);

  // Check the entries of few rows of the DataTable.
  std::cout << "test3 -- checkDataTableRow(row 0): Real.\n";
  checkDataTableRow(copy_dt_real, 
                    decltype(data_real){data_real.cbegin(), 
                                        data_real.cbegin() + 4}, 0);
  std::cout << "test3 -- checkDataTableRow(row 2): Real.\n";
  checkDataTableRow(copy_dt_real, 
                    decltype(data_real){data_real.cend() - 4,
                                        data_real.cend()}, 2);
  std::cout << "test3 -- checkDataTableRow(row 0): Vec3.\n";
  checkDataTableRow(copy_dt_vec3, 
                    decltype(data_vec3){data_vec3.cbegin(),
                                        data_vec3.cbegin() + 4}, 0);
  std::cout << "test3 -- checkDataTableRow(row 2): Vec3.\n";
  checkDataTableRow(copy_dt_vec3, 
                    decltype(data_vec3){data_vec3.cend() - 4,
                                        data_vec3.cend()}, 2);
  std::cout << "test3 -- checkDataTableRow(row 0): Vec6.\n";
  checkDataTableRow(copy_dt_vec6, 
                    decltype(data_vec6){data_vec6.cbegin(),
                                        data_vec6.cbegin() + 4}, 0);
  std::cout << "test3 -- checkDataTableRow(row 2): Vec6.\n";
  checkDataTableRow(copy_dt_vec6, 
                    decltype(data_vec6){data_vec6.cend() - 4,
                                        data_vec6.cend()}, 2);

  // Virtual constructor.
  std::cout << "test3 -- Virtual(clone) constructor: Real.\n";
  OpenSim::AbstractDataTable& abs_dt_real{dt_real};
  auto clone_absdt_real = abs_dt_real.clone();
  auto clone_dt_real = static_cast<decltype(dt_real)&>(*clone_absdt_real);
  std::cout << "test3 -- Virtual(clone) constructor: Vec3.\n";
  OpenSim::AbstractDataTable& abs_dt_vec3{dt_vec3};
  auto clone_absdt_vec3 = abs_dt_vec3.clone();
  auto clone_dt_vec3 = static_cast<decltype(dt_vec3)&>(*clone_absdt_vec3);
  std::cout << "test3 -- Virtual(clone) constructor: Vec6.\n";
  OpenSim::AbstractDataTable& abs_dt_vec6{dt_vec6};
  auto clone_absdt_vec6 = abs_dt_vec6.clone();
  auto clone_dt_vec6 = static_cast<decltype(dt_vec6)&>(*clone_absdt_vec6);

  // Check the size of the DataTable.
  std::cout << "test3 -- checkDataTableLimits(): Real.\n";
  checkDataTableLimits(dt_real, 3, 4);
  std::cout << "test3 -- checkDataTableLimits(): Vec3.\n";
  checkDataTableLimits(dt_vec3, 3, 4);
  std::cout << "test3 -- checkDataTableLimits(): Vec6.\n";
  checkDataTableLimits(dt_vec6, 3, 4);

  // Check the entries of few rows of the DataTable.
  std::cout << "test3 -- checkDataTableRow(row 0): Real.\n";
  checkDataTableRow(clone_dt_real, 
                    decltype(data_real){data_real.cbegin(), 
                                        data_real.cbegin() + 4}, 0);
  std::cout << "test3 -- checkDataTableRow(row 2): Real.\n";
  checkDataTableRow(clone_dt_real, 
                    decltype(data_real){data_real.cend() - 4,
                                        data_real.cend()}, 2);
  std::cout << "test3 -- checkDataTableRow(row 0): Vec3.\n";
  checkDataTableRow(clone_dt_vec3, 
                    decltype(data_vec3){data_vec3.cbegin(),
                                        data_vec3.cbegin() + 4}, 0);
  std::cout << "test3 -- checkDataTableRow(row 2): Vec3.\n";
  checkDataTableRow(clone_dt_vec3, 
                    decltype(data_vec3){data_vec3.cend() - 4,
                                        data_vec3.cend()}, 2);
  std::cout << "test3 -- checkDataTableRow(row 0): Vec6.\n";
  checkDataTableRow(clone_dt_vec6, 
                    decltype(data_vec6){data_vec6.cbegin(),
                                        data_vec6.cbegin() + 4}, 0);
  std::cout << "test3 -- checkDataTableRow(row 2): Vec6.\n";
  checkDataTableRow(clone_dt_vec6, 
                    decltype(data_vec6){data_vec6.cend() - 4,
                                        data_vec6.cend()}, 2);

  // Move constructor.
  std::cout << "test3 -- Move constructor: Real.\n";
  decltype(dt_real) move_dt_real{std::move(dt_real)};
  std::cout << "test3 -- Move constructor: Vec3.\n";
  decltype(dt_vec3) move_dt_vec3{std::move(dt_vec3)};
  std::cout << "test3 -- Move constructor: Vec6.\n";
  decltype(dt_vec6) move_dt_vec6{std::move(dt_vec6)};

  // Check the size of the DataTable.
  std::cout << "test3 -- checkDataTableLimits(): Real.\n";
  checkDataTableLimits(dt_real, 3, 4);
  std::cout << "test3 -- checkDataTableLimits(): Vec3.\n";
  checkDataTableLimits(dt_vec3, 3, 4);
  std::cout << "test3 -- checkDataTableLimits(): Vec6.\n";
  checkDataTableLimits(dt_vec6, 3, 4);

  // Check the entries of few rows of the DataTable.
  std::cout << "test3 -- checkDataTableRow(row 0): Real.\n";
  checkDataTableRow(clone_dt_real, 
                    decltype(data_real){data_real.cbegin(), 
                                        data_real.cbegin() + 4}, 0);
  std::cout << "test3 -- checkDataTableRow(row 2): Real.\n";
  checkDataTableRow(clone_dt_real, 
                    decltype(data_real){data_real.cend() - 4,
                                        data_real.cend()}, 2);
  std::cout << "test3 -- checkDataTableRow(row 0): Vec3.\n";
  checkDataTableRow(clone_dt_vec3, 
                    decltype(data_vec3){data_vec3.cbegin(),
                                        data_vec3.cbegin() + 4}, 0);
  std::cout << "test3 -- checkDataTableRow(row 2): Vec3.\n";
  checkDataTableRow(clone_dt_vec3, 
                    decltype(data_vec3){data_vec3.cend() - 4,
                                        data_vec3.cend()}, 2);
  std::cout << "test3 -- checkDataTableRow(row 0): Vec6.\n";
  checkDataTableRow(clone_dt_vec6, 
                    decltype(data_vec6){data_vec6.cbegin(),
                                        data_vec6.cbegin() + 4}, 0);
  std::cout << "test3 -- checkDataTableRow(row 2): Vec6.\n";
  checkDataTableRow(clone_dt_vec6, 
                    decltype(data_vec6){data_vec6.cend() - 4,
                                        data_vec6.cend()}, 2);

  // Test adding rows to the DataTable.
  std::cout << "test3 -- testAddRow(): Real.\n";
  testAddRow(dt_real, std::vector<SimTK::Real>{1, 2, 3, 4});
  std::cout << "test3 -- testAddRow(): Vec3.\n";
  testAddRow(dt_vec3, std::vector<SimTK::Vec3>{{ 1,  2,  3}, 
                                               { 4,  5,  6},
                                               { 7,  8,  9},
                                               {10, 11, 12}});
  std::cout << "test3 -- testAddRow(): Vec6.\n";
  testAddRow(dt_vec6, 
             std::vector<SimTK::Vec6>{{ 1,  2,  3,  11,  22,  33},
                                      { 4,  5,  6,  44,  55,  66},
                                      { 7,  8,  9,  77,  88,  99},
                                      {10, 11, 12, 110, 120, 140}});

  // Test adding cols to previously populated DataTable.
  std::cout << "test3 -- testAddCol(): Real.\n";
  testAddCol(dt_real, std::vector<SimTK::Real>{1, 2});
  std::cout << "test3 -- testAddCol(): Vec3.\n";
  testAddCol(dt_vec3, std::vector<SimTK::Vec3>{{1, 2, 3},
                                               {4, 5, 6}});
  std::cout << "test3 -- testAddCol(): Vec6.\n";
  testAddCol(dt_vec6, std::vector<SimTK::Vec6>{{1, 2, 3, 11, 22, 33},
                                               {4, 5, 6, 44, 55, 66}});

  // Test adding rows to previously populated DataTable.
  std::cout << "test3 -- testAddRow(): Real.\n";
  testAddRow(dt_real, std::vector<SimTK::Real>{1, 2});
  std::cout << "test3 -- testAddRow(): Vec3.\n";
  testAddRow(dt_vec3, std::vector<SimTK::Vec3>{{1, 2, 3}, 
                                               {4, 5, 6}});
  std::cout << "test3 -- testAddRow(): Vec6.\n";
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
  std::cout << "test4 -- Constructor DataTable 1: Real.\n";
  OpenSim::DataTable_<SimTK::Real> dt1_real{3, 4, 10};
  std::cout << "test4 -- Constructor DataTable 1: Vec3.\n";
  OpenSim::DataTable_<SimTK::Vec3> dt1_vec3{3, 4, {10, 20, 30}};
  std::cout << "test4 -- Constructor DataTable 1: Vec6.\n";
  OpenSim::DataTable_<SimTK::Vec6> dt1_vec6{3, 4, {10, 20, 30, 40, 50, 60}};

  // Construct DataTable 2 with SimtTK::Real
  std::cout << "test4 -- Construct DataTable 2: Real.\n";
  OpenSim::DataTable_<SimTK::Real> dt2_real{data_real.cbegin(), 
                                           data_real.cend(),
                                           3,
                                           OpenSim::COLWISE};
  std::cout << "test4 -- Construct DataTable 2: Vec3.\n";
  OpenSim::DataTable_<SimTK::Vec3> dt2_vec3{data_vec3.cbegin(),
                                           data_vec3.cend(),
                                           3,
                                           OpenSim::COLWISE};
  std::cout << "test4 -- Construct DataTable 2: Vec6.\n";
  OpenSim::DataTable_<SimTK::Vec6> dt2_vec6{data_vec6.cbegin(),
                                           data_vec6.cend(), 
                                           3,
                                           OpenSim::COLWISE};
  
  // Bind DataTable 2 to DataTable 1 by row.
  std::cout << "test4 -- dt1.rbindDataTable(dt2): Real.\n";
  dt1_real.rbindDataTable(dt2_real);
  std::cout << "test4 -- dt1.rbindDataTable(dt2): Vec3.\n";
  dt1_vec3.rbindDataTable(dt2_vec3);
  std::cout << "test4 -- dt1.rbindDataTable(dt2): Vec6.\n";
  dt1_vec6.rbindDataTable(dt2_vec6);


  // Check the size of the DataTable.
  std::cout << "test4 -- checkDataTableLimits(): Real.\n";
  checkDataTableLimits(dt1_real, 6, 4);
  std::cout << "test4 -- checkDataTableLimits(): Vec3.\n";
  checkDataTableLimits(dt1_vec3, 6, 4);
  std::cout << "test4 -- checkDataTableLimits(): Vec6.\n";
  checkDataTableLimits(dt1_vec6, 6, 4);
 
  // Check the entries of the DataTable.
  std::cout << "test4 -- checkDataTableCol(col 0): Real.\n";
  checkDataTableCol(dt1_real, 
                    decltype(data_real){10, 10, 10, 100, 100, 100}, 0);
  std::cout << "test4 -- checkDataTableCol(col 2): Real.\n";
  checkDataTableCol(dt1_real, 
                    decltype(data_real){10, 10, 10, 100, 100, 100}, 2);
  std::cout << "test4 -- checkDataTableCol(col 0): Vec3.\n";
  checkDataTableCol(dt1_vec3, decltype(data_vec3){{ 10,  20,  30},
                                                  { 10,  20,  30},
                                                  { 10,  20,  30},
                                                  {100, 200, 300},
                                                  {100, 200, 300},
                                                  {100, 200, 300}}, 0);
  std::cout << "test4 -- checkDataTableCol(col 2): Vec3.\n";
  checkDataTableCol(dt1_vec3, decltype(data_vec3){{ 10,  20,  30},
                                                  { 10,  20,  30},
                                                  { 10,  20,  30},
                                                  {100, 200, 300},
                                                  {100, 200, 300},
                                                  {100, 200, 300}}, 2);
  std::cout << "test4 -- checkDataTableCol(col 0): Vec6.\n";
  checkDataTableCol(dt1_vec6, decltype(data_vec6){{10, 20, 30, 40, 50, 60},
                                                  {10, 20, 30, 40, 50, 60},
                                                  {10, 20, 30, 40, 50, 60},
                                            {100, 200, 300, 400, 500, 600},
                                            {100, 200, 300, 400, 500, 600},
                                            {100, 200, 300, 400, 500, 600}}, 0);
  std::cout << "test4 -- checkDataTableCol(col 2): Vec6.\n";
  checkDataTableCol(dt1_vec6, decltype(data_vec6){{10, 20, 30, 40, 50, 60},
                                                  {10, 20, 30, 40, 50, 60},
                                                  {10, 20, 30, 40, 50, 60},
                                            {100, 200, 300, 400, 500, 600},
                                            {100, 200, 300, 400, 500, 600},
                                            {100, 200, 300, 400, 500, 600}}, 2);

  // Try binding DataTable 1 to DataTable 2 by col.
  std::cout << "test4 -- dt2.cbindDataTable(dt1): Real.\n";
  try {
    dt2_real.cbindDataTable(dt1_real);
  } catch(OpenSim::InvalidEntry&) {}
  std::cout << "test4 -- dt2.cbindDataTable(dt1): Vec3.\n";
  try {
    dt2_vec3.cbindDataTable(dt1_vec3);
  } catch(OpenSim::InvalidEntry&) {}
  std::cout << "test4 -- dt2.cbindDataTable(dt1): Vec6.\n";
  try {
    dt2_vec6.cbindDataTable(dt1_vec6);
  } catch(OpenSim::InvalidEntry&) {}
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
  OpenSim::DataTable_<SimTK::Real> dt2_real{data_real.cbegin(), 
                                           data_real.cend(),
                                           3,
                                           OpenSim::COLWISE};
  std::cout << "test4 -- Construct DataTable 2: Vec3.\n";
  OpenSim::DataTable_<SimTK::Vec3> dt2_vec3{data_vec3.cbegin(),
                                           data_vec3.cend(),
                                           3,
                                           OpenSim::COLWISE};
  std::cout << "test4 -- Construct DataTable 2: Vec6.\n";
  OpenSim::DataTable_<SimTK::Vec6> dt2_vec6{data_vec6.cbegin(),
                                           data_vec6.cend(), 
                                           3,
                                           OpenSim::COLWISE};
  // Bind DataTable 2 to DataTable 1 by col.
  std::cout << "test4 -- dt1.cbindDataTable(dt2): Real.\n";
  dt1_real.cbindDataTable(dt2_real);
  std::cout << "test4 -- dt1.cbindDataTable(dt2): Vec3.\n";
  dt1_vec3.cbindDataTable(dt2_vec3);
  std::cout << "test4 -- dt1.cbindDataTable(dt2): Vec6.\n";
  dt1_vec6.cbindDataTable(dt2_vec6);

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
                    decltype(data_real){10, 10, 10, 10, 100, 100, 100, 100}, 0);
  std::cout << "test4 -- checkDataTableRow(row 2): Real.\n";
  checkDataTableRow(dt1_real, 
                    decltype(data_real){10, 10, 10, 10, 100, 100, 100, 100}, 2);
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
  checkDataTableRow(dt1_vec6, decltype(data_vec6){{10, 20, 30, 40, 50, 60},
                                                  {10, 20, 30, 40, 50, 60},
                                                  {10, 20, 30, 40, 50, 60},
                                                  {10, 20, 30, 40, 50, 60},
                                            {100, 200, 300, 400, 500, 600},
                                            {100, 200, 300, 400, 500, 600},
                                            {100, 200, 300, 400, 500, 600},
                                            {100, 200, 300, 400, 500, 600}}, 0);
  std::cout << "test4 -- checkDataTableRow(row 2): Vec6.\n";
  checkDataTableRow(dt1_vec6, decltype(data_vec6){{10, 20, 30, 40, 50, 60},
                                                  {10, 20, 30, 40, 50, 60},
                                                  {10, 20, 30, 40, 50, 60},
                                                  {10, 20, 30, 40, 50, 60},
                                            {100, 200, 300, 400, 500, 600},
                                            {100, 200, 300, 400, 500, 600},
                                            {100, 200, 300, 400, 500, 600},
                                            {100, 200, 300, 400, 500, 600}}, 2);

  // Try binding DataTable 1 to DataTable 2 by row.
  std::cout << "test4 -- dt2.rbindDataTable(dt1): Real.\n";
  try {
    dt2_real.rbindDataTable(dt1_real);
  } catch(OpenSim::InvalidEntry&) {}
  std::cout << "test4 -- dt2.rbindDataTable(dt1): Vec3.\n";
  try {
    dt2_vec3.rbindDataTable(dt1_vec3);
  } catch(OpenSim::InvalidEntry&) {}
  std::cout << "test4 -- dt2.rbindDataTable(dt1): Vec6.\n";
  try {
    dt2_vec6.rbindDataTable(dt1_vec6);
  } catch(OpenSim::InvalidEntry&) {}

  // Try binding DataTable 2 to itself by row.
  std::cout << "test4 -- dt2.rbindDataTable(dt2): Real.\n";
  try {
    dt2_real.rbindDataTable(dt2_real);
  } catch(OpenSim::InvalidEntry&) {}
  std::cout << "test4 -- dt2.rbindDataTable(dt2): Vec3.\n";
  try {
    dt2_vec3.rbindDataTable(dt2_vec3);
  } catch(OpenSim::InvalidEntry&) {}
  std::cout << "test4 -- dt2.rbindDataTable(dt2): Vec6.\n";
  try {
    dt2_vec6.rbindDataTable(dt2_vec6);
  } catch(OpenSim::InvalidEntry&) {}

  // Try binding DataTable 2 to itself by col.
  std::cout << "test4 -- dt2.cbindDataTable(dt2): Real.\n";
  try {
    dt2_real.cbindDataTable(dt2_real);
  } catch(OpenSim::InvalidEntry&) {}
  std::cout << "test4 -- dt2.cbindDataTable(dt2): Vec3.\n";
  try {
    dt2_vec3.cbindDataTable(dt2_vec3);
  } catch(OpenSim::InvalidEntry&) {}
  std::cout << "test4 -- dt2.cbindDataTable(dt2): Vec6.\n";
  try {
    dt2_vec6.cbindDataTable(dt2_vec6);
  } catch(OpenSim::InvalidEntry&) {}
}


// Test colum label interface.
void test5() {
  // Construct DataTable.
  std::cout << "test4 -- Constructor DataTable 1: Real.\n";
  OpenSim::DataTable_<SimTK::Real> dt_real{3, 4, 10};
  std::cout << "test4 -- Constructor DataTable 1: Vec3.\n";
  OpenSim::DataTable_<SimTK::Vec3> dt_vec3{3, 4, {10, 20, 30}};
  std::cout << "test4 -- Constructor DataTable 1: Vec6.\n";
  OpenSim::DataTable_<SimTK::Vec6> dt_vec6{3, 4, {10, 20, 30, 40, 50, 60}};

  dt_real.colHasLabel()

}


int main() {
  test1();

  test2();

  test3();

  test4();

  test5();

  return 0;
}
