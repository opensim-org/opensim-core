/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Testing.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2026 Stanford University and the Authors                *
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

#include "Testing.h"

using namespace OpenSim;

void CHECK_STORAGE_AGAINST_STANDARD(const OpenSim::Storage& result,
        const OpenSim::Storage& standard, const std::vector<double>& tolerances,
        const std::string& testFile, const int testFileLine,
        const std::string& errorMessage) {

    std::vector<std::string> columnsUsed;
    std::vector<double> comparisons;
    result.compareWithStandard(standard, columnsUsed, comparisons);

    size_t ncolumns = columnsUsed.size();

    ASSERT(ncolumns > 0, testFile, testFileLine,
           errorMessage + "- no common columns to compare!");

    for (size_t i = 0; i < ncolumns; ++i) {
        std::cout << "column:    " << columnsUsed[i] << std::endl;
        std::cout << "RMS error: " << comparisons[i] << std::endl;
        std::cout << "tolerance: " << tolerances[i] << std::endl << std::endl;
        ASSERT(comparisons[i] < tolerances[i], testFile, testFileLine,
               errorMessage);
    }
}

OpenSim::Object* OpenSim::Testing::randomize(OpenSim::Object* obj) {
    using namespace std;

    if (obj==nullptr) return 0; // maybe empty tag
    std::stringstream stream;
    stream << rand();
    obj->setName(obj->getConcreteClassName()+stream.str());
    // Cycle thru properties and based on type, populate with a random valid
    // value.
     for (int p=0; p < obj->getNumProperties(); ++p) {
        AbstractProperty& ap = obj->updPropertyByIndex(p);
        bool isList = ap.isListProperty();
        string ts = ap.getTypeName();
        if (ap.isOptionalProperty())
            continue;
        if (ts == "bool"&& !isList)
            ap.updValue<bool>() = (rand() % 2 == 0);
        else if (ts == "integer"&& !isList)
            ap.updValue<int>() = rand();
        else if (ts == "double" && !isList)
            ap.updValue<double>() = (double)rand()/RAND_MAX;
        else if (ts == "Vec3" && !isList) {
            Property<SimTK::Vec3>& prop = Property<SimTK::Vec3>::updAs(ap);
            prop = SimTK::Vec3(abs(rand()), abs(rand()), abs(rand()));
        } else if (ts == "Vec6" && !isList) {
            // Only property that uses a Vec6 is the inertia property
            // Might as well select valid inertias for the purpose of testing
            Property<SimTK::Vec6>& prop = Property<SimTK::Vec6>::updAs(ap);
            double Ixx = abs(rand());
            double Ixy = 0.01*Ixx;
            prop = SimTK::Vec6(Ixx, Ixx, Ixx, Ixy, Ixy, Ixy);
        } else if (ts == "string") {
            // We cannot use an arbitrary string for ExpressionBasedBushingForce
            // properties since they must contain specific variable names (e.g.,
            // "theta_x", "delta_x", etc.).
            if (obj->getConcreteClassName() != "ExpressionBasedBushingForce") {
                string base("ABCXYZ");
                if (isList) {
                    stringstream val;
                    val << base << "_" << ap.size();
                    ap.appendValue<string>(val.str());
                } else {
                    ap.updValue<string>() = base;
                }
            }
        } else if (ts == "double" && isList && ap.getMaxListSize() < 20) {
            for (int i=0; i< ap.getMaxListSize(); ++i)
                ap.updValue<double>(i) = (double) rand() / RAND_MAX;
        } else if (ts == "Function") {
            //FunctionSet's objects getTypeName() returns "Function"
            //which is wrong! This is a HACK to test that we aren't
            //treating the PropertyObjArray<Function> as a Function.
            PropertyObjArray<Function>* propObjArray =
                dynamic_cast<PropertyObjArray<Function>*>(&ap);
            if (propObjArray){
                if (propObjArray->size()){
                    randomize(&(propObjArray->updValueAsObject(0)));
                }
            }
            else{
                Property<Function>& prop = Property<Function>::updAs(ap);
                LinearFunction f;
                randomize(&f);
                prop = f;
            }

        } else if (ap.isObjectProperty() && !isList) {
            randomize(&ap.updValueAsObject(0));
            if (ap.isUnnamedProperty())
                ap.updValueAsObject(0).setName("");
            ap.setValueIsDefault(false);
        } else {
            //cerr << "Unrecognized Property:"<< ap.getName()<< ":"
            //     << ap.toString() << endl;
        }
     }
     return obj;
}

bool OpenSim::Testing::revertToVersionNumber1(const std::string& filenameOld,
        const std::string& filenameNew) {

    std::regex versionline{ R"([ \t]*version[ \t]*=[ \t]*2[ \t]*)" };
    std::ifstream fileOld{ filenameOld };
    std::ofstream fileNew{ filenameNew };
    std::string line{};
    bool changedVersion{false};
    while (std::getline(fileOld, line)) {
        if (std::regex_match(line, versionline)) {
            fileNew << "version=1\n";
            changedVersion = true;
        } else
            fileNew << line << "\n";
    }
    return changedVersion;
}

SimTK::Vector OpenSim::Testing::interp(
        const OpenSim::TimeSeriesTable& actualTable,
        const OpenSim::TimeSeriesTable& expectedTable,
        const std::string& expectedColumnLabel) {

    const auto& actualTime = actualTable.getIndependentColumn();
    // Interpolate the expected values based on `actual`'s time.
    const auto& expectedTime = expectedTable.getIndependentColumn();
    const auto& expectedCol =
            expectedTable.getDependentColumn(expectedColumnLabel);
    // Create a linear function for interpolation.
    OpenSim::PiecewiseLinearFunction expectedFunc(
        (int)expectedTable.getNumRows(), expectedTime.data(), &expectedCol[0]);
    SimTK::Vector expected((int)actualTable.getNumRows());
    for (int i = 0; i < (int)actualTable.getNumRows(); ++i) {
        const auto& time = actualTime[i];
        expected[i] = expectedFunc.calcValue(SimTK::Vector(1, time));
    }
    return expected;
};

void OpenSim::Testing::compare(const OpenSim::TimeSeriesTable& actualTable,
        const std::string& actualColumnLabel,
        const OpenSim::TimeSeriesTable& expectedTable,
        const std::string& expectedColumnLabel,
        double tol, bool verbose) {
    const auto& actual = actualTable.getDependentColumn(actualColumnLabel);
    SimTK::Vector expected = interp(actualTable, expectedTable,
                                    expectedColumnLabel);
    if (verbose) {
        std::cout << "Comparing " << expectedColumnLabel << std::endl;
        for (int i = 0; i < (int)actualTable.getNumRows(); ++i) {
            std::cout << actual[i] << " " << expected[i] << " "
                    << SimTK::isNumericallyEqual(actual[i], expected[i], tol)
                    << std::endl;
        }
    }
    SimTK_TEST_EQ_TOL(actual, expected, tol);
};

void OpenSim::Testing::rootMeanSquare(
        const OpenSim::TimeSeriesTable& actualTable,
        const std::string& actualColumnLabel,
        const OpenSim::TimeSeriesTable& expectedTable,
        const std::string& expectedColumnLabel,
        double tol, bool verbose) {
    const auto& actual = actualTable.getDependentColumn(actualColumnLabel);
    SimTK::Vector expected = interp(actualTable, expectedTable,
                                    expectedColumnLabel);
    const auto rmsError = (actual - expected).normRMS();
    if (verbose) {
        std::cout << "Comparing " << expectedColumnLabel << std::endl;
        for (int i = 0; i < actual.size(); ++i) {
            std::cout << actual[i] << " " << expected[i] << std::endl;
        }
        std::cout << "RMS error: " << rmsError << std::endl;
    }
    SimTK_TEST(rmsError < tol);
};
