#ifndef OPENSIM_AUXILIARY_TEST_FUNCTIONS_H_
#define OPENSIM_AUXILIARY_TEST_FUNCTIONS_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  auxiliaryTestFunctions.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
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

#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/PropertyObjArray.h>
#include "getRSS.h"

#include <fstream>
#include <string>
#include <regex>

template <typename T>
void ASSERT_EQUAL(T expected, 
                  T found, 
                  T tolerance, 
                  std::string file = "", 
                  int line = -1, 
                  std::string message = "") {
    if (found < expected - tolerance || found > expected + tolerance)
        throw OpenSim::Exception(message, file, line);
}
template<int M, typename ELT, int STRIDE>
void ASSERT_EQUAL(const SimTK::Vec<M, ELT, STRIDE>& vecA,
                  const SimTK::Vec<M, ELT, STRIDE>& vecB,
                  const std::string& file = "",
                  int line = -1,
                  const std::string& message = "") {
    try {
        SimTK_TEST_EQ(vecA, vecB);
    } catch(const SimTK::Exception::Assert&) {
        throw OpenSim::Exception(message, file, line);
    }
}
template<int M, typename ELT, int STRIDE>
void ASSERT_EQUAL(const SimTK::Vec<M, ELT, STRIDE>& vecA,
                  const SimTK::Vec<M, ELT, STRIDE>& vecB,
                  double tolerance,
                  const std::string& file = "",
                  int line = -1,
                  const std::string& message = "") {
    try {
        SimTK_TEST_EQ_TOL(vecA, vecB, tolerance);
    } catch(const SimTK::Exception::Assert&) {
        throw OpenSim::Exception(message, file, line);
    }
}


inline void ASSERT(bool cond, 
                   std::string file="", 
                   int line=-1, 
                   std::string message="Exception") {
    if (!cond) throw OpenSim::Exception(message, file, line);
}
/**
 * Check this storage object against a standard storage object using the
 * specified tolerances. If RMS error for any column is outside the
 * tolerance, throw an Exception.
 */
void CHECK_STORAGE_AGAINST_STANDARD(const OpenSim::Storage& result, 
                                    const OpenSim::Storage& standard, 
                                    const std::vector<double>& tolerances, 
                                    const std::string& testFile, 
                                    const int testFileLine, 
                                    const std::string& errorMessage)
{
    std::vector<std::string> columnsUsed;
    std::vector<double> comparisons;
    result.compareWithStandard(standard, columnsUsed, comparisons);

    auto ncolumns = columnsUsed.size();

    ASSERT(ncolumns > 0, testFile, testFileLine, 
           errorMessage + "- no common columns to compare!");

    for (int i = 0; i < ncolumns; ++i) {
        std::cout << "column:    " << columnsUsed[i] << std::endl;
        std::cout << "RMS error: " << comparisons[i] << std::endl;
        std::cout << "tolerance: " << tolerances[i] << std::endl << std::endl;
        ASSERT(comparisons[i] < tolerances[i], testFile, testFileLine, 
               errorMessage);
    }
}

// Informed by googletest.
#define ASSERT_THROW(EXPECTED_EXCEPTION, STATEMENT) \
do { \
    bool caughtExpectedException = false; \
    try { \
        STATEMENT; \
    } \
    catch (EXPECTED_EXCEPTION const&) { \
        caughtExpectedException = true; \
    } \
    catch (...) { \
        throw OpenSim::Exception("TESTING: Expected exception " \
            #EXPECTED_EXCEPTION " but caught different exception."); \
    } \
    if (!caughtExpectedException) { \
        throw OpenSim::Exception("TESTING: Expected exception " \
            #EXPECTED_EXCEPTION " but no exception thrown."); \
    } \
} while(false) 

OpenSim::Object* randomize(OpenSim::Object* obj)
{
    using namespace OpenSim;
    using namespace std;

    if (obj==nullptr) return 0; // maybe empty tag
    std::stringstream stream;
    stream << rand();
    obj->setName(obj->getConcreteClassName()+stream.str());
    // Cycle thru properties and based on type, populate with a random valid 
    // value.
     for (int p=0; p < obj->getNumProperties(); ++p) {
        AbstractProperty& ap = obj->updPropertyByIndex(p); 
        //cout << ap.getName() << "=" << ap.toString() << endl;
        // Check return values from Property API for debugging purposes
        bool isList = ap.isListProperty();
        // bool t2 = ap.isObjectProperty();
        // bool t3 = ap.isOneObjectProperty();
        // bool t4 = ap.isOneValueProperty();
        string ts = ap.getTypeName();
        //cout << ts << endl;
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
            string base("ABCXYZ");
            if (isList){
                stringstream val;
                val << base << "_" << ap.size();
                ap.appendValue<string>(val.str());
            }
            else{
                ap.updValue<string>() = base;
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
                prop = LinearFunction();
            }
            
        } else if (ap.isObjectProperty() && !isList) {
            randomize(&ap.updValueAsObject(0));
            ap.setValueIsDefault(false);
        } else {
            //cerr << "Unrecognized Property:"<< ap.getName()<< ":" 
            //     << ap.toString() << endl;
        }
     }
     return obj;
}

// Change version number of the file to 1 so that Storage can read it.
// Storage can only read files with version <= 1.
// This function can be removed when Storage class is removed.
inline void revertToVersionNumber1(const std::string& filenameOld,
    const std::string& filenameNew) {
    std::regex versionline{ R"([ \t]*version[ \t]*=[ \t]*\d[ \t]*)" };
    std::ifstream fileOld{ filenameOld };
    std::ofstream fileNew{ filenameNew };
    std::string line{};
    while (std::getline(fileOld, line)) {
        if (std::regex_match(line, versionline))
            fileNew << "version=1\n";
        else
            fileNew << line << "\n";
    }
}

// Estimate the memory usage of a *creator* that heap allocates an object
// of type C and returns a pointer to it. Creator can also perform any 
// initialization before returning the pointer.
template <typename C, typename T>
size_t estimateMemoryChangeForCreator(T creator, const size_t nSamples = 100)
{
    std::vector<std::unique_ptr<C>> pointers;
    std::vector<size_t> deltas;

    for (size_t i = 0; i < nSamples; ++i) {
        size_t mem0 = getCurrentRSS();
        // Execute the desired creator 
        // store in unique_ptrs to delay deletion
        pointers.push_back(std::unique_ptr<C>(creator()));
        // poll the change in memory usage
        size_t mem1 = getCurrentRSS();
        // store change in memory usage (negative values are invalid)
        deltas.push_back(mem1 > mem0 ? mem1 - mem0 : 0);
    }

    OPENSIM_THROW_IF(deltas.size() < 2, OpenSim::Exception,
        "Insufficient number of nonzero samples to estimate memory change. "
        "Consider increasing the number of samples.");

    size_t nmedian = deltas.size() / 2;
    // sort the deltas up to and including the nth element
    std::nth_element(deltas.begin(), deltas.begin() + nmedian, deltas.end());

    return deltas[nmedian];
}

// Determine if getRSS is providing reliable estimates of memory usage by
// testing against an allocation of known size and verifying that the change
// memory use is detected. Employ this method to validate the use of
// memory use estimators (e.g. estimateMemoryChangeForCreator and
// estimateMemoryChangeForCommand). Do this at the beginning of your test 
// involving checks for memory use.
void validateMemoryUseEstimates()
{
    size_t nSamples = 5;
    size_t size = 20 * 1024; // 20 * 1KB;

    auto creator = [size]() { 
        char* block =  new char[size];
        // initialize the block to ensure it is now allocated
        for (size_t i = 0; i < size; ++i) {
            block[i] = rand();
        }
        return block;
    };

    size_t delta = estimateMemoryChangeForCreator<char>(creator, nSamples);

    std::cout << "delta = " << delta << " expected: " << size << std::endl;

    OPENSIM_THROW_IF(delta < size/2, OpenSim::Exception,
        "Cannot estimate memory usage due to invalid getRSS() evaluation.");
}

// Estimate the change in memory usage resulting from executing a command
template <typename T>
size_t estimateMemoryChangeForCommand(T command, const size_t nSamples = 100)
{
    std::vector<size_t> deltas;

    for (size_t i = 0; i < nSamples; ++i) {
        size_t mem0 = getCurrentRSS();
        // Execute the desired command
        command();
        // wait a millisecond to allow memory usage to settle
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // poll the change in memory usage
        size_t mem1 = getCurrentRSS();
        // store change in memory usage (negative values are invalid)
        deltas.push_back(mem1 > mem0 ? mem1 - mem0 : 0);
    }

    size_t nmedian = deltas.size() / 2;
    // sort the deltas up to and including the nth element
    std::nth_element(deltas.begin(), deltas.begin() + nmedian, deltas.end());

    return deltas[nmedian];
}


#endif // OPENSIM_AUXILIARY_TEST_FUNCTIONS_H_
