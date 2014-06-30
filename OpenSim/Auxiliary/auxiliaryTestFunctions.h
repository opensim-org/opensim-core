#ifndef __auxiliaryTestFunctions_h__
#define __auxiliaryTestFunctions_h__
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  auxiliaryTestFunctions.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include "getRSS.h"

using namespace OpenSim;
using namespace std;

template <typename T>
void ASSERT_EQUAL(T expected, T found, T tolerance, std::string file="", int line=-1, std::string message="") {
	if (found < expected - tolerance || found > expected + tolerance)
		throw OpenSim::Exception(message, file, line);
}
inline void ASSERT(bool cond, std::string file="", int line=-1, std::string message="Exception") {
	if (!cond) throw OpenSim::Exception(message, file, line);
}
/**
 * Check this storage object against a standard storage object using the
 * specified tolerances. If RMS error for any column is outside the
 * tolerance, throw an Exception.
 */
void CHECK_STORAGE_AGAINST_STANDARD(OpenSim::Storage& result, OpenSim::Storage& standard, OpenSim::Array<double> tolerances, std::string testFile, int testFileLine, std::string errorMessage)
{
	OpenSim::Array<std::string> columnsUsed;
	OpenSim::Array<double> comparisons;
	result.compareWithStandard(standard, columnsUsed, comparisons);

	int ncolumns = columnsUsed.getSize();

	ASSERT(ncolumns > 0, testFile, testFileLine, errorMessage + "- no common columns to compare!");

	for (int i = 0; i < ncolumns; ++i) {
		std::cout << "column:    " << columnsUsed[i] << std::endl;
		std::cout << "RMS error: " << comparisons[i] << std::endl;
		std::cout << "tolerance: " << tolerances[i] << std::endl << std::endl;
		ASSERT(comparisons[i] < tolerances[i], testFile, testFileLine, errorMessage);
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

static OpenSim::Object* randomize(OpenSim::Object* obj)
{
    if (obj==0) return 0; // maybe empty tag
    std::stringstream stream;
    stream << rand();
    obj->setName(obj->getConcreteClassName()+stream.str());
    // Cycle thru properties and based on type, populate with a random valid value
     for (int p=0; p < obj->getNumProperties(); ++p) {
        AbstractProperty& ap = obj->updPropertyByIndex(p); 
        //cout << ap.getName() << "=" << ap.toString() << endl;
        // Check return values from Property API for debugging purposes
        bool t1 = ap.isListProperty();
        bool t2 = ap.isObjectProperty();
        bool t3 = ap.isOneObjectProperty();
        bool t4 = ap.isOneValueProperty();
        string ts = ap.getTypeName();
        cout << ts << endl;
        if (ap.isOptionalProperty()) 
            continue;
        if (ts == "bool"&& !ap.isListProperty()) ap.updValue<bool>()= !ap.getValue<bool>();
        else if (ts == "integer"&& !ap.isListProperty()) ap.updValue<int>()=rand();
        else if (ts == "double" && !ap.isListProperty()) ap.updValue<double>()=(double) rand();
        else if (ts == "double" && ap.isListProperty() && ap.getMaxListSize() < 20){
            for (int i=0; i< ap.getMaxListSize(); ++i)
                ap.updValue<double>(i) = (double) rand();
        }
		else if (ap.isObjectProperty() && !ap.isListProperty()){
			randomize(&ap.updValueAsObject(0));
			ap.setValueIsDefault(false);
		}
		else{
			cerr << "Unrecognized Property" << std::endl;
			cerr << ap.getName() << ": " << ap.toString() << std::endl;
		}
     }
     return obj;
}

#endif // __auxiliaryTestFunctions_h__
