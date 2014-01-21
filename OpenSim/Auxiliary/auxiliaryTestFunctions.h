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

	int columns = columnsUsed.getSize();
	for (int i = 0; i < columns; ++i) {
		std::cout << "column:    " << columnsUsed[i] << std::endl;
		std::cout << "RMS error: " << comparisons[i] << std::endl;
		std::cout << "tolerance: " << tolerances[i] << std::endl << std::endl;
		ASSERT(comparisons[i] < tolerances[i], testFile, testFileLine, errorMessage);
	}
}

#endif // __auxiliaryTestFunctions_h__
