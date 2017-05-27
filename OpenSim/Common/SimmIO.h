#ifndef __SimmIO_h__
#define __SimmIO_h__
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  SimmIO.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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


// INCLUDE
#include "osimCommonDLL.h"
#include <iostream>
#include <string>
#include "SimTKcommon/SmallMatrix.h"

namespace OpenSim {

bool OSIMCOMMON_API readNonCommentStringFromStream(std::istream &aStream, std::string &rBuffer);
bool OSIMCOMMON_API readStringFromStream(std::istream &aStream, std::string &rBuffer);
bool OSIMCOMMON_API readStringFromString(std::string &aString, std::string &rBuffer);
bool OSIMCOMMON_API readTabDelimitedStringFromString(std::string &aString, std::string &rBuffer);
bool OSIMCOMMON_API readIntegerFromString(std::string &aString, int *rNumber);
bool OSIMCOMMON_API readDoubleFromString(std::string &aString, double *rNumber, bool allowNaNs=false);
bool OSIMCOMMON_API readVectorFromString(std::string &aString, SimTK::Vec3 &rVec);
bool OSIMCOMMON_API readVectorFromString(std::string &aString, double *rVX, double *rVY, double *rVZ);
bool OSIMCOMMON_API readCoordinatesFromString(std::string &aString, double rVec[3], bool allowNaNs=false);
int OSIMCOMMON_API findFirstNonWhiteSpace(std::string &aString);
int OSIMCOMMON_API findFirstWhiteSpace(std::string &aString);
void OSIMCOMMON_API convertString(std::string& aString, bool aPrependUnderscore);
std::string OSIMCOMMON_API getCurrentTimeString();

} // end of namespace OpenSim

#endif // __SimmIO_h__


