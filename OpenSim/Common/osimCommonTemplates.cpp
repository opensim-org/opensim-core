/* -------------------------------------------------------------------------- *
 *                     OpenSim:  osimCommonTemplates.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */
#define OSIMCOMMONTEMPLATES


// INCLUDES
#include "osimCommonDLL.h"
#include <string>
#include "Array.h"
#include "ArrayPtrs.h"
#include "Set.h"


using namespace OpenSim;
#ifdef _WIN32

//template class OSIMCOMMON_API Array<bool>;
//template class OSIMCOMMON_API Array<int>;
//template class OSIMCOMMON_API Array<double>;
//template class OSIMCOMMON_API Array<std::string>;

//template class OSIMCOMMON_API Set<Material>;

#endif  // _WIN32


typedef Array<bool> ArrayBool;
typedef Array<int> ArrayInt;
typedef Array<double> ArrayDbl;
typedef Array<std::string> ArrayStr;

