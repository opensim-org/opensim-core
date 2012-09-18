#ifndef _osimCommonTemplates_h_
#define _osimCommonTemplates_h_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  osimCommonTemplates.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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


// NOTES:
// This header file should not be included in any file that is a part of
// the osimCommon library.  If it is included in files in the osimCommon project,
// templates will be instantiated multiple times.
//
// Other projects, such as libraries other than osimCommon or executables, should
// include this header file to import the template classes below.
//


// INCLUDES
#include "osimCommonDLL.h"
#include <string>
#include "Array.h"
#include "ArrayPtrs.h"
#include "Set.h"


#ifdef WIN32

extern template class OSIMCOMMON_API Array<bool>;
extern template class OSIMCOMMON_API Array<int>;
extern template class OSIMCOMMON_API Array<double>;
extern template class OSIMCOMMON_API Array<std::string>;

#endif  // WIN32


#endif  // __osimCommonTemplates_h__
