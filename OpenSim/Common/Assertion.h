#ifndef OPENSIM_ASSERTION_H_
#define OPENSIM_ASSERTION_H_

/* ------------------------------------------------------------------------- *
*                           OpenSim:  Exception.h                            *
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

#include "osimCommonDLL.h"

namespace OpenSim {

    class Object;

    [[noreturn]] OSIMCOMMON_API void OnAssertionError(
        char const* failingCode,
        char const* failingFile,
        char const* failingFunction,
        unsigned int failingLine,
        Object const* maybeSourceObject = nullptr
    );
}

#define OPENSIM_ASSERT_ALWAYS(CONDITION) \
    static_cast<bool>(CONDITION) ? static_cast<void>(0) : OpenSim::OnAssertionError(#CONDITION, __FILE__, __func__, __LINE__)
#define OPENSIM_ASSERT_FRMOBJ_ALWAYS(CONDITION) \
    static_cast<bool>(CONDITION) ? static_cast<void>(0) : OpenSim::OnAssertionError(#CONDITION, __FILE__, __func__, __LINE__, this)

#if defined(NDEBUG)
#define OPENSIM_ASSERT(CONDITION)
#define OPENSIM_ASSERT_FRMOBJ(CONDITION)
#else
#define OPENSIM_ASSERT(CONDITION) OPENSIM_ASSERT_ALWAYS(CONDITION)
#define OPENSIM_ASSERT_FRMOBJ(CONDITION) OPENSIM_ASSERT_FRMOBJ_ALWAYS(CONDITION)
#endif

#endif // OPENSIM_ASSERTION_H_
